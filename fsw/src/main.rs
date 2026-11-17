//! adcs-fsw: single-axis attitude determination and control flight software.
//!
//!   adcs-fsw --port /dev/ttyS4 --ground 192.168.1.50:5005   (HWIL, on the BBB)
//!   adcs-fsw --sim --ground 127.0.0.1:5005                   (SIL, anywhere)
//!
//! Loop: every gyro packet (50 Hz) -> propagate estimator -> controller ->
//! wheel torque command. Every valid star tracker packet (5 Hz) -> estimator
//! update. Telemetry goes out over UDP at 10 Hz; commands come in over UDP
//! as text ("point 1.57", "point-deg 90", "damp", "idle").
mod link;

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use adcs_core::controller::{Controller, Mode};
use adcs_core::estimator::Estimator;
use adcs_core::params::*;
use adcs_core::proto::{st_status, Msg};
use link::{Link, SerialLink, SimLink};

struct Args {
    port: String,
    baud: u32,
    sim: bool,
    ground: String,
    listen: String,
}

fn parse_args() -> Args {
    let mut a = Args {
        port: "/dev/ttyS4".into(),
        baud: 115_200,
        sim: false,
        ground: "127.0.0.1:5005".into(),
        listen: "0.0.0.0:5006".into(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut val = || {
            it.next()
                .unwrap_or_else(|| usage(&format!("{arg} needs a value")))
        };
        match arg.as_str() {
            "--port" => a.port = val(),
            "--baud" => a.baud = val().parse().unwrap_or_else(|_| usage("bad --baud")),
            "--sim" => a.sim = true,
            "--ground" => a.ground = val(),
            "--listen" => a.listen = val(),
            "-h" | "--help" => usage(""),
            other => usage(&format!("unknown argument {other}")),
        }
    }
    a
}

fn usage(msg: &str) -> ! {
    if !msg.is_empty() {
        eprintln!("error: {msg}");
    }
    eprintln!(
        "usage: adcs-fsw [--sim | --port DEV [--baud N]] [--ground HOST:PORT] [--listen ADDR:PORT]"
    );
    std::process::exit(2);
}

fn parse_cmd(s: &str) -> Option<Mode> {
    let mut w = s.split_whitespace();
    match (w.next()?, w.next()) {
        ("idle", _) => Some(Mode::Idle),
        ("damp", _) => Some(Mode::RateDamp),
        ("point", Some(v)) => v.parse().ok().map(Mode::Point),
        ("point-deg", Some(v)) => v.parse::<f32>().ok().map(|d| Mode::Point(d.to_radians())),
        _ => None,
    }
}

fn main() -> std::io::Result<()> {
    let args = parse_args();
    let mut link: Box<dyn Link> = if args.sim {
        eprintln!("adcs-fsw: SIL mode (in-process emulator)");
        Box::new(SimLink::new(0.0, true))
    } else {
        eprintln!("adcs-fsw: HWIL mode on {} @ {}", args.port, args.baud);
        Box::new(SerialLink::open(&args.port, args.baud)?)
    };

    let sock = UdpSocket::bind(&args.listen)?;
    sock.set_nonblocking(true)?;
    eprintln!(
        "adcs-fsw: telemetry -> {}, commands <- {}",
        args.ground, args.listen
    );

    let mut est = Estimator::new(GYRO_NOISE, PLANT_DT * GYRO_DIV as f32);
    let mut ctrl = Controller::new();
    ctrl.set_mode(Mode::RateDamp);

    let mut last_gyro_us: Option<u32> = None;
    let mut last_gyro_at = Instant::now();
    let mut gyro_count: u32 = 0;
    let mut st_status_last = st_status::VALID;
    let (mut wheel_rpm, mut wheel_status) = (0.0f32, 0u8);
    let (mut truth_theta, mut truth_omega, mut truth_t_us) = (f32::NAN, f32::NAN, 0u32);
    let mut accel_x = 0.0f32;
    let mut link_up = false;
    let mut cmd_buf = [0u8; 128];

    loop {
        // ---- ground commands (non-blocking)
        while let Ok((n, from)) = sock.recv_from(&mut cmd_buf) {
            let text = String::from_utf8_lossy(&cmd_buf[..n]);
            match parse_cmd(text.trim()) {
                Some(mode) => {
                    eprintln!("cmd from {from}: {mode:?}");
                    ctrl.set_mode(mode);
                }
                None => eprintln!("cmd from {from}: rejected '{}'", text.trim()),
            }
        }

        // ---- device messages
        let msg = link.recv(Duration::from_millis(50))?;

        // Fault protection: no gyro for 200 ms means we are flying blind.
        // Drop to Idle and command zero torque; the emulator also zeroes the
        // wheel on its own if our commands stop (defense in depth).
        if last_gyro_at.elapsed() > Duration::from_millis(200) && link_up {
            eprintln!("FAULT: gyro data lost, entering Idle");
            ctrl.set_mode(Mode::Idle);
            link_up = false;
            let _ = link.send(&Msg::WheelCmd { torque_nm: 0.0 });
        }

        let Some(msg) = msg else { continue };
        match msg {
            Msg::Gyro { t_us, rate } => {
                let dt = match last_gyro_us {
                    Some(prev) => t_us.wrapping_sub(prev) as f32 * 1e-6,
                    None => 0.0,
                };
                last_gyro_us = Some(t_us);
                last_gyro_at = Instant::now();
                link_up = true;
                gyro_count = gyro_count.wrapping_add(1);

                est.propagate(rate[2], dt);
                let torque_cmd = if est.initialized {
                    ctrl.update(est.theta, est.rate(), dt)
                } else {
                    0.0 // no attitude yet: do not move the wheel
                };
                link.send(&Msg::WheelCmd {
                    torque_nm: torque_cmd,
                })?;

                // 10 Hz telemetry: every 5th gyro packet.
                if gyro_count % 5 == 0 {
                    let target = match ctrl.mode {
                        Mode::Point(t) => t,
                        _ => f32::NAN,
                    };
                    let (ok, crc, bad) = link.stats();
                    // Truth arrives at 10 Hz on a different tick than this
                    // gyro sample. Propagate it to THIS sample's timestamp,
                    // or a 1.7 deg/s slew shows ~180 arcsec of fake error.
                    let age = t_us.wrapping_sub(truth_t_us) as f32 * 1e-6;
                    let truth_now = adcs_core::wrap_pi(truth_theta + truth_omega * age);
                    let line = format!(
                        "{t},{th:.6},{rt:.6},{b:.6},{sig:.6},{tg:.6},{rpm:.1},{tq:.6},{st},{ws},{tt:.6},{to:.6},{ax:.3},{ok},{crc},{bad},{mode}",
                        t = t_us as f64 * 1e-6,
                        th = est.theta,
                        rt = est.rate(),
                        b = est.bias,
                        sig = est.sigma_theta(),
                        tg = target,
                        rpm = wheel_rpm,
                        tq = torque_cmd,
                        st = st_status_last,
                        ws = wheel_status,
                        tt = truth_now,
                        to = truth_omega,
                        ax = accel_x,
                        mode = match ctrl.mode {
                            Mode::Idle => "IDLE",
                            Mode::RateDamp => "DAMP",
                            Mode::Point(_) => "POINT",
                        },
                    );
                    let _ = sock.send_to(line.as_bytes(), &args.ground);
                }
            }
            Msg::StarTracker {
                t_us, q, status, ..
            } => {
                st_status_last = status;
                // Sanity check before trusting a quaternion off the wire.
                if status == st_status::VALID && (q.norm() - 1.0).abs() < 1e-3 {
                    // The fix is time-stamped; our state is as of the last
                    // gyro sample. Compare them at the same instant.
                    let dt_meas = match last_gyro_us {
                        Some(g) => t_us.wrapping_sub(g) as i32 as f32 * 1e-6,
                        None => 0.0,
                    };
                    est.update_at(q.z_angle(), ST_NOISE * ST_NOISE, dt_meas);
                }
            }
            Msg::WheelTlm {
                speed_rpm, status, ..
            } => {
                wheel_rpm = speed_rpm;
                wheel_status = status;
            }
            Msg::Truth { t_us, theta, omega } => {
                truth_theta = theta;
                truth_omega = omega;
                truth_t_us = t_us;
            }
            Msg::Accel { g, .. } => accel_x = g[0],
            Msg::WheelCmd { .. } => {} // we send these, never receive them
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ground_commands() {
        assert_eq!(parse_cmd("idle"), Some(Mode::Idle));
        assert_eq!(parse_cmd("damp"), Some(Mode::RateDamp));
        assert_eq!(parse_cmd("point 1.5"), Some(Mode::Point(1.5)));
        assert_eq!(
            parse_cmd("point-deg 180"),
            Some(Mode::Point(core::f32::consts::PI))
        );
        assert_eq!(parse_cmd("point"), None);
        assert_eq!(parse_cmd("launch"), None);
    }
}
