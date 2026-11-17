//! Software-in-the-loop (SIL) tests: emulator <-> bytes <-> parser <->
//! estimator + controller, exactly the chain the BeagleBone runs, minus
//! the UART. These run in CI on every push.
use adcs_core::controller::{Controller, Mode};
use adcs_core::emulator::{Emulator, TICK_OUT_MAX};
use adcs_core::estimator::Estimator;
use adcs_core::params::*;
use adcs_core::proto::{st_status, Msg, Parser, MAX_FRAME};
use adcs_core::wrap_pi;

struct Fsw {
    est: Estimator,
    ctrl: Controller,
    last_gyro_us: Option<u32>,
}

impl Fsw {
    fn new() -> Self {
        Fsw {
            est: Estimator::new(GYRO_NOISE, PLANT_DT * GYRO_DIV as f32),
            ctrl: Controller::new(),
            last_gyro_us: None,
        }
    }

    /// Returns a wheel command to send, if any.
    fn on_msg(&mut self, m: &Msg) -> Option<Msg> {
        match *m {
            Msg::Gyro { t_us, rate } => {
                let dt = match self.last_gyro_us {
                    Some(prev) => t_us.wrapping_sub(prev) as f32 * 1e-6,
                    None => 0.0,
                };
                self.last_gyro_us = Some(t_us);
                self.est.propagate(rate[2], dt);
                if !self.est.initialized {
                    return None;
                }
                let tau = self.ctrl.update(self.est.theta, self.est.rate(), dt);
                Some(Msg::WheelCmd { torque_nm: tau })
            }
            Msg::StarTracker {
                t_us, q, status, ..
            } if status == st_status::VALID => {
                let dt = match self.last_gyro_us {
                    Some(g) => t_us.wrapping_sub(g) as i32 as f32 * 1e-6,
                    None => 0.0,
                };
                self.est.update_at(q.z_angle(), ST_NOISE * ST_NOISE, dt);
                None
            }
            _ => None,
        }
    }
}

/// Run `seconds` of closed loop. Returns (emulator, fsw).
fn run(emu: &mut Emulator, fsw: &mut Fsw, seconds: f32) {
    let mut parser = Parser::new();
    let mut out = [0u8; TICK_OUT_MAX];
    for _ in 0..(seconds / PLANT_DT) as u32 {
        let n = emu.tick([0.0, 0.0, 1.0], &mut out);
        for &b in &out[..n] {
            if let Some(m) = parser.push(b) {
                if let Some(cmd) = fsw.on_msg(&m) {
                    // Command goes back over the "wire" too.
                    let mut f = [0u8; MAX_FRAME];
                    let len = cmd.encode(&mut f);
                    let mut back = Parser::new();
                    for &c in &f[..len] {
                        if let Some(cm) = back.push(c) {
                            emu.on_msg(&cm);
                        }
                    }
                }
            }
        }
    }
    assert_eq!(parser.crc_errors, 0);
}

#[test]
fn slews_90_degrees_and_holds() {
    let mut emu = Emulator::new(1, 0.0);
    let mut fsw = Fsw::new();
    fsw.ctrl.set_mode(Mode::Point(core::f32::consts::FRAC_PI_2));
    run(&mut emu, &mut fsw, 200.0);
    let err = wrap_pi(core::f32::consts::FRAC_PI_2 - emu.plant.theta);
    assert!(err.abs() < 1e-3, "pointing error {err} rad");
    assert!(emu.plant.omega.abs() < 1e-4);
    // The estimator found the gyro bias it was never told about.
    assert!(
        (fsw.est.bias - GYRO_BIAS_TRUE).abs() < 2e-4,
        "bias {}",
        fsw.est.bias
    );
}

#[test]
fn rejects_constant_disturbance_until_wheel_fills() {
    let mut emu = Emulator::new(2, 0.0);
    emu.bias_disturbance = 2.0e-5; // 20 uN m, ~ solar pressure scale x10
    let mut fsw = Fsw::new();
    fsw.ctrl.set_mode(Mode::Point(0.5));
    run(&mut emu, &mut fsw, 600.0);
    let err = wrap_pi(0.5 - emu.plant.theta);
    assert!(
        err.abs() < 2e-3,
        "integral action removes steady error: {err}"
    );
    // All that disturbance momentum went into the wheel.
    let h = emu.plant.wheel_momentum().abs();
    assert!(h > 0.8 * 2.0e-5 * 600.0, "wheel momentum {h}");
}

#[test]
fn wheel_saturation_loses_control() {
    let mut emu = Emulator::new(3, 0.0);
    emu.bias_disturbance = 2.0e-4; // 10x bigger: fills the wheel in ~160 s
    let mut fsw = Fsw::new();
    fsw.ctrl.set_mode(Mode::Point(0.0));
    run(&mut emu, &mut fsw, 400.0);
    assert!(emu.plant.wheel_speed.abs() >= WHEEL_MAX_SPEED * 0.999);
    assert!(
        wrap_pi(emu.plant.theta).abs() > 0.1,
        "attitude drifts once saturated"
    );
}

#[test]
fn star_tracker_outage_is_survivable() {
    // Slew through the 120 deg "sun" exclusion zone; the filter must coast on
    // the gyro and settle at 180 deg.
    let mut emu = Emulator::new(4, 0.0);
    let mut fsw = Fsw::new();
    fsw.ctrl.set_mode(Mode::Point(3.0));
    run(&mut emu, &mut fsw, 300.0);
    let err = wrap_pi(3.0 - emu.plant.theta);
    assert!(err.abs() < 1e-3, "pointing error {err}");
}

#[test]
fn wheel_stops_when_commands_stop() {
    let mut emu = Emulator::new(5, 0.0);
    emu.on_msg(&Msg::WheelCmd { torque_nm: 0.001 });
    let mut out = [0u8; TICK_OUT_MAX];
    for _ in 0..100 {
        emu.tick([0.0; 3], &mut out);
    }
    // 1 s later, with no new commands, the wheel torque must be zero.
    assert_eq!(emu.plant.wheel_torque, 0.0);
}
