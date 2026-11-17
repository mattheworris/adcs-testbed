//! The "spacecraft components" as one device: plant + gyro + star tracker +
//! reaction wheel. Call `tick()` every PLANT_DT seconds; it returns encoded
//! frames to send. Feed received frames to `on_msg()`.
//!
//! The same struct runs on the STM32 (HWIL) and inside the flight software's
//! `--sim` mode (SIL), so both paths exercise identical device behavior and
//! the identical byte protocol.
use crate::dynamics::Plant;
use crate::params::*;
use crate::proto::{st_status, wheel_status, Msg, MAX_FRAME};
use crate::quat::Quat;
use crate::rng::Rng;
use crate::wrap_pi;

/// Room for the most frames that can be emitted in one tick.
pub const TICK_OUT_MAX: usize = 3 * MAX_FRAME;

pub struct Emulator {
    pub plant: Plant,
    rng: Rng,
    tick: u32,
    t_us: u32,
    torque_cmd: f32,
    last_cmd_us: u32,
    pub cmds_rx: u32,
    /// Status of the last star tracker solution (see proto::st_status).
    pub last_st_status: u8,
    /// Extra constant disturbance [N m] (tests; the STM32 uses tilt instead).
    pub bias_disturbance: f32,
}

impl Emulator {
    pub fn new(seed: u32, theta0: f32) -> Self {
        Emulator {
            plant: Plant::new(theta0),
            rng: Rng::new(seed),
            tick: 0,
            t_us: 0,
            torque_cmd: 0.0,
            last_cmd_us: 0,
            cmds_rx: 0,
            last_st_status: st_status::VALID,
            bias_disturbance: 0.0,
        }
    }

    pub fn time_us(&self) -> u32 {
        self.t_us
    }

    pub fn on_msg(&mut self, m: &Msg) {
        if let Msg::WheelCmd { torque_nm } = *m {
            // Reject NaN/inf: a corrupted command must never reach the motor.
            if torque_nm.is_finite() {
                self.torque_cmd = torque_nm;
                self.last_cmd_us = self.t_us;
                self.cmds_rx += 1;
            }
        }
    }

    /// True when the flight computer has gone quiet (wheel torque forced to 0).
    pub fn cmd_timed_out(&self) -> bool {
        self.t_us.wrapping_sub(self.last_cmd_us) > CMD_TIMEOUT_US
    }

    /// Disturbance torque from board tilt (accel X in g).
    pub fn tilt_torque(accel_x_g: f32) -> f32 {
        if accel_x_g.abs() < DIST_DEADBAND_G {
            0.0
        } else {
            DIST_PER_G * accel_x_g
        }
    }

    /// Advance one step. `accel_g` is the real accelerometer reading (or
    /// zeros). Writes frames into `out`, returns the number of bytes.
    ///
    /// Messages are staggered across ticks so no tick sends more than a gyro
    /// frame plus one other: at 115200 baud that is under 5 ms of UART time.
    pub fn tick(&mut self, accel_g: [f32; 3], out: &mut [u8; TICK_OUT_MAX]) -> usize {
        let torque = if self.cmd_timed_out() {
            0.0
        } else {
            self.torque_cmd
        };
        let dist = Self::tilt_torque(accel_g[0]) + self.bias_disturbance;
        self.plant.step(PLANT_DT, torque, dist);
        self.tick = self.tick.wrapping_add(1);
        self.t_us = self.t_us.wrapping_add((PLANT_DT * 1e6) as u32);

        let mut n = 0;
        let k = self.tick;
        let t_us = self.t_us;

        if k % GYRO_DIV == 0 {
            let r = |rng: &mut Rng| GYRO_NOISE * rng.gauss();
            let z = self.plant.omega + GYRO_BIAS_TRUE + r(&mut self.rng);
            let rate = [r(&mut self.rng), r(&mut self.rng), z];
            n += Msg::Gyro { t_us, rate }.encode(&mut out[n..]);
        }
        // Phases: star tracker on k%20==1, wheel on k%10==3, accel on k%10==5,
        // truth on k%10==7. All odd, so they never coincide with each other.
        if k % ST_DIV == 1 {
            n += self.star_tracker(t_us).encode(&mut out[n..]);
        } else if k % TLM_DIV == 3 {
            let mut status = 0;
            if self.plant.wheel_speed.abs() >= WHEEL_MAX_SPEED {
                status |= wheel_status::SATURATED;
            }
            if self.cmd_timed_out() {
                status |= wheel_status::CMD_TIMEOUT;
            }
            let speed_rpm = self.plant.wheel_speed * 60.0 / (2.0 * core::f32::consts::PI);
            let torque_nm = self.plant.wheel_torque;
            n += Msg::WheelTlm {
                t_us,
                speed_rpm,
                torque_nm,
                status,
            }
            .encode(&mut out[n..]);
        } else if k % TLM_DIV == 5 {
            n += Msg::Accel { t_us, g: accel_g }.encode(&mut out[n..]);
        } else if k % TLM_DIV == 7 {
            let (theta, omega) = (self.plant.theta, self.plant.omega);
            n += Msg::Truth { t_us, theta, omega }.encode(&mut out[n..]);
        }
        n
    }

    fn star_tracker(&mut self, t_us: u32) -> Msg {
        let theta = self.plant.theta;
        let status = if self.plant.omega.abs() > ST_MAX_RATE {
            st_status::RATE_TOO_HIGH
        } else if wrap_pi(theta - ST_SUN_ANGLE).abs() < ST_SUN_HALF_WIDTH {
            st_status::SUN_BLIND
        } else {
            st_status::VALID
        };
        self.last_st_status = status;
        if status != st_status::VALID {
            return Msg::StarTracker {
                t_us,
                q: Quat::IDENTITY,
                status,
                n_stars: 0,
            };
        }
        let meas = theta + ST_NOISE * self.rng.gauss();
        let n_stars = 8 + (self.rng.next_u32() % 10) as u8;
        Msg::StarTracker {
            t_us,
            q: Quat::from_z_angle(meas),
            status,
            n_stars,
        }
    }
}
