//! Attitude controller: rate-limited PID that outputs a wheel torque command.
//!
//! torque = Kd * (rate_cmd - rate) + Ki * integral(error)
//! rate_cmd = clamp(Kp/Kd * error, +-max_rate)
//! For small errors this is ordinary PID; for big slews the rate is capped so
//! the star tracker (which loses lock above ~2 deg/s) keeps tracking.
//!
//! Sign convention: positive wheel torque spins the BODY negative (reaction),
//! so the controller asks the wheel for the negative of the body torque it
//! wants.
use crate::params::*;
use crate::wrap_pi;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    /// Wheel torque zero.
    Idle,
    /// Damp body rate to zero (what you do right after separation).
    RateDamp,
    /// Hold or slew to a target angle [rad].
    Point(f32),
}

pub struct Controller {
    pub mode: Mode,
    kp: f32,
    kd: f32,
    ki: f32,
    integral: f32,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    pub fn new() -> Self {
        let kp = J_BODY * CTRL_WN * CTRL_WN;
        let kd = 2.0 * CTRL_ZETA * CTRL_WN * J_BODY;
        Controller {
            mode: Mode::Idle,
            kp,
            kd,
            ki: kp * CTRL_WN / 10.0,
            integral: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: Mode) {
        if mode != self.mode {
            self.integral = 0.0;
        }
        self.mode = mode;
    }

    /// Returns the WHEEL torque command [N m].
    pub fn update(&mut self, theta: f32, rate: f32, dt: f32) -> f32 {
        let body_torque = match self.mode {
            Mode::Idle => 0.0,
            Mode::RateDamp => -self.kd * rate,
            Mode::Point(target) => {
                let err = wrap_pi(target - theta);
                let rate_cmd = (self.kp / self.kd * err).clamp(-CTRL_MAX_RATE, CTRL_MAX_RATE);
                // Integrate only near the target (anti-windup during slews).
                if err.abs() < 0.05 {
                    self.integral += err * dt;
                    let lim = 0.5 * WHEEL_MAX_TORQUE / self.ki;
                    self.integral = self.integral.clamp(-lim, lim);
                } else {
                    self.integral = 0.0;
                }
                self.kd * (rate_cmd - rate) + self.ki * self.integral
            }
        };
        (-body_torque).clamp(-WHEEL_MAX_TORQUE, WHEEL_MAX_TORQUE)
    }
}
