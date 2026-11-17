//! Two-state Kalman filter: attitude angle and gyro bias.
//!
//! This is the 1-D version of what every star-tracker + gyro spacecraft runs
//! (the 3-D version is the "multiplicative EKF", MEKF):
//! - Propagate at the gyro rate: integrate (gyro - bias_estimate).
//! - Update at the star tracker rate: the tracker gives absolute attitude,
//!   which corrects the angle AND, over time, reveals the gyro bias.
//! - Between fixes (sun in the field of view, high rates), the filter coasts
//!   on the gyro and its uncertainty P grows.
use crate::wrap_pi;

const BIAS_PRIOR_VAR: f32 = 1.0e-4;

pub struct Estimator {
    /// State: [theta, bias].
    pub theta: f32,
    pub bias: f32,
    /// Covariance, row-major 2x2.
    pub p: [[f32; 2]; 2],
    /// Gyro white noise density [rad/s/sqrt(Hz)] and bias random walk.
    q_gyro: f32,
    q_bias: f32,
    pub initialized: bool,
    pub updates: u32,
    pub rejected: u32,
    last_rate: f32,
}

impl Estimator {
    pub fn new(gyro_noise_per_sample: f32, gyro_dt: f32) -> Self {
        Estimator {
            theta: 0.0,
            bias: 0.0,
            p: [[10.0, 0.0], [0.0, BIAS_PRIOR_VAR]],
            // Convert per-sample noise into a spectral density.
            q_gyro: gyro_noise_per_sample * gyro_noise_per_sample * gyro_dt,
            q_bias: 1.0e-12,
            initialized: false,
            updates: 0,
            rejected: 0,
            last_rate: 0.0,
        }
    }

    /// Bias-corrected body rate from the latest gyro sample [rad/s].
    pub fn rate(&self) -> f32 {
        self.last_rate
    }

    /// 1-sigma angle uncertainty [rad].
    pub fn sigma_theta(&self) -> f32 {
        libm::sqrtf(self.p[0][0])
    }

    /// Propagate with one gyro sample.
    pub fn propagate(&mut self, gyro: f32, dt: f32) {
        self.last_rate = gyro - self.bias;
        self.theta = wrap_pi(self.theta + self.last_rate * dt);
        // F = [[1, -dt], [0, 1]];  P = F P F^T + Q
        if !self.initialized {
            return; // nothing to be uncertain about until the first fix
        }
        let [[a, b], [_, d]] = self.p;
        let a2 = a - 2.0 * dt * b + dt * dt * d + self.q_gyro * dt;
        let b2 = b - dt * d;
        let d2 = d + self.q_bias * dt;
        self.p = [[a2, b2], [b2, d2]];
    }

    /// Update with a star tracker angle measurement and its variance.
    /// Returns false if the measurement failed the 3-sigma innovation gate.
    pub fn update(&mut self, theta_meas: f32, r: f32) -> bool {
        self.update_at(theta_meas, r, 0.0)
    }

    /// Update with a measurement taken `dt_meas` seconds after the filter's
    /// current state time (negative if older). The innovation compares the
    /// measurement with the state predicted to the measurement's own time.
    /// Ignoring this costs rate x latency: 10 ms at 1.7 deg/s is ~60 arcsec.
    pub fn update_at(&mut self, theta_meas: f32, r: f32, dt_meas: f32) -> bool {
        if !self.initialized {
            // First fix: take the angle outright and reset the covariance.
            // Keep the bias prior (1-sigma 0.01 rad/s) and no cross-terms;
            // whatever P accumulated before now is meaningless.
            self.theta = theta_meas;
            self.p = [[r, 0.0], [0.0, BIAS_PRIOR_VAR]];
            self.initialized = true;
            return true;
        }
        let predicted = self.theta + self.last_rate * dt_meas;
        let y = wrap_pi(theta_meas - predicted);
        let s = self.p[0][0] + r;
        if y * y / s > 9.0 {
            self.rejected += 1;
            return false;
        }
        let k0 = self.p[0][0] / s;
        let k1 = self.p[1][0] / s;
        self.theta = wrap_pi(self.theta + k0 * y);
        self.bias += k1 * y;
        // P = (I - K H) P with H = [1, 0]
        let [[a, b], [_, d]] = self.p;
        // (I - K H) P, written out and kept symmetric so f32 rounding cannot
        // push it non-positive (a classic filter divergence cause).
        let a2 = (1.0 - k0) * a;
        let b2 = (1.0 - k0) * b;
        let d2 = (d - k1 * b).max(1e-14);
        self.p = [[a2, b2], [b2, d2]];
        self.updates += 1;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    #[test]
    fn learns_gyro_bias_while_stationary() {
        let true_bias = 0.002;
        let dt = 0.02;
        let mut est = Estimator::new(0.0005, dt);
        let mut rng = Rng::new(7);
        for k in 0..(120.0 / dt) as u32 {
            est.propagate(true_bias + 0.0005 * rng.gauss(), dt);
            if k % 10 == 0 {
                est.update(1.0 + 1e-4 * rng.gauss(), 1e-8);
            }
        }
        assert!((est.bias - true_bias).abs() < 2e-4, "bias {}", est.bias);
        assert!(wrap_pi(est.theta - 1.0).abs() < 1e-3);
    }

    #[test]
    fn uncertainty_grows_while_coasting() {
        let mut est = Estimator::new(0.0005, 0.02);
        est.update(0.0, 1e-8);
        let s0 = est.sigma_theta();
        for _ in 0..500 {
            est.propagate(0.0, 0.02);
        }
        assert!(est.sigma_theta() > 10.0 * s0);
    }
}
