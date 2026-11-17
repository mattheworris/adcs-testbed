//! Single-axis rigid body with one reaction wheel.
//!
//! Conservation of angular momentum about the axis:
//!   J_body * d(omega)/dt   = -tau_wheel + tau_disturbance
//!   J_wheel * d(Omega)/dt  = +tau_wheel
//! Spinning the wheel up one way spins the body the other way. External
//! disturbances can only be absorbed by changing wheel speed, so the wheel
//! slowly fills with momentum until it saturates; then you need an external
//! torque (magnetorquers or thrusters) to "dump" it.
use crate::params::*;
use crate::wrap_pi;

#[derive(Clone, Copy, Debug, Default)]
pub struct Plant {
    /// Body angle about Z [rad], wrapped to (-pi, pi].
    pub theta: f32,
    /// Body rate [rad/s].
    pub omega: f32,
    /// Wheel speed relative to the body [rad/s].
    pub wheel_speed: f32,
    /// Wheel torque actually applied in the last step [N m] (after limits).
    pub wheel_torque: f32,
}

impl Plant {
    pub fn new(theta: f32) -> Self {
        Plant {
            theta: wrap_pi(theta),
            ..Default::default()
        }
    }

    /// Advance by `dt` seconds with a commanded wheel torque and an external
    /// disturbance torque.
    pub fn step(&mut self, dt: f32, torque_cmd: f32, disturbance: f32) {
        // Motor torque limit.
        let mut tau = torque_cmd.clamp(-WHEEL_MAX_TORQUE, WHEEL_MAX_TORQUE);
        // Speed limit: at max speed the motor cannot push further that way.
        if (self.wheel_speed >= WHEEL_MAX_SPEED && tau > 0.0)
            || (self.wheel_speed <= -WHEEL_MAX_SPEED && tau < 0.0)
        {
            tau = 0.0;
        }
        self.wheel_torque = tau;

        // Semi-implicit Euler: update rates, then angle with the new rate.
        self.wheel_speed =
            (self.wheel_speed + tau / J_WHEEL * dt).clamp(-WHEEL_MAX_SPEED, WHEEL_MAX_SPEED);
        self.omega += (-tau + disturbance) / J_BODY * dt;
        self.theta = wrap_pi(self.theta + self.omega * dt);
    }

    /// Wheel momentum [N m s].
    pub fn wheel_momentum(&self) -> f32 {
        J_WHEEL * self.wheel_speed
    }

    /// Total system momentum about the axis [N m s]. Constant unless a
    /// disturbance acts: a handy invariant for tests. (J_BODY includes the
    /// wheel's mass; the small J_WHEEL * omega coupling term is neglected.)
    pub fn total_momentum(&self) -> f32 {
        J_BODY * self.omega + J_WHEEL * self.wheel_speed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn momentum_is_conserved_without_disturbance() {
        let mut p = Plant::new(0.0);
        let h0 = p.total_momentum();
        for _ in 0..1000 {
            p.step(0.01, 0.001, 0.0);
        }
        assert!((p.total_momentum() - h0).abs() < 1e-6);
        assert!(p.omega < 0.0, "body spins opposite to the wheel");
        assert!(p.wheel_speed > 0.0);
    }

    #[test]
    fn wheel_saturates_under_constant_disturbance() {
        let mut p = Plant::new(0.0);
        // Hold the body still by commanding exactly the disturbance torque.
        for _ in 0..30_000 {
            p.step(0.01, 0.0005, 0.0005);
        }
        assert!((p.wheel_speed - WHEEL_MAX_SPEED).abs() < 1e-3);
        assert_eq!(p.wheel_torque, 0.0, "saturated wheel can no longer help");
    }
}
