//! Minimal unit quaternion, scalar-first (w, x, y, z), Hamilton convention.
//!
//! Convention matters: cflib sends (x, y, z, w), many star trackers send
//! scalar-first, and JPL-convention quaternions multiply in the opposite
//! order. Every interface control document (ICD) states its convention; ours
//! is "scalar-first Hamilton, body-to-inertial".
use libm::{atan2f, cosf, sinf, sqrtf};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Rotation of `angle` radians about the body Z axis (our control axis).
    pub fn from_z_angle(angle: f32) -> Quat {
        let h = 0.5 * angle;
        Quat {
            w: cosf(h),
            x: 0.0,
            y: 0.0,
            z: sinf(h),
        }
    }

    pub fn conj(self) -> Quat {
        Quat {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn norm(self) -> f32 {
        sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn normalized(self) -> Quat {
        let n = self.norm();
        Quat {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    /// Rotation angle about Z (yaw), valid for any unit quaternion.
    pub fn z_angle(self) -> f32 {
        atan2f(
            2.0 * (self.w * self.z + self.x * self.y),
            1.0 - 2.0 * (self.y * self.y + self.z * self.z),
        )
    }
}

/// Hamilton product: `a * b` applies b first, then a (for body-to-inertial
/// quaternions composed as q_total = q_outer * q_inner).
impl core::ops::Mul for Quat {
    type Output = Quat;
    fn mul(self, r: Quat) -> Quat {
        Quat {
            w: self.w * r.w - self.x * r.x - self.y * r.y - self.z * r.z,
            x: self.w * r.x + self.x * r.w + self.y * r.z - self.z * r.y,
            y: self.w * r.y - self.x * r.z + self.y * r.w + self.z * r.x,
            z: self.w * r.z + self.x * r.y - self.y * r.x + self.z * r.w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Quat;

    #[test]
    fn z_angle_round_trip() {
        for a in [-3.0f32, -1.0, 0.0, 0.5, 2.5] {
            let q = Quat::from_z_angle(a);
            assert!((q.z_angle() - a).abs() < 1e-5, "angle {a}");
            assert!((q.norm() - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn composition_adds_angles() {
        let q = Quat::from_z_angle(0.3) * Quat::from_z_angle(0.4);
        assert!((q.z_angle() - 0.7).abs() < 1e-5);
        let back = q * Quat::from_z_angle(0.4).conj();
        assert!((back.z_angle() - 0.3).abs() < 1e-5);
    }

    #[test]
    fn q_and_minus_q_are_the_same_attitude() {
        let q = Quat::from_z_angle(1.0);
        let m = Quat {
            w: -q.w,
            x: -q.x,
            y: -q.y,
            z: -q.z,
        };
        assert!((q.z_angle() - m.z_angle()).abs() < 1e-5);
    }
}
