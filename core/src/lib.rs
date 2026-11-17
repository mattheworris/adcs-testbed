//! Shared, `no_std` building blocks for the single-axis ADCS testbed.
//!
//! The same code runs in three places:
//! - on the STM32, inside the device emulator (`emulator`, `dynamics`)
//! - on the BeagleBone Black, inside the flight software (`proto`,
//!   `estimator`, `controller`)
//! - on your laptop / in CI, in unit tests and the closed-loop SIL test
#![cfg_attr(not(test), no_std)]

pub mod controller;
pub mod crc;
pub mod dynamics;
pub mod emulator;
pub mod estimator;
pub mod params;
pub mod proto;
pub mod quat;
pub mod rng;

/// Wrap an angle to (-pi, pi].
pub fn wrap_pi(a: f32) -> f32 {
    use core::f32::consts::PI;
    let mut x = a % (2.0 * PI);
    if x > PI {
        x -= 2.0 * PI;
    } else if x <= -PI {
        x += 2.0 * PI;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::wrap_pi;
    use core::f32::consts::PI;

    #[test]
    fn wrap_keeps_range() {
        assert!((wrap_pi(3.0 * PI / 2.0) - (-PI / 2.0)).abs() < 1e-5);
        assert!((wrap_pi(-3.0 * PI / 2.0) - (PI / 2.0)).abs() < 1e-5);
        assert!((wrap_pi(0.25) - 0.25).abs() < 1e-7);
    }
}
