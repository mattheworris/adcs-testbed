//! Physical and tuning parameters. Small-satellite class numbers, chosen so
//! the dynamics are slow enough to watch on a plot.

/// Spacecraft moment of inertia about the control axis [kg m^2].
pub const J_BODY: f32 = 0.1;

/// Reaction wheel rotor inertia [kg m^2].
pub const J_WHEEL: f32 = 5.0e-5;
/// Maximum wheel motor torque [N m] (2 mN m).
pub const WHEEL_MAX_TORQUE: f32 = 0.002;
/// Maximum wheel speed [rad/s] (6000 rpm). Momentum capacity ~0.031 N m s.
pub const WHEEL_MAX_SPEED: f32 = 6000.0 * 2.0 * core::f32::consts::PI / 60.0;

/// Gyro white noise, 1-sigma per sample [rad/s].
pub const GYRO_NOISE: f32 = 0.0005;
/// Gyro bias the emulator applies [rad/s] (about 0.1 deg/s). The estimator
/// does not know this number; it has to learn it from star tracker fixes.
pub const GYRO_BIAS_TRUE: f32 = 0.0017;

/// Star tracker noise, 1-sigma about the boresight axis [rad] (~20 arcsec).
pub const ST_NOISE: f32 = 1.0e-4;
/// Star tracker loses lock above this body rate [rad/s] (2 deg/s).
pub const ST_MAX_RATE: f32 = 0.035;
/// "Sun" direction in the rotation plane [rad] and exclusion half-angle [rad].
pub const ST_SUN_ANGLE: f32 = 2.094; // 120 deg
pub const ST_SUN_HALF_WIDTH: f32 = 0.26; // 15 deg

/// Disturbance torque per g of STM32 board tilt (accel X) [N m / g].
pub const DIST_PER_G: f32 = 2.0e-4;
/// Ignore tilt below this [g] so a board on the bench injects nothing.
pub const DIST_DEADBAND_G: f32 = 0.05;

/// Emulator / plant step [s] (100 Hz). Gyro and control run at 50 Hz,
/// the star tracker at 5 Hz, wheel and truth telemetry at 10 Hz.
pub const PLANT_DT: f32 = 0.01;
pub const GYRO_DIV: u32 = 2;
pub const ST_DIV: u32 = 20;
pub const TLM_DIV: u32 = 10;

/// Wheel goes to zero torque if no command arrives for this long [us].
pub const CMD_TIMEOUT_US: u32 = 250_000;

/// Controller: natural frequency [rad/s] and damping ratio.
pub const CTRL_WN: f32 = 0.3;
pub const CTRL_ZETA: f32 = 0.8;
/// Slew rate limit [rad/s]. Kept under ST_MAX_RATE so the tracker stays in lock.
pub const CTRL_MAX_RATE: f32 = 0.03;
