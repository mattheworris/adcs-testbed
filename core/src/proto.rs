//! Wire protocol between the "spacecraft components" (STM32 emulator) and
//! the flight computer (BeagleBone Black). Real star trackers and wheels
//! speak protocols just like this over RS-422/RS-485 or CAN.
//!
//! Frame:  0xEB 0x90 | id:u8 | len:u8 | payload[len] | crc16:u16 (big-endian)
//! The CRC covers id, len and payload. Multi-byte payload fields are
//! little-endian. Quaternions are scalar-first (w, x, y, z).
use crate::crc::crc16_ccitt;
use crate::quat::Quat;

pub const SYNC0: u8 = 0xEB;
pub const SYNC1: u8 = 0x90;
pub const MAX_PAYLOAD: usize = 32;
pub const MAX_FRAME: usize = 4 + MAX_PAYLOAD + 2;

pub mod id {
    pub const STAR_TRACKER: u8 = 0x10;
    pub const WHEEL_CMD: u8 = 0x20;
    pub const WHEEL_TLM: u8 = 0x21;
    pub const GYRO: u8 = 0x30;
    pub const ACCEL: u8 = 0x31;
    pub const TRUTH: u8 = 0x40;
}

/// Star tracker status codes.
pub mod st_status {
    pub const VALID: u8 = 0;
    /// Sun in the field of view: no solution.
    pub const SUN_BLIND: u8 = 1;
    /// Body rate too high to track stars (streaks).
    pub const RATE_TOO_HIGH: u8 = 2;
}

/// Wheel status bits.
pub mod wheel_status {
    pub const SATURATED: u8 = 1 << 0;
    pub const CMD_TIMEOUT: u8 = 1 << 1;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Msg {
    /// Attitude solution, sent at 5 Hz.
    StarTracker {
        t_us: u32,
        q: Quat,
        status: u8,
        n_stars: u8,
    },
    /// Wheel torque command from the flight computer [N m].
    WheelCmd { torque_nm: f32 },
    /// Wheel telemetry at 10 Hz.
    WheelTlm {
        t_us: u32,
        speed_rpm: f32,
        torque_nm: f32,
        status: u8,
    },
    /// Body rates [rad/s] at 50 Hz.
    Gyro { t_us: u32, rate: [f32; 3] },
    /// Real LIS3DSH reading from the STM32 board [g] at 10 Hz.
    Accel { t_us: u32, g: [f32; 3] },
    /// Simulator truth (test equipment only, never used by the estimator).
    Truth { t_us: u32, theta: f32, omega: f32 },
}

struct W<'a> {
    buf: &'a mut [u8],
    n: usize,
}

impl<'a> W<'a> {
    fn u8(&mut self, v: u8) {
        self.buf[self.n] = v;
        self.n += 1;
    }
    fn u32(&mut self, v: u32) {
        self.buf[self.n..self.n + 4].copy_from_slice(&v.to_le_bytes());
        self.n += 4;
    }
    fn f32(&mut self, v: f32) {
        self.u32(v.to_bits());
    }
}

struct R<'a> {
    buf: &'a [u8],
    n: usize,
}

impl<'a> R<'a> {
    fn u8(&mut self) -> u8 {
        let v = self.buf[self.n];
        self.n += 1;
        v
    }
    fn u32(&mut self) -> u32 {
        let mut b = [0u8; 4];
        b.copy_from_slice(&self.buf[self.n..self.n + 4]);
        self.n += 4;
        u32::from_le_bytes(b)
    }
    fn f32(&mut self) -> f32 {
        f32::from_bits(self.u32())
    }
}

impl Msg {
    pub fn id(&self) -> u8 {
        match self {
            Msg::StarTracker { .. } => id::STAR_TRACKER,
            Msg::WheelCmd { .. } => id::WHEEL_CMD,
            Msg::WheelTlm { .. } => id::WHEEL_TLM,
            Msg::Gyro { .. } => id::GYRO,
            Msg::Accel { .. } => id::ACCEL,
            Msg::Truth { .. } => id::TRUTH,
        }
    }

    fn payload_len(id: u8) -> Option<usize> {
        match id {
            id::STAR_TRACKER => Some(4 + 16 + 2),
            id::WHEEL_CMD => Some(4),
            id::WHEEL_TLM => Some(4 + 4 + 4 + 1),
            id::GYRO | id::ACCEL => Some(4 + 12),
            id::TRUTH => Some(4 + 4 + 4),
            _ => None,
        }
    }

    /// Encode a complete frame into `out`. Returns the frame length.
    /// Panics if `out` is shorter than MAX_FRAME.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let id = self.id();
        let len = Self::payload_len(id).unwrap();
        out[0] = SYNC0;
        out[1] = SYNC1;
        out[2] = id;
        out[3] = len as u8;
        let mut w = W {
            buf: &mut out[4..],
            n: 0,
        };
        match *self {
            Msg::StarTracker {
                t_us,
                q,
                status,
                n_stars,
            } => {
                w.u32(t_us);
                w.f32(q.w);
                w.f32(q.x);
                w.f32(q.y);
                w.f32(q.z);
                w.u8(status);
                w.u8(n_stars);
            }
            Msg::WheelCmd { torque_nm } => w.f32(torque_nm),
            Msg::WheelTlm {
                t_us,
                speed_rpm,
                torque_nm,
                status,
            } => {
                w.u32(t_us);
                w.f32(speed_rpm);
                w.f32(torque_nm);
                w.u8(status);
            }
            Msg::Gyro { t_us, rate: v } | Msg::Accel { t_us, g: v } => {
                w.u32(t_us);
                for x in v {
                    w.f32(x);
                }
            }
            Msg::Truth { t_us, theta, omega } => {
                w.u32(t_us);
                w.f32(theta);
                w.f32(omega);
            }
        }
        debug_assert_eq!(w.n, len);
        let crc = crc16_ccitt(&out[2..4 + len]);
        out[4 + len..6 + len].copy_from_slice(&crc.to_be_bytes());
        6 + len
    }

    fn decode(id: u8, p: &[u8]) -> Option<Msg> {
        if Self::payload_len(id)? != p.len() {
            return None;
        }
        let mut r = R { buf: p, n: 0 };
        Some(match id {
            id::STAR_TRACKER => {
                let t_us = r.u32();
                let q = Quat {
                    w: r.f32(),
                    x: r.f32(),
                    y: r.f32(),
                    z: r.f32(),
                };
                Msg::StarTracker {
                    t_us,
                    q,
                    status: r.u8(),
                    n_stars: r.u8(),
                }
            }
            id::WHEEL_CMD => Msg::WheelCmd { torque_nm: r.f32() },
            id::WHEEL_TLM => Msg::WheelTlm {
                t_us: r.u32(),
                speed_rpm: r.f32(),
                torque_nm: r.f32(),
                status: r.u8(),
            },
            id::GYRO => Msg::Gyro {
                t_us: r.u32(),
                rate: [r.f32(), r.f32(), r.f32()],
            },
            id::ACCEL => Msg::Accel {
                t_us: r.u32(),
                g: [r.f32(), r.f32(), r.f32()],
            },
            id::TRUTH => Msg::Truth {
                t_us: r.u32(),
                theta: r.f32(),
                omega: r.f32(),
            },
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Sync0,
    Sync1,
    Id,
    Len,
    Payload,
    Crc0,
    Crc1,
}

/// Byte-at-a-time frame parser. Feed it whatever the UART hands you; it
/// resynchronizes on the sync word after noise, dropped bytes or bad CRCs.
pub struct Parser {
    state: State,
    id: u8,
    len: usize,
    buf: [u8; MAX_PAYLOAD],
    n: usize,
    crc_hi: u8,
    pub frames_ok: u32,
    pub crc_errors: u32,
    pub bad_frames: u32,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Parser {
            state: State::Sync0,
            id: 0,
            len: 0,
            buf: [0; MAX_PAYLOAD],
            n: 0,
            crc_hi: 0,
            frames_ok: 0,
            crc_errors: 0,
            bad_frames: 0,
        }
    }

    pub fn push(&mut self, b: u8) -> Option<Msg> {
        match self.state {
            State::Sync0 => {
                if b == SYNC0 {
                    self.state = State::Sync1;
                }
            }
            State::Sync1 => {
                self.state = match b {
                    SYNC1 => State::Id,
                    SYNC0 => State::Sync1,
                    _ => State::Sync0,
                };
            }
            State::Id => {
                self.id = b;
                self.state = State::Len;
            }
            State::Len => {
                self.len = b as usize;
                self.n = 0;
                self.state = if self.len > MAX_PAYLOAD {
                    self.bad_frames += 1;
                    State::Sync0
                } else if self.len == 0 {
                    State::Crc0
                } else {
                    State::Payload
                };
            }
            State::Payload => {
                self.buf[self.n] = b;
                self.n += 1;
                if self.n == self.len {
                    self.state = State::Crc0;
                }
            }
            State::Crc0 => {
                self.crc_hi = b;
                self.state = State::Crc1;
            }
            State::Crc1 => {
                self.state = State::Sync0;
                let rx = u16::from_be_bytes([self.crc_hi, b]);
                let mut hdr = [0u8; 2 + MAX_PAYLOAD];
                hdr[0] = self.id;
                hdr[1] = self.len as u8;
                hdr[2..2 + self.len].copy_from_slice(&self.buf[..self.len]);
                if crc16_ccitt(&hdr[..2 + self.len]) != rx {
                    self.crc_errors += 1;
                    return None;
                }
                match Msg::decode(self.id, &self.buf[..self.len]) {
                    Some(m) => {
                        self.frames_ok += 1;
                        return Some(m);
                    }
                    None => self.bad_frames += 1,
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_msgs() -> [Msg; 6] {
        [
            Msg::StarTracker {
                t_us: 1,
                q: Quat::from_z_angle(0.5),
                status: 0,
                n_stars: 12,
            },
            Msg::WheelCmd { torque_nm: -0.0015 },
            Msg::WheelTlm {
                t_us: 2,
                speed_rpm: 1234.5,
                torque_nm: 0.001,
                status: 3,
            },
            Msg::Gyro {
                t_us: 3,
                rate: [0.1, -0.2, 0.3],
            },
            Msg::Accel {
                t_us: 4,
                g: [0.0, 0.0, 1.0],
            },
            Msg::Truth {
                t_us: 5,
                theta: 1.0,
                omega: -0.01,
            },
        ]
    }

    #[test]
    fn round_trip_every_message() {
        let mut p = Parser::new();
        for m in all_msgs() {
            let mut f = [0u8; MAX_FRAME];
            let n = m.encode(&mut f);
            let got: Vec<Msg> = f[..n].iter().filter_map(|&b| p.push(b)).collect();
            assert_eq!(got, vec![m]);
        }
        assert_eq!(p.frames_ok, 6);
    }

    #[test]
    fn resyncs_after_garbage_and_rejects_bad_crc() {
        let m = Msg::Gyro {
            t_us: 9,
            rate: [1.0, 2.0, 3.0],
        };
        let mut f = [0u8; MAX_FRAME];
        let n = m.encode(&mut f);
        let mut stream = vec![0x00, 0xEB, 0x13, 0xEB, 0xEB];
        let mut corrupt = f[..n].to_vec();
        corrupt[8] ^= 0x40; // flip one payload bit
        stream.extend_from_slice(&corrupt);
        stream.extend_from_slice(&f[..n]);
        let mut p = Parser::new();
        let got: Vec<Msg> = stream.iter().filter_map(|&b| p.push(b)).collect();
        assert_eq!(got, vec![m]);
        assert_eq!(p.crc_errors, 1);
    }
}
