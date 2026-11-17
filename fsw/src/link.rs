//! The link to the spacecraft components. Two implementations with the same
//! interface:
//! - `SerialLink`: a real UART (/dev/ttyS4 on the BBB) to the STM32 (HWIL)
//! - `SimLink`: the same `Emulator` the STM32 runs, in-process (SIL)
//!
//! Both move real protocol bytes through a `Parser`, so the flight software
//! cannot tell them apart except by timing.
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::time::{Duration, Instant};

use adcs_core::emulator::{Emulator, TICK_OUT_MAX};
use adcs_core::params::PLANT_DT;
use adcs_core::proto::{Msg, Parser, MAX_FRAME};

pub trait Link {
    /// Wait up to `timeout` for the next decoded message.
    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Msg>>;
    fn send(&mut self, m: &Msg) -> io::Result<()>;
    /// (frames_ok, crc_errors, bad_frames)
    fn stats(&self) -> (u32, u32, u32);
}

// ---------------------------------------------------------------- serial

pub struct SerialLink {
    file: File,
    parser: Parser,
    rx: [u8; 256],
    rx_len: usize,
    rx_pos: usize,
}

impl SerialLink {
    /// Open a tty in raw mode, 8N1, at `baud`. Reads block for at most 0.1 s.
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        let fd = file.as_raw_fd();
        let speed = match baud {
            115_200 => libc::B115200,
            230_400 => libc::B230400,

            // #[cfg(target_os = "linux)]
            // 460_800 => libc::B460800,
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "unsupported baud",
                ))
            }
        };
        // SAFETY: fd is a valid open tty for the life of `file`; termios is
        // plain old data that tcgetattr fully initializes.
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut t) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::cfmakeraw(&mut t); // no echo, no line editing, 8 bits
            t.c_cflag |= libc::CLOCAL | libc::CREAD;
            t.c_cflag &= !(libc::CSTOPB | libc::PARENB | libc::CRTSCTS);
            t.c_cc[libc::VMIN] = 0; // return with whatever has arrived...
            t.c_cc[libc::VTIME] = 1; // ...after at most 100 ms
            libc::cfsetispeed(&mut t, speed);
            libc::cfsetospeed(&mut t, speed);
            if libc::tcsetattr(fd, libc::TCSANOW, &t) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::tcflush(fd, libc::TCIOFLUSH);
        }
        Ok(SerialLink {
            file,
            parser: Parser::new(),
            rx: [0; 256],
            rx_len: 0,
            rx_pos: 0,
        })
    }
}

impl Link for SerialLink {
    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Msg>> {
        let deadline = Instant::now() + timeout;
        loop {
            while self.rx_pos < self.rx_len {
                let b = self.rx[self.rx_pos];
                self.rx_pos += 1;
                if let Some(m) = self.parser.push(b) {
                    return Ok(Some(m));
                }
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            self.rx_len = self.file.read(&mut self.rx)?; // VTIME bounds this
            self.rx_pos = 0;
        }
    }

    fn send(&mut self, m: &Msg) -> io::Result<()> {
        let mut f = [0u8; MAX_FRAME];
        let n = m.encode(&mut f);
        self.file.write_all(&f[..n])
    }

    fn stats(&self) -> (u32, u32, u32) {
        (
            self.parser.frames_ok,
            self.parser.crc_errors,
            self.parser.bad_frames,
        )
    }
}

// ---------------------------------------------------------------- sim

pub struct SimLink {
    emu: Emulator,
    parser: Parser,
    cmd_parser: Parser,
    out: [u8; TICK_OUT_MAX],
    out_len: usize,
    out_pos: usize,
    next_tick: Instant,
    realtime: bool,
}

impl SimLink {
    pub fn new(theta0: f32, realtime: bool) -> Self {
        SimLink {
            emu: Emulator::new(0xADC5, theta0),
            parser: Parser::new(),
            cmd_parser: Parser::new(),
            out: [0; TICK_OUT_MAX],
            out_len: 0,
            out_pos: 0,
            next_tick: Instant::now(),
            realtime,
        }
    }
}

impl Link for SimLink {
    fn recv(&mut self, _timeout: Duration) -> io::Result<Option<Msg>> {
        loop {
            while self.out_pos < self.out_len {
                let b = self.out[self.out_pos];
                self.out_pos += 1;
                if let Some(m) = self.parser.push(b) {
                    return Ok(Some(m));
                }
            }
            if self.realtime {
                let now = Instant::now();
                if self.next_tick > now {
                    std::thread::sleep(self.next_tick - now);
                }
                self.next_tick += Duration::from_secs_f32(PLANT_DT);
            }
            self.out_len = self.emu.tick([0.0, 0.0, 1.0], &mut self.out);
            self.out_pos = 0;
        }
    }

    fn send(&mut self, m: &Msg) -> io::Result<()> {
        let mut f = [0u8; MAX_FRAME];
        let n = m.encode(&mut f);
        for &b in &f[..n] {
            if let Some(cmd) = self.cmd_parser.push(b) {
                self.emu.on_msg(&cmd);
            }
        }
        Ok(())
    }

    fn stats(&self) -> (u32, u32, u32) {
        (
            self.parser.frames_ok,
            self.parser.crc_errors,
            self.parser.bad_frames,
        )
    }
}
