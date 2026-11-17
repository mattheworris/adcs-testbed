//! adcs_emu: the STM32F407 Discovery pretends to be a star tracker, a gyro
//! and a reaction wheel, and talks to the BeagleBone Black over USART2.
//!
//! Drop this file into your stm32_playground repo as src/bin/adcs_emu.rs.
//! It reuses your Accelerometer (LIS3DSH) and LedController modules.
//!
//! Wiring (both sides 3.3 V logic):
//!   PA2 (USART2 TX) -> BBB P9.11 (UART4 RX)
//!   PA3 (USART2 RX) <- BBB P9.13 (UART4 TX)
//!   GND             -- BBB P9.1
//!
//! LEDs: green  = star tracker has a valid solution
//!       red    = star tracker blind (sun in view, or rate too high)
//!       orange = wheel speed (brightness = fraction of max speed)
//!       blue   = no wheel commands from the BBB (wheel forced to zero torque)
//!
//! Design: a bare-metal superloop with NO blocking I/O. The UART has a
//! one-byte receive register, so if we ever block (e.g. on a long transmit)
//! incoming command bytes overrun. Instead we poll RX, drain a TX queue one
//! byte at a time, and step the emulator when the 10 ms timer expires.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_probe as _;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    serial::{config::Config as SerialConfig, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};

use adcs_core::emulator::{Emulator, TICK_OUT_MAX};
use adcs_core::params::WHEEL_MAX_SPEED;
use adcs_core::proto::{st_status, Parser};
use aero_app::{accelerometer::Accelerometer, led_controller::LedController};

/// Outgoing byte queue: frames from one tick, drained as the UART is ready.
struct TxQueue {
    buf: [u8; TICK_OUT_MAX],
    len: usize,
    pos: usize,
    dropped: u32,
}

impl TxQueue {
    fn load(&mut self, src: &[u8]) {
        if self.pos < self.len {
            // Previous tick's frames still not sent: the link is too slow.
            // Count it; flight software will see the gap as missing packets.
            self.dropped += 1;
        }
        self.buf[..src.len()].copy_from_slice(src);
        self.len = src.len();
        self.pos = 0;
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(RccConfig::hse(8.MHz()).sysclk(84.MHz()).pclk1(42.MHz()));

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);
    let gpioe = dp.GPIOE.split(&mut rcc);

    // LEDs on TIM4 PWM, exactly as in level_indicator.rs.
    let (_pwm, (ch1, ch2, ch3, ch4)) = dp.TIM4.pwm_hz(1.kHz(), &mut rcc);
    let mut leds = LedController::new(
        ch1.with(gpiod.pd12),
        ch2.with(gpiod.pd13),
        ch3.with(gpiod.pd14),
        ch4.with(gpiod.pd15),
    );

    // LIS3DSH on SPI1 (mode 3), CS on PE3.
    let spi = Spi::new(
        dp.SPI1,
        (
            Some(gpioa.pa5.into_alternate()),
            Some(gpioa.pa6.into_alternate()),
            Some(gpioa.pa7.into_alternate()),
        ),
        Mode { polarity: Polarity::IdleHigh, phase: Phase::CaptureOnSecondTransition },
        10.MHz(),
        &mut rcc,
    );
    let cs = gpioe.pe3.into_push_pull_output();
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut accel = Accelerometer::new(spi, cs, &mut delay).unwrap();

    // USART2: PA2 = TX, PA3 = RX, 115200 8N1.
    let serial: Serial<pac::USART2> = Serial::new(
        dp.USART2,
        (gpioa.pa2, gpioa.pa3),
        SerialConfig::default().baudrate(115_200.bps()),
        &mut rcc,
    )
    .unwrap();
    let (mut tx, mut rx) = serial.split();

    // 10 ms periodic tick from TIM2 (independent of how long the work takes).
    let mut timer = dp.TIM2.counter_us(&mut rcc);
    timer.start(10.millis()).unwrap();

    let mut emu = Emulator::new(0xC0FFEE, 0.0);
    let mut parser = Parser::new();
    let mut txq = TxQueue { buf: [0; TICK_OUT_MAX], len: 0, pos: 0, dropped: 0 };
    let mut frame = [0u8; TICK_OUT_MAX];
    let mut rx_errors: u32 = 0;
    let mut g = [0.0f32; 3];

    defmt::println!("adcs_emu: running, 100 Hz plant, USART2 @ 115200");

    loop {
        // 1. Receive: drain every byte the UART has.
        loop {
            match embedded_hal_nb::serial::Read::read(&mut rx) {
                Ok(b) => {
                    if let Some(m) = parser.push(b) {
                        emu.on_msg(&m);
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {
                    // Overrun, framing or noise. The HAL clears the flag; break
                    // anyway so a stuck error can never hang the superloop.
                    rx_errors += 1;
                    break;
                }
            }
        }

        // 2. Transmit: one byte if the data register is free.
        if txq.pos < txq.len
            && embedded_hal_nb::serial::Write::write(&mut tx, txq.buf[txq.pos]).is_ok()
        {
            txq.pos += 1;
        }

        // 3. Every 10 ms: read the real sensor, step the simulated spacecraft.
        if timer.wait().is_ok() {
            if let Ok([x, y, z]) = accel.read_raw() {
                const LSB_PER_G: f32 = 16_384.0;
                g = [x as f32 / LSB_PER_G, y as f32 / LSB_PER_G, z as f32 / LSB_PER_G];
            }
            let n = emu.tick(g, &mut frame);
            txq.load(&frame[..n]);

            let st_ok = emu.last_st_status == st_status::VALID;
            let wheel = (libm::fabsf(emu.plant.wheel_speed) / WHEEL_MAX_SPEED * 255.0) as u8;
            leds.set_brightness(
                if st_ok { 255 } else { 0 },             // west  = green
                wheel,                                   // north = orange
                if st_ok { 0 } else { 255 },             // east  = red
                if emu.cmd_timed_out() { 255 } else { 0 }, // south = blue
            );

            if emu.time_us() % 5_000_000 == 0 {
                defmt::println!(
                    "t={}s theta={} wheel_rpm={} cmds={} ok={} crc_err={} rx_err={} tx_drop={}",
                    emu.time_us() / 1_000_000,
                    emu.plant.theta,
                    emu.plant.wheel_speed * 9.549_297,
                    emu.cmds_rx,
                    parser.frames_ok,
                    parser.crc_errors,
                    rx_errors,
                    txq.dropped
                );
            }
        }
    }
}
