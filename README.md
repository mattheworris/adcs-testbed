# ADCS testbed

A single-axis spacecraft attitude determination and control system (ADCS)
for learning flight software:

- `core/`: `no_std` Rust shared by every target: wire protocol (CRC-16
  framed), body + reaction-wheel dynamics, gyro / star tracker / wheel
  emulator, Kalman filter (angle + gyro bias), rate-limited PID controller.
- `fsw/`: Linux flight software for the BeagleBone Black. `--sim` runs the
  emulator in-process (SIL); `--port /dev/ttyS4` talks to the STM32 (HWIL).
- `emu/adcs_emu.rs`: STM32F407 Discovery firmware (add to `stm32_playground`).
- `meta-adcs/`: Yocto Whinlatter application layer (flight software recipe,
  systemd/SysV service, image, UART4 device-tree snippet). Builds on the
  board support in github.com/mattheworris/beagleBone-yocto
  (`meta-beaglebone-bsp`, MACHINE = "beaglebone-black").
- `ground/ground.py`: live telemetry plots.

Quick start (laptop):

```bash
cargo test
python3 ground/ground.py &
cargo run --release -p adcs-fsw -- --sim --ground 127.0.0.1:5005
echo "point-deg 160" | nc -u -w0 127.0.0.1 5006
```

Ground commands: `idle`, `damp`, `point <rad>`, `point-deg <deg>`.

Telemetry CSV fields (10 Hz UDP): t, theta, rate, bias, sigma, target,
wheel_rpm, torque, st_status, wheel_status, truth_theta, truth_omega,
accel_x, frames_ok, crc_errors, bad_frames, mode.

The workspace also builds on Rust 1.75 (CI checks 1.75 and stable), so the
layer can move to the Scarthgap LTS release without code changes.
