# STM32 emulator firmware

`adcs_emu.rs` is a new binary for your existing `stm32_playground` repo.
It reuses `aero_app::accelerometer` and `aero_app::led_controller`.

1. Copy `adcs_emu.rs` to `stm32_playground/src/bin/adcs_emu.rs`.
2. In `stm32_playground/Cargo.toml` add:

```toml
[[bin]]
name = "adcs_emu"
path = "src/bin/adcs_emu.rs"
test = false

[dependencies]
adcs-core = { path = "../adcs-testbed/core" }   # or a git dependency
embedded-hal-nb = "1.0"
nb = "1.1"
```

3. `cargo rb adcs_emu` (your alias for `cargo run --bin`) flashes it with probe-rs.

This file type-checks against stm32f4xx-hal 0.23 (your current version).
It has not been run on hardware yet.
