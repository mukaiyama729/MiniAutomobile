# Robot Rust: Pico 2 + Ubuntu, Clean Architecture starter

A deliberately small, extensible example for a Raspberry Pi Pico 2 (RP2350)
controlling one **28BYJ-48 through ULN2003** and an Ubuntu controller written in Rust.

## Structure

- `crates/robot-domain`: hardware-independent types
- `crates/robot-application`: `MotorPort`, `DriveUseCase`
- `crates/robot-device-manager`: heterogeneous device registry + health
- `crates/robot-protocol`: shared `#![no_std]` newline protocol
- `crates/adapter-pico-usb`: USB serial worker + periodic heartbeat
- `apps/robot-cli`: current inbound command adapter
- `firmwares/pico-motor`: RP2350 firmware (separate embedded Cargo workspace)
- `config/devices.example.yaml`: future configuration example (not loaded yet)
- `ros2_ws/`: future ROS 2/Autoware integration

See `docs/architecture.md` for constraints and planned expansion.

## Initialize submodules

After cloning this repository, initialize its nested firmware and protocol
repositories:

```sh
git submodule update --init --recursive
```

## Hardware and wiring

- Pico 2 GP2, GP3, GP4, GP5 -> ULN2003 IN1, IN2, IN3, IN4 respectively.
- Power 28BYJ-48 via ULN2003 with suitable **external regulated 5V** supply.
- Connect Pico GND and ULN2003 supply GND together.
- NEVER power the motor from a Pico GPIO pin.
- Pico 2 Micro-USB data port -> Ubuntu USB for CDC ACM traffic.
- Debug Probe -> Pico DEBUG connector for flashing/debugging (optional if using picotool).

**Important:** the Pico 2 must be connected to the PC by the Micro-USB data cable
for USB CDC, even if you flash over a separate Debug Probe.

## Host tests (no Pico required)

```sh
cargo test --workspace
```

## Flash firmware

Install current Rust and Pico 2 target:

```sh
rustup target add thumbv8m.main-none-eabihf
cargo install --locked probe-rs-tools
cd firmwares/pico-motor
cargo run --release
```

If your existing Pico 2 project already has a working `probe-rs`/`picotool`
runner, use that instead of the sample `.cargo/config.toml`. If using a Debug
Probe, check `probe-rs list` and the `--chip` label for your installed version.
Alternatively use `picotool` with BOOTSEL according to your existing setup.

## Run Ubuntu CLI

```sh
ls -l /dev/serial/by-id/
cargo run -p robot-cli -- /dev/ttyACM0
```

Or use the actual `/dev/serial/by-id/...` path as the argument.

Commands: `f` forward, `b` reverse, `s` stop, `?` state, `h` device health, `q` stop and quit.
The I/O worker sends heartbeat packets while waiting for keyboard input, so
rotation continues while the USB link is alive. On link loss the Pico stops
in about 1.5 seconds; it does not automatically resume when reconnected.

For `Permission denied` use `sudo usermod -aG dialout "$USER"` and re-login.

## Scope and verification

This is **source code prepared for local build**. It has not been built on an
RP2350 or checked against the attached hardware here. Reconcile GPIO mapping,
HAL/runner versions and power connections with your already-working project.

Don't use for an unattended moving vehicle: add a hardware emergency stop,
validated failsafe logic, torque/velocity limits and integration tests first.
