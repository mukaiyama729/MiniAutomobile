# Robot System: Pico 2 + Ubuntu

A Rust robot-control foundation for Ubuntu/Jetson and a Raspberry Pi Pico 2
(RP2350). The current hardware path drives a 28BYJ-48 stepper through USB CDC;
the application boundary is structured for future devices and middleware.

## Repository layout

```text
crates/
  robot-domain/                  Hardware-independent types and rules
  robot-application/              Inbound/outbound ports, use case, coordination
  robot-state/                    Versioned shared snapshots and narrow writers
  robot-device-manager/           Device registration and health
  robot-adapter/
    cli/                          Terminal input adapter
    drivers/
      pico-usb/                   Linux USB CDC driver
    transports/                   Future shared USB, CAN, UDP code
    bridges/
      ros2/                       Future ROS 2 integration
      autoware/                   Future Autoware integration
    protocol/                     robot-protocol Git submodule
    firmware/                     robot-firmware Git submodule
apps/robot-cli/                    Composition root for the CLI application
config/                            Example device configuration
docs/                              Architecture notes
.github/workflows/                 CI and submodule update PRs
```

The CLI path now goes through the application command gateway, resource
arbiter, safety state, and motor port. `robot-state` provides a consistent
snapshot of motion, control ownership, and the safety latch. ROS 2, Autoware,
camera, LiDAR, mission workflow, perception, and localization adapters remain
future work; their directories are not represented as implemented crates.
`robot-protocol` is shared by Linux and Pico firmware, while `robot-firmware`
owns the embedded Cargo workspace. Both are pinned by Git submodules. See
[docs/architecture.md](docs/architecture.md) for current boundaries and safety
limits.

Tokioのタスク、チャネル、タイムアウト、キャンセルの使い方は
[docs/communication-state-tokio.md](docs/communication-state-tokio.md) を参照してください。

## Clone and update

The current GitHub remote for the main repository is `mukaiyama729/MiniAutomobile`:

```sh
git clone --recurse-submodules https://github.com/mukaiyama729/MiniAutomobile.git
cd MiniAutomobile
```

For an existing checkout, initialize the pinned child repository commits with:

```sh
git submodule update --init --recursive
```

To develop the two child repositories independently, place their checkouts side
by side so the firmware's relative protocol dependency resolves:

```text
dev/
  firmware/   # robot-firmware checkout
  protocol/   # robot-protocol checkout
```

## Update flow

Pushes to `main` in either child repository can notify the main repository.
The main workflow also checks for updates every two hours and supports manual
dispatch. It advances both submodules, runs the host workspace tests and the
Pico firmware cross-build, then opens a pull request with the new pinned commits.
Submodule updates are reviewed and merged through that pull request.

Configure each child repository with:

| Setting | Value |
| --- | --- |
| Variable `ROBOT_SYSTEM_REPO` | `<owner>/<main-repository>` (currently `mukaiyama729/MiniAutomobile`) |
| Secret `ROBOT_SYSTEM_DISPATCH_TOKEN` | Fine-grained token with `Contents: write` on the main repository |

In the main repository's Actions settings, allow GitHub Actions to create pull
requests. If the child repositories are private, also provide a read token for
the submodules as the `ROBOT_SUBMODULES_TOKEN` secret.

The notification workflows and firmware path change live inside the two child
repositories. Commit and push those child repository changes first; the main
repository's pinned commits change only after the update workflow opens a PR and
that PR is merged.

The manual equivalent is:

```sh
git submodule update --remote --checkout -- \
  crates/robot-adapter/protocol crates/robot-adapter/firmware
cargo test --workspace
cargo build --release \
  --manifest-path crates/robot-adapter/firmware/pico-motor/Cargo.toml \
  --target thumbv8m.main-none-eabihf
git diff --submodule=log
```

## Hardware and wiring

- Pico 2 GP2, GP3, GP4, GP5 -> ULN2003 IN1, IN2, IN3, IN4 respectively.
- Power 28BYJ-48 via ULN2003 with a suitable **external regulated 5V** supply.
- Connect Pico GND and ULN2003 supply GND together.
- Never power the motor from a Pico GPIO pin.
- Pico 2 Micro-USB data port -> Ubuntu USB for CDC ACM traffic.
- Debug Probe -> Pico DEBUG connector for flashing/debugging (optional with picotool).

**Important:** the Pico 2 needs the Micro-USB data cable for USB CDC, even when
you flash through a separate Debug Probe.

## Build and run

Install the Pico 2 target and `probe-rs` tools:

```sh
rustup target add thumbv8m.main-none-eabihf
cargo install --locked probe-rs-tools
cd crates/robot-adapter/firmware/pico-motor
cargo run --release
```

If your Pico project already has a working `probe-rs` or `picotool` runner, use
that instead of the sample `.cargo/config.toml`. For a Debug Probe, check
`probe-rs list` and the `--chip` label for your installed version.

Run the Ubuntu CLI with:

```sh
ls -l /dev/serial/by-id/
cargo run -p robot-cli -- /dev/ttyACM0
```

Use the actual `/dev/serial/by-id/...` path when available. Commands are `f`
forward, `b` reverse, `s` stop, `?` state, `h` device health, `e` latched
software emergency stop, `r` checked reset, and `q` release control and quit.
The CLI renews a short control lease while it is running. The USB worker repeats
the active direction while that lease is valid, sends STOP at expiry, and stops
background watchdog traffic after control is lost. Firmware also stops motion
after 1.5 seconds without a valid request. A separate RP2350 hardware watchdog
reboots the MCU if its main loop stalls; it does not replace that command-timeout
stop path. These are best-effort software safeguards, not a certified physical
emergency stop.
The firmware command timeout is refreshed by every valid protocol request,
including STATUS and PING, so additional drivers must not poll after losing
their control lease. The hardware watchdog is fed by completed main-loop
iterations, independently of protocol traffic. `?` performs a fresh query only
while a lease is active; without one, the driver returns a cached stopped state
or rejects a moving or unknown state. A latched stop blocks `?`. Reset sends a
fresh STOP and requires a matching `STOPPED` acknowledgement before clearing
the latch.

For `Permission denied`, add your user to the `dialout` group and re-login.

This is an instructional prototype, not a certified emergency-stop system.
Before unattended operation, add a hardware emergency stop, validated failsafe
logic, torque and velocity limits, and hardware integration testing.
