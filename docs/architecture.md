# Architecture decisions

## Current data and control flow

```text
robot-cli or future ROS 2 bridge
  -> DriveUseCase
  -> MotorPort
  -> crates/robot-adapter/drivers/pico-usb
  -> crates/robot-adapter/protocol
  -> USB CDC
  -> crates/robot-adapter/firmware/pico-motor
  -> GP2..GP5 -> ULN2003
```

Replies travel back through the same layers from firmware to the CLI or bridge.

## Boundaries

- `robot-domain` owns hardware-independent types.
- `robot-application` defines use cases and output ports.
- `robot-device-manager` registers and monitors devices. It does not proxy image frames or point clouds.
- `crates/robot-adapter/drivers` contains concrete hardware drivers such as Pico USB, V4L2 cameras, and USB LiDARs.
- `crates/robot-adapter/transports` contains reusable USB, CAN, and UDP communication implementations.
- `crates/robot-adapter/bridges` connects external software stacks such as ROS 2 and Autoware to application use cases.
- `crates/robot-adapter/protocol` is the `no_std` `robot-protocol` Git submodule shared by Linux and Pico.
- `crates/robot-adapter/firmware` is the separate `robot-firmware` Git submodule. Its embedded workspace is excluded from the Linux workspace.
- `robot-cli` is an inbound adapter. ROS 2 callbacks will be another inbound adapter.

The USB driver currently owns its serial worker directly. A shared transport
crate can be extracted when another driver needs the same transport behavior.

## Scaling

- For multiple Pico devices, create several `PicoUsbMotor::connect(id, path)` instances and register each with `DeviceManager`.
- Add cameras and LiDARs as independent drivers or supervise their vendor ROS nodes.
- Send large images and point clouds through ROS 2 topics with suitable QoS rather than through `DeviceManager`.
- Device discovery, reconnection, factory wiring, persistent configuration, and lifecycle management remain future work.

## Protocol v0 (USB CDC)

- ASCII line framing, maximum 64 bytes per line, newline-terminated.
- Host request: `<seq> <FWD|BACK|STOP|STATUS|PING>\n`.
- Pico reply: `<seq> <OK|ERR> <STOPPED|FORWARD|BACKWARD>\n`.
- One outstanding command per device; responses match sequence numbers with `u16` wrapping.
- Linux uses one serial worker and sends `PING` every 300 ms while idle.
- The Pico watchdog stops the motor 1.5 seconds after the last valid command.
- USB errors are reported in device health; automatic reconnection is not implemented.
- Development-only example VID/PID values are used, not a production allocation.
- STOP disables the motor coils and releases holding torque.
- This is an instructional prototype, not a certified emergency-stop system.

## Submodule update policy

Both child repositories are pinned to commits in the main repository. A push to
their `main` branches can send a `repository_dispatch`; a scheduled run checks
for missed notifications every two hours. The main workflow updates the local
submodule refs, runs the Linux workspace tests and Pico firmware cross-build,
and opens a pull request. Only merging that PR changes the versions used by the
main repository.

Configure `ROBOT_SYSTEM_REPO` and `ROBOT_SYSTEM_DISPATCH_TOKEN` in each child
repository to enable push notifications. The token needs `Contents: write` on
the main repository. The main repository's Actions settings must allow workflows
to create pull requests.

## Next steps

1. Add device configuration validation, a factory, and stable udev symlinks.
2. Replace the one-motor demo with a differential-drive or Ackermann actuator port and wheel feedback.
3. Add host reconnect, control leases, request ordering, and CRC for CAN/UART transports.
4. Implement ROS 2 message conversion and appropriate QoS.
5. Add telemetry, monotonic timestamps, rosbag2 recording, simulated drivers, and hardware integration testing.
