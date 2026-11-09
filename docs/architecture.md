# Architecture decisions

## Current data/control flow

`robot-cli` -> `DriveUseCase` -> `MotorPort` -> `PicoUsbMotor` -> USB CDC -> Pico firmware -> GP2..GP5 -> ULN2003.

Pico firmware -> reply line -> `PicoUsbMotor` -> `DriveUseCase` -> `robot-cli`.

## Dependencies

- `robot-domain` knows no hardware or middleware.
- `robot-application` defines input use cases and output ports.
- `robot-device-manager` registers and monitors heterogeneous devices, and does **not** proxy frames or pointclouds.
- `adapter-pico-usb` implements `MotorPort` AND `Device`.
- `robot-protocol` is `no_std`, used by both Linux and Pico.
- `robot-cli` is an inbound adapter. Future ROS 2 callbacks are a second inbound adapter.

## Scalability

- Multiple Pico devices: create several `PicoUsbMotor::connect(id, path)` instances; register each with `DeviceManager`.
- To add cameras/LiDAR: create independent driver crates or supervise existing ROS nodes.
- For large image and point-cloud data, use ROS 2 topics and appropriate QoS rather than passing through DeviceManager.
- `DeviceManager` currently registers/inspects devices. Reconnection, discovery, factory wiring, persistent configuration, and lifecycle management remain future work.

## Protocol v0 (USB CDC)

- ASCII line framing, maximum 64 bytes per line, newline-terminated.
- Host request: `<seq> <FWD|BACK|STOP|STATUS|PING>\n`.
- Pico reply: `<seq> <OK|ERR> <STOPPED|FORWARD|BACKWARD>\n`.
- Only one outstanding command per device. Responses match sequence (u16 wrapping).
- Linux has a single serial worker and sends `PING` every 300ms while idle.
- Pico watchdog stops the motor 1.5 seconds after the last valid command.
- USB errors are reported in device health; automatic reconnection is not yet implemented.
- Development-only example VID/PID are used, not a production allocation.
- STOP disables motor coils and releases holding torque.
- This is an instructional prototype, not a certified emergency-stop system.

## Next steps

1. Add device config validation + factory, stable symlinks from udev.
2. Replace the one-motor demo with a differential-drive or Ackermann actuator port and wheel feedback.
3. Implement host reconnect / control leases / request ordering / CRC for CAN-UART transports.
4. Add ROS 2 bridge and tests for QoS/ROS message conversion.
5. Add telemetry, monotonic timestamps, rosbag2 recording, simulated drivers, and integration tests.
