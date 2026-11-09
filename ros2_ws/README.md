# ROS 2 bridge (future integration point)

No ROS 2 dependency is required for the working USB version.

Future `robot-ros2-bridge` should depend on `robot-application` and call exactly
these methods when ROS messages arrive:

```rust
let drive = DriveUseCase::new(motor_port);
// On a validated ROS 2 motion request:
drive.drive(Direction::Forward)?;
// On STOP, timeout, or an emergency condition:
drive.stop()?;
```

Conceptual topology:

- CSI/USB camera -> vendor ROS 2 driver -> `sensor_msgs/Image`.
- USB/Ethernet LiDAR -> vendor ROS 2 driver -> `sensor_msgs/PointCloud2`.
- Autoware -> compatible vehicle control interface -> `robot-ros2-bridge`.
- `robot-ros2-bridge` -> `DriveUseCase` -> `MotorPort` -> Pico USB driver.

For real vehicles use a speed/steering controller, appropriate message mapping,
feedback and a hardware emergency stop. This demo only controls one stepper motor.

A ROS 2 bridge built with `rclrs` will require a matching ROS 2 installation and
its message packages. Keep it outside default `cargo test --workspace` until installed.
