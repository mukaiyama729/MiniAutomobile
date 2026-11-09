# ROS 2 bridge

This is the integration point for future ROS 2 nodes. Keep ROS message
conversion and callbacks here; call application use cases instead of invoking
hardware drivers directly.

```text
ROS 2 callback -> DriveUseCase -> MotorPort -> Pico USB driver
```

No ROS 2 dependency is required for the current USB controller. A bridge using
`rclrs` must match the installed ROS 2 distribution and should remain outside
the default Rust workspace until those dependencies are available.

For large image and point-cloud data, use ROS 2 topics and appropriate QoS.
Keep those data paths out of `robot-device-manager`.
