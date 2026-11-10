# ROS 2 bridge

This is the integration point for future ROS 2 nodes. Keep ROS message
conversion and callbacks here; call application use cases instead of invoking
hardware drivers directly. `DriveControl` operations are async. A ROS 2
callback should translate and enqueue a request to the async runtime, then
return; it should not synchronously wait for a hardware acknowledgement.
Coalesce periodic control inputs to the latest valid command instead of
creating a task for every topic sample. Publish device acknowledgements and
status through the ROS 2 feedback topics.

```text
ROS 2 callback -> async request -> DriveUseCase -> CommandGateway queue
                                      -> MotorPort -> Pico USB driver
ROS 2 status topics <- device feedback <- async completion
```

No ROS 2 dependency is required for the current USB controller. A bridge using
`rclrs` must match the installed ROS 2 distribution and should remain outside
the default Rust workspace until those dependencies are available.

For large image and point-cloud data, use ROS 2 topics and appropriate QoS.
Keep those data paths out of `robot-device-manager`.
