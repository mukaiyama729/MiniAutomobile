//! Hardware-agnostic domain types; neither USB nor ROS 2 appears here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction { Forward, Backward }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionState { Stopped, Forward, Backward }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Capability { Motor, Encoder, Camera, Lidar, Imu, Gnss }

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Health { Ready, Degraded(String), Disconnected }
