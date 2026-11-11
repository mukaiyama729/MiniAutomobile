use crate::ports::{inbound::DeviceHealthQuery, outbound::DeviceHealthReader};
use std::sync::Arc;

/// Reads device health for inbound adapters without exposing the device manager.
pub struct DeviceHealthUseCase {
    reader: Arc<dyn DeviceHealthReader>,
}

impl DeviceHealthUseCase {
    pub fn new(reader: Arc<dyn DeviceHealthReader>) -> Self {
        Self { reader }
    }
}

impl DeviceHealthQuery for DeviceHealthUseCase {
    fn statuses(&self) -> Vec<(String, robot_domain::Health)> {
        self.reader.statuses()
    }
}
