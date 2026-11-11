use robot_application::ports::outbound::DeviceHealthReader;
use robot_domain::{Capability, Health};
use std::{collections::HashMap, sync::Arc};

pub trait Device: Send + Sync {
    fn id(&self) -> &str;
    fn capabilities(&self) -> &'static [Capability];
    fn health(&self) -> Health;
}

#[derive(Default)]
pub struct DeviceManager {
    devices: HashMap<String, Arc<dyn Device>>,
}
impl DeviceManager {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn register(&mut self, device: Arc<dyn Device>) -> Result<(), String> {
        let id = device.id().to_owned();
        if self.devices.contains_key(&id) {
            return Err(format!("duplicate device ID: {id}"));
        }
        self.devices.insert(id, device);
        Ok(())
    }
    pub fn get(&self, id: &str) -> Option<Arc<dyn Device>> {
        self.devices.get(id).cloned()
    }
    pub fn statuses(&self) -> Vec<(String, Health)> {
        self.devices
            .iter()
            .map(|(id, dev)| (id.clone(), dev.health()))
            .collect()
    }
}

impl DeviceHealthReader for DeviceManager {
    fn health(&self, device_id: &str) -> Option<Health> {
        self.devices.get(device_id).map(|device| device.health())
    }

    fn statuses(&self) -> Vec<(String, Health)> {
        DeviceManager::statuses(self)
    }
}
