use adapter_cli::CliController;
use adapter_pico_usb::PicoUsbMotor;
use robot_application::{
    coordination::{CommandGateway, SafetySupervisor},
    ports::{
        inbound::{DeviceHealthQuery, DriveControl},
        outbound::{
            ControlStateWriter, DeviceHealthReader, MotorPort, RobotMotionStateWriter,
            RobotStateReader, SafetyStateWriter,
        },
    },
    usecases::{DeviceHealthUseCase, DriveUseCase},
};
use robot_device_manager::{Device, DeviceManager};
use robot_domain::{CommandSource, ControlMode};
use robot_state::RobotStateStore;
use std::{error::Error, sync::Arc};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyACM0".to_owned());

    // Runtime composition root: wire concrete adapters to application ports.
    let motor = Arc::new(PicoUsbMotor::connect("motor-left", &port).await?);
    let mut devices = DeviceManager::new();
    let managed_device: Arc<dyn Device> = motor.clone();
    devices.register(managed_device)?;
    let devices = Arc::new(devices);

    let state_store = RobotStateStore::new();
    let state_reader: Arc<dyn RobotStateReader> = Arc::new(state_store.reader());
    let motion_writer: Arc<dyn RobotMotionStateWriter> = Arc::new(state_store.motion_writer());
    let control_writer: Arc<dyn ControlStateWriter> = Arc::new(state_store.control_writer());
    let safety_writer: Arc<dyn SafetyStateWriter> = Arc::new(state_store.safety_writer());
    let motor_port: Arc<dyn MotorPort> = motor;
    let health_reader: Arc<dyn DeviceHealthReader> = devices.clone();
    let device_health: Arc<dyn DeviceHealthQuery> =
        Arc::new(DeviceHealthUseCase::new(Arc::clone(&health_reader)));

    let safety = Arc::new(SafetySupervisor::new(
        Arc::clone(&state_reader),
        Arc::clone(&control_writer),
        safety_writer,
        Arc::clone(&motion_writer),
        Arc::clone(&motor_port),
        Arc::clone(&health_reader),
        "motor-left",
    ));
    let gateway = Arc::new(CommandGateway::new(
        state_reader,
        control_writer,
        motion_writer,
        motor_port,
        health_reader,
        "motor-left",
        safety,
    ));
    let drive: Arc<dyn DriveControl> = Arc::new(DriveUseCase::new(
        gateway,
        CommandSource::Cli,
        ControlMode::Manual,
    ));

    CliController::new(drive, device_health).run(&port).await?;
    Ok(())
}
