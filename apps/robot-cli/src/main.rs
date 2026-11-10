use adapter_pico_usb::PicoUsbMotor;
use robot_application::{
    coordination::{CommandGateway, SafetySupervisor},
    ports::{
        inbound::DriveControl,
        outbound::{
            ControlError, ControlStateWriter, DeviceHealthReader, MotorPort,
            RobotMotionStateWriter, RobotStateReader, SafetyStateWriter,
        },
    },
    usecases::DriveUseCase,
};
use robot_device_manager::{Device, DeviceManager};
use robot_domain::{CommandSource, ControlLease, ControlMode, Direction, MotionState};
use robot_state::RobotStateStore;
use std::{
    error::Error,
    io,
    sync::{Arc, Mutex, mpsc},
    thread,
    time::{Duration, Instant},
};

const CONTROL_LEASE_TTL: Duration = Duration::from_secs(3);
const LEASE_RENEW_INTERVAL: Duration = Duration::from_millis(250);
const INPUT_POLL_INTERVAL: Duration = Duration::from_millis(50);

enum InputEvent {
    Command(String),
    EmergencyStop(Result<MotionState, ControlError>),
    Closed,
}

fn main() -> Result<(), Box<dyn Error>> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyACM0".to_owned());

    // Runtime composition root: concrete adapters are wired to application
    // ports here; the use case itself does not know about USB or the CLI.
    let motor = Arc::new(PicoUsbMotor::connect("motor-left", &port)?);
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
        Arc::new(Mutex::new(())),
    ));
    let drive: Arc<dyn DriveControl> = Arc::new(DriveUseCase::new(
        gateway,
        CommandSource::Cli,
        ControlMode::Manual,
    ));
    let mut lease = Some(drive.acquire_control(CONTROL_LEASE_TTL)?);

    println!("Connected to {port}");
    println!(
        "f=forward, b=backward, s=stop, ?=status, h=health, e=emergency stop, r=reset, q=quit"
    );

    let input = spawn_input_reader(Arc::clone(&drive));
    let mut last_renewal = Instant::now();
    let mut running = true;
    while running {
        let response = match input.recv_timeout(INPUT_POLL_INTERVAL) {
            Ok(InputEvent::Command(line)) => match line.trim() {
                "f" => Some(with_lease(lease, |active| {
                    drive.drive(active, Direction::Forward)
                })),
                "b" => Some(with_lease(lease, |active| {
                    drive.drive(active, Direction::Backward)
                })),
                "s" => Some(with_lease(lease, |active| drive.stop(active))),
                "?" => Some(drive.status()),
                "h" => {
                    println!("device statuses: {:?}", devices.statuses());
                    None
                }
                "r" => {
                    match drive.reset_safety_stop() {
                        Ok(()) => match drive.acquire_control(CONTROL_LEASE_TTL) {
                            Ok(acquired) => {
                                lease = Some(acquired);
                                last_renewal = Instant::now();
                                println!("safety stop cleared; manual control acquired");
                            }
                            Err(error) => eprintln!("could not acquire manual control: {error}"),
                        },
                        Err(error) => eprintln!("safety stop reset rejected: {error}"),
                    }
                    None
                }
                "q" => {
                    running = false;
                    None
                }
                _ => {
                    println!("unknown command");
                    None
                }
            },
            Ok(InputEvent::EmergencyStop(result)) => {
                lease = None;
                match result {
                    Ok(state) => println!("emergency stop latched: {state:?}"),
                    Err(error) => {
                        eprintln!("emergency stop failed; stop remains latched: {error}")
                    }
                }
                None
            }
            Ok(InputEvent::Closed) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                running = false;
                None
            }
            Err(mpsc::RecvTimeoutError::Timeout) => None,
        };

        if let Some(response) = response {
            match response {
                Ok(state) => print_motion_state(state),
                Err(error) => eprintln!("motion request failed: {error}"),
            }
        }

        if let Some(active) = lease {
            if last_renewal.elapsed() >= LEASE_RENEW_INTERVAL {
                match drive.renew_control(active, CONTROL_LEASE_TTL) {
                    Ok(renewed) => lease = Some(renewed),
                    Err(error) => {
                        eprintln!("control lease renewal failed; motion is stopped: {error}");
                        lease = None;
                    }
                }
                last_renewal = Instant::now();
            }
        }
    }

    if let Some(active) = lease {
        if let Err(error) = drive.release_control(active) {
            eprintln!("could not release control cleanly; requesting emergency stop: {error}");
            let _ = drive.emergency_stop(format!("CLI shutdown stop failed: {error}"));
        }
    }
    Ok(())
}

fn with_lease<T>(
    lease: Option<ControlLease>,
    action: impl FnOnce(ControlLease) -> Result<T, ControlError>,
) -> Result<T, ControlError> {
    match lease {
        Some(lease) => action(lease),
        None => Err(ControlError::Unsafe),
    }
}

fn print_motion_state(state: MotionState) {
    println!("Pico 2 -> {state:?}");
}

fn spawn_input_reader(drive: Arc<dyn DriveControl>) -> mpsc::Receiver<InputEvent> {
    let (sender, receiver) = mpsc::channel();
    thread::spawn(move || {
        loop {
            let mut line = String::new();
            match io::stdin().read_line(&mut line) {
                Ok(0) => {
                    let _ = sender.send(InputEvent::Closed);
                    break;
                }
                Ok(_) => {
                    let event = if line.trim() == "e" {
                        InputEvent::EmergencyStop(
                            drive.emergency_stop("emergency stop requested from CLI".to_owned()),
                        )
                    } else {
                        InputEvent::Command(line)
                    };
                    if sender.send(event).is_err() {
                        break;
                    }
                }
                Err(error) => {
                    eprintln!("input failed: {error}");
                    let _ = sender.send(InputEvent::Closed);
                    break;
                }
            }
        }
    });
    receiver
}
