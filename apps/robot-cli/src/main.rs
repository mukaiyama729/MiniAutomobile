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
use std::{error::Error, future::Future, sync::Arc, time::Duration};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    sync::{mpsc, watch},
    time::{self, Instant},
};

const CONTROL_LEASE_TTL: Duration = Duration::from_secs(3);
const LEASE_RENEW_INTERVAL: Duration = Duration::from_millis(250);

enum InputEvent {
    Command(String),
    EmergencyStop(Result<MotionState, ControlError>),
    Closed,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyACM0".to_owned());

    // Runtime composition root: concrete adapters are wired to application
    // ports here; the use case does not know about USB or the CLI.
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
    let mut lease = Some(drive.acquire_control(CONTROL_LEASE_TTL).await?);
    let mut next_renewal = Instant::now() + LEASE_RENEW_INTERVAL;

    println!("Connected to {port}");
    println!(
        "f=forward, b=backward, s=stop, ?=status, h=health, e=emergency stop, r=reset, q=quit"
    );

    let (mut input, mut quit) = spawn_input_reader(Arc::clone(&drive));
    let mut running = true;
    while running {
        let event = tokio::select! {
            biased;
            result = quit.changed() => {
                if result.is_err() || *quit.borrow() {
                    running = false;
                }
                continue;
            }
            _ = time::sleep_until(next_renewal), if lease.is_some() => {
                next_renewal = Instant::now() + LEASE_RENEW_INTERVAL;
                renew_lease(&drive, &mut lease).await;
                continue;
            }
            event = input.recv() => event,
        };

        if matches!(&event, Some(InputEvent::Command(_))) && Instant::now() >= next_renewal {
            next_renewal = Instant::now() + LEASE_RENEW_INTERVAL;
            renew_lease(&drive, &mut lease).await;
        }

        match event {
            Some(InputEvent::Command(line)) => match line.trim() {
                "f" => {
                    let active_lease = lease;
                    let action_drive = Arc::clone(&drive);
                    let result =
                        while_renewing(&drive, &mut lease, &mut next_renewal, async move {
                            match active_lease {
                                Some(active) => {
                                    action_drive.drive(active, Direction::Forward).await
                                }
                                None => Err(ControlError::Unsafe),
                            }
                        })
                        .await;
                    report_motion(result);
                }
                "b" => {
                    let active_lease = lease;
                    let action_drive = Arc::clone(&drive);
                    let result =
                        while_renewing(&drive, &mut lease, &mut next_renewal, async move {
                            match active_lease {
                                Some(active) => {
                                    action_drive.drive(active, Direction::Backward).await
                                }
                                None => Err(ControlError::Unsafe),
                            }
                        })
                        .await;
                    report_motion(result);
                }
                "s" => {
                    let active_lease = lease;
                    let action_drive = Arc::clone(&drive);
                    let result =
                        while_renewing(&drive, &mut lease, &mut next_renewal, async move {
                            match active_lease {
                                Some(active) => action_drive.stop(active).await,
                                None => Err(ControlError::Unsafe),
                            }
                        })
                        .await;
                    report_motion(result);
                }
                "?" => {
                    let action_drive = Arc::clone(&drive);
                    let result =
                        while_renewing(&drive, &mut lease, &mut next_renewal, async move {
                            action_drive.status().await
                        })
                        .await;
                    report_motion(result);
                }
                "h" => println!("device statuses: {:?}", devices.statuses()),
                "r" => match drive.reset_safety_stop().await {
                    Ok(()) => match drive.acquire_control(CONTROL_LEASE_TTL).await {
                        Ok(acquired) => {
                            lease = Some(acquired);
                            next_renewal = Instant::now() + LEASE_RENEW_INTERVAL;
                            println!("safety stop cleared; manual control acquired");
                        }
                        Err(error) => eprintln!("could not acquire manual control: {error}"),
                    },
                    Err(error) => eprintln!("safety stop reset rejected: {error}"),
                },
                "q" => running = false,
                _ => println!("unknown command"),
            },
            Some(InputEvent::EmergencyStop(result)) => {
                lease = None;
                match result {
                    Ok(state) => println!("emergency stop latched: {state:?}"),
                    Err(error) => {
                        eprintln!("emergency stop failed; stop remains latched: {error}")
                    }
                }
            }
            Some(InputEvent::Closed) | None => running = false,
        }
    }

    if let Some(active) = lease {
        if let Err(error) = drive.release_control(active).await {
            eprintln!("could not release control cleanly; requesting emergency stop: {error}");
            let _ = drive
                .emergency_stop(format!("CLI shutdown stop failed: {error}"))
                .await;
        }
    }
    Ok(())
}

async fn renew_lease(drive: &Arc<dyn DriveControl>, lease: &mut Option<ControlLease>) {
    if let Some(active) = *lease {
        match drive.renew_control(active, CONTROL_LEASE_TTL).await {
            Ok(renewed) => *lease = Some(renewed),
            Err(error) => {
                eprintln!("control lease renewal failed; relinquishing control: {error}");
                *lease = None;
            }
        }
    }
}

async fn while_renewing<F, T>(
    drive: &Arc<dyn DriveControl>,
    lease: &mut Option<ControlLease>,
    next_renewal: &mut Instant,
    operation: F,
) -> Result<T, ControlError>
where
    F: Future<Output = Result<T, ControlError>>,
{
    tokio::pin!(operation);
    loop {
        if lease.is_none() {
            return operation.await;
        }

        tokio::select! {
            biased;
            result = &mut operation => return result,
            _ = time::sleep_until(*next_renewal) => {
                *next_renewal = Instant::now() + LEASE_RENEW_INTERVAL;
                renew_lease(drive, lease).await;
            }
        }
    }
}

fn report_motion(result: Result<MotionState, ControlError>) {
    match result {
        Ok(state) => print_motion_state(state),
        Err(error) => eprintln!("motion request failed: {error}"),
    }
}

fn print_motion_state(state: MotionState) {
    println!("Pico 2 -> {state:?}");
}

fn spawn_input_reader(
    drive: Arc<dyn DriveControl>,
) -> (mpsc::Receiver<InputEvent>, watch::Receiver<bool>) {
    let (sender, receiver) = mpsc::channel(32);
    let (quit_tx, quit_rx) = watch::channel(false);
    tokio::spawn(async move {
        let stdin = tokio::io::stdin();
        let mut lines = BufReader::new(stdin).lines();
        loop {
            match lines.next_line().await {
                Ok(Some(line)) => {
                    if line.trim() == "e" {
                        let result = drive
                            .emergency_stop("emergency stop requested from CLI".to_owned())
                            .await;
                        if sender
                            .send(InputEvent::EmergencyStop(result))
                            .await
                            .is_err()
                        {
                            break;
                        }
                    } else if line.trim() == "q" {
                        let _ = quit_tx.send(true);
                        break;
                    } else {
                        match sender.try_send(InputEvent::Command(line)) {
                            Ok(()) => {}
                            Err(mpsc::error::TrySendError::Full(InputEvent::Command(command)))
                                if command.trim() == "s" =>
                            {
                                eprintln!("input queue full; escalating stop to emergency stop");
                                let result = drive
                                    .emergency_stop(
                                        "CLI stop escalated because its input queue was full"
                                            .to_owned(),
                                    )
                                    .await;
                                if sender
                                    .send(InputEvent::EmergencyStop(result))
                                    .await
                                    .is_err()
                                {
                                    break;
                                }
                            }
                            Err(mpsc::error::TrySendError::Full(_)) => {
                                eprintln!("input queue full; dropping command");
                            }
                            Err(mpsc::error::TrySendError::Closed(_)) => break,
                        }
                    }
                }
                Ok(None) => {
                    let _ = sender.send(InputEvent::Closed).await;
                    break;
                }
                Err(error) => {
                    eprintln!("input failed: {error}");
                    let _ = sender.send(InputEvent::Closed).await;
                    break;
                }
            }
        }
    });
    (receiver, quit_rx)
}
