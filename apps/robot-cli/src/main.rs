use adapter_pico_usb::PicoUsbMotor;
use robot_application::usecases::DriveUseCase;
use robot_device_manager::{Device, DeviceManager};
use robot_domain::Direction;
use std::{error::Error, io::{self, Write}, sync::Arc};

fn main() -> Result<(), Box<dyn Error>> {
    let port = std::env::args().nth(1).unwrap_or_else(|| "/dev/ttyACM0".to_owned());
    let motor = Arc::new(PicoUsbMotor::connect("motor-left", &port)?);
    let mut manager = DeviceManager::new();
    let managed_device: Arc<dyn Device> = motor.clone();
    manager.register(managed_device)?;
    let drive = DriveUseCase::new(motor);

    println!("Connected to {port}");
    println!("f=forward, b=backward, s=stop, ?=status, h=health, q=quit");
    loop {
        print!("command > ");
        io::stdout().flush()?;
        let mut input = String::new();
        if io::stdin().read_line(&mut input)? == 0 {
            let _ = drive.stop();
            break;
        }
        let result = match input.trim() {
            "f" => Some(drive.drive(Direction::Forward)),
            "b" => Some(drive.drive(Direction::Backward)),
            "s" => Some(drive.stop()),
            "?" => Some(drive.status()),
            "h" => { println!("device statuses: {:?}", manager.statuses()); None },
            "q" => { let _ = drive.stop(); break; },
            _ => { println!("unknown command"); None },
        };
        if let Some(response) = result {
            match response {
                Ok(state) => println!("Pico 2 -> {state:?}"),
                Err(e) => eprintln!("communication failed: {e}"),
            }
        }
    }
    Ok(())
}
