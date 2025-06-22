use anyhow::Result;
use std::time::Duration;
use udev::{Enumerator, MonitorBuilder, EventType};

fn print_existing() -> Result<()> {
    let udev = udev::Udev::new()?;
    let mut enumr = Enumerator::with_udev(udev)?;
    enumr.match_subsystem("video4linux")?;
    for dev in enumr.scan_devices()? {
        if let (Some(node), Some(id_path)) = (dev.devnode(), dev.property_value("ID_PATH")) {
            println!("EXISTING {:<4} => {}", node.display(), id_path.to_string_lossy());
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    println!("Camera discovery utility — plug / unplug USB cameras to see their ports\n");

    print_existing()?;

    let monitor = MonitorBuilder::new()?
        .match_subsystem("video4linux")?
        .listen()?;

    println!("Listening for udev events… (Ctrl-C to quit)\n");

    loop {
        for event in monitor.iter() {
            if let Some(node) = event.devnode() {
                let id_path = event.property_value("ID_PATH").map(|s| s.to_string_lossy());
                match event.event_type() {
                    EventType::Add => {
                        println!("ADD  {:<4} => {}", node.display(), id_path.unwrap_or_default());
                    }
                    EventType::Remove => {
                        println!("REM  {:<4}", node.display());
                    }
                    EventType::Change => {
                        println!("CHG  {:<4}", node.display());
                    }
                    _ => {}
                }
            }
        }
        std::thread::sleep(Duration::from_millis(200));
    }

    Ok(())
} 