use std::sync::Arc;
use bincode::{Decode, Encode};
use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable}, output_msg, CuResult};
use cu29::cutask::CuMsg;
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
#[cfg(target_os = "linux")]
use udev::{EventType, Udev};

#[cfg(target_os = "linux")]
pub struct UdevMonitor {
    monitor_socket: Option<udev::MonitorSocket>,
    initial_enumerated: Vec<NewDevice>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Decode, Encode)]
pub struct NewDevice {
    pub port: Box<String>,
    pub dev_path: Box<String>,
}

impl Default for NewDevice {
    fn default() -> Self {
        Self { port: Box::new("pci-0000:35:00.4-usb-0:1:1.3".to_string()), dev_path: Box::new("/dev/video4".to_string()) }
    }
}

#[cfg(target_os = "linux")]
impl Freezable for UdevMonitor {}

#[cfg(target_os = "linux")]
impl<'cl> CuSrcTask<'cl> for UdevMonitor {
    type Output = output_msg!('cl, NewDevice);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self { monitor_socket: None, initial_enumerated: Vec::new() })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let mut enumerator = {
            let udev = match Udev::new() {
                Ok(x) => x,
                Err(e) => {
                    error!("Failed to create udev context: {e}");
                    return Err(CuError::new_with_cause("Failed to create udev context", e));
                }
            };
            match udev::Enumerator::with_udev(udev) {
                Ok(x) => x,
                Err(e) => {
                    error!("Failed to create udev enumerator: {e}");
                    return Err(CuError::new_with_cause("Failed to create udev enumerator", e));
                }
            }
        };
        if let Err(e) = enumerator.match_subsystem("video4linux") {
            error!("Failed to set match-subsystem filter: {e}");
        }
        let devices = match enumerator.scan_devices() {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to scan devices: {e}");
                return Err(CuError::new_with_cause("Failed to enumerate devices", e));
            }
        };
        for device in devices {
            let Some(path) = device.devnode() else {
                continue;
            };
            // Valid camera paths are of the form /dev/videoN
            let Some(path_str) = path.to_str() else {
                continue;
            };
            if !path_str.starts_with("/dev/video") {
                continue;
            }
            let Some(udev_index) = device.attribute_value("index") else {
                info!("No udev_index for camera {}", path_str);
                continue;
            };
            if udev_index.to_str() != Some("0") {
                continue;
            }
            if let Some(name) = device.attribute_value("name") {
                if let Some(name) = name.to_str() {
                    if name.contains("RealSense") {
                        continue;
                    }
                }
            }
            let Some(port_raw) = device.property_value("ID_PATH") else {
                info!("No port for camera {}", &path_str);
                continue;
            };
            let Some(port) = port_raw.to_str() else {
                info!("Failed to parse port of camera {path_str}");
                continue;
            };
            self.initial_enumerated.push(NewDevice {
                port: Box::new(port.to_string()),
                dev_path: Box::new(path_str.to_string()),
            });
        }

        // Create a udev monitor socket listening for video4linux subsystem events.
        let monitor_socket = udev::MonitorBuilder::new()
            .and_then(|m| m.match_subsystem("video4linux"))
            .and_then(|m| m.listen())
            .map_err(|e| CuError::new_with_cause("Failed to initialize udev monitor", e))?;
        self.monitor_socket = Some(monitor_socket);
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        output.clear_payload();
        // first pop off the initial enumerated devices
        if !self.initial_enumerated.is_empty() {
            let device = self.initial_enumerated.pop().unwrap();
            info!("udev_monitor: Found device {}", &device.dev_path);
            output.set_payload(device);
            return Ok(());
        }

        if let Some(monitor_socket) = &self.monitor_socket {
            // Create an iterator from the socket and fetch the next event if any.
            if let Some(event) = monitor_socket.iter().next() {
                info!("got udev event");
                match event.event_type() {
                    EventType::Add => {
                        let Some(devnode) = event.devnode() else { return Ok(()); };
                        let Some(path_str) = devnode.to_str() else { return Ok(()); };
                        if !path_str.starts_with("/dev/video") { return Ok(()); }
                        let Some(udev_index) = event.attribute_value("index") else {
                            info!("No udev_index for camera {path_str}");
                            return Ok(());
                        };
                        if udev_index.to_str() != Some("0") { return Ok(()); }
                        if let Some(name) = event.attribute_value("name") {
                            if let Some(name) = name.to_str() {
                                if name.contains("RealSense") {
                                    return Ok(());
                                }
                            }
                        }
                        let Some(port_raw) = event.property_value("ID_PATH") else {
                            info!("No port for camera {}", &path_str);
                            return Ok(());
                        };
                        let Some(port) = port_raw.to_str() else {
                            info!("Failed to parse port of camera {path_str}");
                            return Ok(());
                        };
                        debug!("got add event for main video device {}", path_str);
                        output.set_payload(NewDevice {
                            port: Box::new(port.to_string()),
                            dev_path: Box::new(path_str.to_string()),
                        });
                    }
                    _ => {}
                }
            }
        }
        Ok(())
    }
}

// Non-Linux stub -----------------------------------------------------------
#[cfg(not(target_os = "linux"))]
#[derive(Default)]
pub struct UdevMonitor;

#[cfg(not(target_os = "linux"))]
impl Freezable for UdevMonitor {}

#[cfg(not(target_os = "linux"))]
impl<'cl> CuSrcTask<'cl> for UdevMonitor {
    type Output = output_msg!('cl, NewDevice);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> where Self: Sized {
        Ok(Self)
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> { Ok(()) }

    fn process(&mut self, _clock: &RobotClock, _output: Self::Output) -> CuResult<()> {
        // No-op stub – never emits new devices.
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> { Ok(()) }
}
