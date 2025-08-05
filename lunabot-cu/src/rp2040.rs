use std::{
    sync::{
        Arc,
        mpsc::{Receiver, RecvTimeoutError, Sender},
    },
    time::Duration,
};

use crossbeam::atomic::AtomicCell;
use cu29::prelude::*;
use embedded_common::*;
use futures_util::StreamExt;
use nalgebra::Vector3;
use simple_motion::{Node, NodeData, StaticImmutableNode};
use tasker::{
    BlockOn, get_tokio_handle,
    tokio::{
        self,
        io::{AsyncWriteExt, BufStream},
        sync::Mutex,
        time::timeout,
    },
};
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::FramedRead;
use udev::{EventType, MonitorBuilder, Udev};

use crate::utils::{CobsCodec, udev_poll};

pub struct V3PicoInfo {
    pub serial: String,
    pub imus: [IMUInfo; 4],
}

pub struct IMUInfo {
    pub node: StaticImmutableNode,
    pub link_name: String,
}

/// find pico connected to the v3 pcb.
pub fn enumerate_v3picos(
    hinge_node: Node<&'static [NodeData]>,
    bucket_node: Node<&'static [NodeData]>,
    pico: V3PicoInfo,
) -> ActuatorController {
    let (path_tx, path_rx) = std::sync::mpsc::sync_channel::<String>(1);
    let (actuator_cmd_tx, actuator_cmd_rx) = std::sync::mpsc::channel();
    let actuator_readings: &_ = Box::leak(Box::new(AtomicCell::new(None)));
    std::thread::spawn(move || {
        let shared = SharedState {
            imus: [
                pico.imus[0].node,
                pico.imus[1].node,
                pico.imus[2].node,
                pico.imus[3].node,
            ],
            hinge_node,
            bucket_node,
        };

        let mut task = V3PicoTask {
            path: path_rx,
            actuator_command_rx: actuator_cmd_rx,
            shared: Arc::new(Mutex::new(shared)),
            actuator_readings,
        };
        loop {
            task.v3pico_task().block_on();
        }
    });
    let controller_serial = pico.serial;
    std::thread::spawn(move || {
        let mut monitor = match MonitorBuilder::new() {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to create udev monitor: {e}");
                return;
            }
        };
        monitor = match monitor.match_subsystem("tty") {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to set match-subsystem filter: {e}");
                return;
            }
        };
        let listener = match monitor.listen() {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to listen for udev events: {e}");
                return;
            }
        };

        let mut enumerator = {
            let udev = match Udev::new() {
                Ok(x) => x,
                Err(e) => {
                    error!("Failed to create udev context: {e}");
                    return;
                }
            };
            match udev::Enumerator::with_udev(udev) {
                Ok(x) => x,
                Err(e) => {
                    error!("Failed to create udev enumerator: {e}");
                    return;
                }
            }
        };
        if let Err(e) = enumerator.match_subsystem("tty") {
            error!("Failed to set match-subsystem filter: {e}");
        }
        let devices = match enumerator.scan_devices() {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to scan devices: {e}");
                return;
            }
        };

        // infinite iterator
        devices
            .into_iter()
            .chain(
                udev_poll(listener)
                    .filter(|event| event.event_type() == EventType::Add)
                    .map(|event| event.device()),
            )
            .for_each(|device| {
                let Some(path) = device.devnode() else {
                    return;
                };
                let Some(path_str) = path.to_str() else {
                    return;
                };
                let Some(serial_cstr) = device.property_value("ID_SERIAL") else {
                    return;
                };
                let Some(mut serial) = serial_cstr.to_str() else {
                    warning!("Failed to parse serial of device {}", path_str);
                    return;
                };
                let Some(tmp) = serial.strip_prefix("USR_V3PICO_") else {
                    if serial == "USR_V3PICO" {
                        warning!(
                            "Actuator controller at path {} has no serial number",
                            path_str
                        );
                        return;
                    }
                    return;
                };
                serial = tmp;

                if serial == controller_serial {
                    if path_tx.send(path_str.into()).is_err() {
                        warning!("Couldnt send controller path");
                    }
                } else {
                    warning!("Unexpected actuator with serial {}", serial);
                }
            })
    });
    ActuatorController {
        command_tx: actuator_cmd_tx,
        actuator_readings,
    }
}

pub struct ActuatorController {
    command_tx: Sender<ActuatorCommand>,
    pub actuator_readings: &'static AtomicCell<Option<ActuatorReading>>,
}

impl ActuatorController {
    pub fn send_command(
        &mut self,
        cmd: ActuatorCommand,
    ) -> Result<(), std::sync::mpsc::SendError<ActuatorCommand>> {
        //tracing::info!("called send_command on ActuatorController");
        self.command_tx.send(cmd)?;
        Ok(())
    }
}

#[derive(Clone)]
struct SharedState {
    /// imu node
    imus: [StaticImmutableNode; 4],
    hinge_node: Node<&'static [NodeData]>,
    bucket_node: Node<&'static [NodeData]>,
}

pub struct V3PicoTask {
    path: Receiver<String>,
    actuator_command_rx: std::sync::mpsc::Receiver<ActuatorCommand>,
    shared: Arc<tokio::sync::Mutex<SharedState>>,
    actuator_readings: &'static AtomicCell<Option<ActuatorReading>>,
}

impl V3PicoTask {
    pub async fn v3pico_task(&mut self) {
        let path_str = match self.path.recv() {
            Ok(x) => x,
            Err(_) => loop {
                std::thread::park();
            },
        };
        let mut port = match tokio_serial::new(&path_str, 150000)
            // .timeout(std::time::Duration::from_millis(100))
            .flow_control(tokio_serial::FlowControl::Hardware)
            .open_native_async()
        {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to open V3Pico controller port {path_str}: {e}");
                return;
            }
        };
        info!("Opened V3Pico controller port {path_str}");
        if let Err(e) = port.set_exclusive(true) {
            warning!("Failed to set V3Pico controller port {path_str} exclusive: {e}");
        }

        let port = BufStream::new(port);
        let (reader, mut writer) = tokio::io::split(port);
        let mut reader = FramedRead::new(reader, CobsCodec {});
        let shared = Arc::clone(&self.shared);
        let (is_broken_tx, is_broken_rx) = tokio::sync::watch::channel(false);
        let actuator_readings = self.actuator_readings;
        get_tokio_handle().spawn(async move {
            let guard = shared.lock().await;
            let mut no_reading_count = 0;
            loop {
                //TODO: figure out some way to check if the pico is disconnected here
                tokio::time::sleep(std::time::Duration::from_millis(IMU_READING_DELAY_MS - 1))
                    .await;
                let Ok(reading) = timeout(Duration::from_millis(200), reader.next()).await else {
                    error!("Pico has become unresponsive.");
                    let _ = is_broken_tx.send(true);
                    break;
                };
                if let Some(Err(e)) = reading {
                    let _ = is_broken_tx.send(true);
                    error!("failed to read from pico: {}", e.to_string());
                    break;
                }
                if let None = reading {
                    no_reading_count += 1;
                    if no_reading_count <= 5 {
                        let _ = is_broken_tx.send(true);
                        error!("Pico has become unresponsive. (no reading count)");
                        break;
                    }
                    continue;
                }
                let reading = reading.unwrap().unwrap();
                let Ok(reading) = reading.try_into() else {
                    warning!("not 105 bytes");
                    continue;
                };
                let Ok(reading) = FromPicoV3::deserialize(reading) else {
                    error!("Failed to deserialize message from picov3 serial port");
                    let _ = is_broken_tx.send(true);
                    match powercycle_ioctl() {
                        Ok(_) => {}
                        Err(e) => {
                            error!("ioctl failed: {}", e.to_string());
                        }
                    }
                    break;
                };

                if let FromPicoV3::Reading(imu_readings, actuators) = reading {
                    let lift_hinge_angle = actuators.m1_reading as f64 * 0.00743033 - 2.19192;
                    actuator_readings.store(Some(actuators));
                    let bucket_angle: f64 =
                        -29.36 - 45.77 * (0.00048836 * actuators.m2_reading as f64 - 1.079).tan();
                    //tracing::info!("lift angle: {}", lift_hinge_angle);
                    guard
                        .hinge_node
                        .set_angle_one_axis(lift_hinge_angle.to_radians());
                    guard
                        .bucket_node
                        .set_angle_one_axis(bucket_angle.to_radians());
                    for (i, (msg, node)) in imu_readings.into_iter().zip(guard.imus).enumerate() {
                        match msg {
                            FromIMU::Reading(rate, accel) => {
                                let rotation = node.get_isometry_from_base().rotation.cast();
                                let angular_velocity = Vector3::new(-rate.x, rate.z, rate.y);
                                let transformed_rate = rotation * angular_velocity;

                                let accel = Vector3::new(accel.x, -accel.z, -accel.y);

                                let transformed_accel = rotation * accel * 9.8;
                                // info!("imu{} {:?}", i, transformed_accel);

                                todo!()
                            }
                            FromIMU::NoDataReady => {
                                // warning!("No data ready");
                                continue;
                            }
                            FromIMU::Error => {
                                // error!("IMU reported error");
                                continue;
                            }
                        }
                    }
                } else {
                    error!("V3 pico reported an error");
                    let _ = is_broken_tx.send(true);
                    break;
                }
            }
        });

        loop {
            if *is_broken_rx.borrow() {
                break;
            }
            let cmd_result = self
                .actuator_command_rx
                .recv_timeout(Duration::from_secs(1));
            let Ok(cmd) = cmd_result else {
                if cmd_result.err().unwrap() != RecvTimeoutError::Timeout {
                    error!("Actuator command thread channel closed");
                }
                continue;
            };
            if let Err(e) = writer.write_all(&cmd.serialize()).await {
                error!("Failed to write to actuator port {e}");
                break;
            }
            if let Err(e) = writer.flush().await {
                error!("Failed to flush to actuator port {e}");
                break;
            }
        }
        // if let Ok(exists) = fs::exists(&path_str) {
        //     if exists {
        //         // if the port still exists try power cycling it
        //         warning!("trying to power cycle device...");
        //         if let Err(e) = power_cycle(&path_str) {
        //             warning!("failed to power cycle: {}", e);
        //         }
        //     }
        // }
    }
}

pub fn powercycle_ioctl() -> Result<(), std::io::Error> {
    let _ = std::process::Command::new("usb-reset").spawn()?;
    std::thread::sleep(Duration::from_secs_f32(0.02));
    Ok(())
}
