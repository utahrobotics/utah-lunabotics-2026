#![cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]

use anyhow::{Context, Result};
use crate::utils::udev_poll;
use core::f32;
use crossbeam::{atomic::AtomicCell, utils::Backoff};
use serde::Deserialize;
use std::collections::HashMap;
use std::sync::Arc;
use std::{
    sync::mpmc::Receiver,
    time::{Duration, Instant},
};
use tasker::{
    BlockOn, get_tokio_handle,
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt, BufStream},
    },
};
use tokio_serial::SerialPortBuilderExt;
use udev::{EventType, MonitorBuilder, Udev};
use vesc_translator::GetValuesResponse;
use vesc_translator::{Alive, CanForwarded, GetValues, Getter, MinLength, SetRPM, VescPacker};

#[derive(Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
pub enum MotorMask {
    Left,
    Right,
}

impl MotorMask {
    fn mask(self, (left, right): (f32, f32)) -> f32 {
        match self {
            MotorMask::Left => left,
            MotorMask::Right => right,
        }
    }
}

#[derive(Default)]
pub struct VescIDs {
    /// Map from Can ID to sibling Can ID
    can_ids: HashMap<u8, Option<(u8, bool)>>,
    motor_masks: HashMap<u8, MotorMask>,
    device_count: usize,
}

#[derive(Deserialize, Debug)]
pub struct VescPair {
    pub id1: u8,
    pub id2: u8,
    pub mask1: MotorMask,
    pub mask2: MotorMask,
    #[serde(default = "default_command_both")]
    pub command_both: bool,
}

fn default_command_both() -> bool {
    true
}

impl VescIDs {
    pub fn add_dual_vesc(
        &mut self,
        id1: u8,
        id2: u8,
        mask1: MotorMask,
        mask2: MotorMask,
        command_both: bool,
    ) -> bool {
        if self.motor_masks.contains_key(&id1) || self.motor_masks.contains_key(&id2) {
            return true;
        }
        self.can_ids.insert(id1, Some((id2, command_both)));
        self.can_ids.insert(id2, Some((id1, command_both)));
        self.motor_masks.insert(id1, mask1);
        self.motor_masks.insert(id2, mask2);
        self.device_count += 1;
        false
    }

    #[allow(unused)]
    pub fn add_single_vesc(&mut self, id: u8, mask: MotorMask) -> bool {
        if self.motor_masks.contains_key(&id) {
            return true;
        }
        self.can_ids.insert(id, None);
        self.motor_masks.insert(id, mask);
        self.device_count += 1;
        false
    }
}

pub struct MotorRef {
    speeds: AtomicCell<Option<(f32, f32, Instant)>>,
    latest_telemetry: std::sync::Mutex<HashMap<u8, GetValuesResponse>>,
    latest_errors: std::sync::Mutex<Vec<String>>,
    speed_multiplier: Arc<AtomicCell<f32>>,
    invert_left: bool,
    invert_right: bool,
}

impl MotorRef {
    pub fn set_speed_multiplier(&self, multiplier: f32) {
        self.speed_multiplier.store(multiplier);
    }

    /// Final RPM = (left_or_right) * weight
    /// weight is the same thing as speed multiplier
    /// Commands expire after 200ms — caller must write at least 5Hz to keep motors running.
    pub fn set_speed(&self, mut left: f32, mut right: f32) {
        if self.invert_left {
            left *= -1.0;
        }
        if self.invert_right {
            right *= -1.0;
        }
        self.speeds.store(Some((left, right, Instant::now())));
    }

    /// Returns and clears any collected telemetry.
    pub fn get_latest_telemetry(&self) -> Option<Vec<GetValuesResponse>> {
        let mut map = self.latest_telemetry.lock().unwrap();
        if map.is_empty() {
            None
        } else {
            let v = map.values().cloned().collect();
            map.clear();
            Some(v)
        }
    }

    /// Pushes an error from a background thread to the queue.
    pub(crate) fn push_error(&self, err: String) {
        if let Ok(mut errors) = self.latest_errors.lock() {
            errors.push(err);
        }
    }

    /// Returns and clears any collected background errors.
    pub fn get_latest_errors(&self) -> Option<Vec<String>> {
        let mut queue = self.latest_errors.lock().unwrap();
        if queue.is_empty() {
            None
        } else {
            Some(std::mem::take(&mut *queue))
        }
    }
}

const SPEED_COMMAND_EXPIRY: Duration = Duration::from_millis(200);
pub fn enumerate_motors(
    vesc_ids: VescIDs,
    speed_multiplier: f32,
    invert_left: bool,
    invert_right: bool,
) -> Result<&'static MotorRef> {
    
    // 1. Initial Scan 
    let udev = Udev::new().context("Failed to create udev context")?;
    let mut enumerator = udev::Enumerator::with_udev(udev).context("Failed to create udev enumerator")?;
    enumerator.match_subsystem("tty").context("Failed to set match-subsystem filter")?;
    
    let initial_devices: Vec<String> = enumerator.scan_devices()
        .context("Failed to scan devices")?
        .filter_map(|device| {
            let vendor = device.property_value("ID_VENDOR")?.to_str()?;
            let serial = device.property_value("ID_SERIAL")?.to_str()?;
            
            if vendor == "STMicroelectronics" && 
               serial == "STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304" {
                return device.devnode()?.to_str().map(|s| s.to_string());
            }
            None
        })
        .collect();

    let speed_multiplier = Arc::new(AtomicCell::new(speed_multiplier));
    let motor_ref: &_ = Box::leak(Box::new(MotorRef {
        speeds: AtomicCell::new(None),
        latest_telemetry: std::sync::Mutex::new(HashMap::new()),
        latest_errors: std::sync::Mutex::new(Vec::new()),
        speed_multiplier: Arc::clone(&speed_multiplier),
        invert_left,
        invert_right,
    }));

    let (tx, rx) = std::sync::mpmc::sync_channel::<String>(1);
    let vesc_ids = Box::leak(Box::new(vesc_ids));

    for path in initial_devices {
        let _ = tx.send(path);
    }

    for _ in 0..vesc_ids.device_count {
        let mut task = MotorTask {
            path: rx.clone(),
            vesc_packer: VescPacker::default(),
            motor_ref,
            vesc_ids,
            speed_multiplier: Arc::clone(&speed_multiplier),
        };
        std::thread::spawn(move || {
            loop {
                task.motor_task();
            }
        });
    }

    std::thread::spawn(move || {
        let monitor_res = (|| -> Result<()> {
            let mut monitor = MonitorBuilder::new().context("Failed to create udev monitor")?;
            monitor = monitor.match_subsystem("tty").context("Failed to set filter")?;
            let listener = monitor.listen().context("Failed to listen")?;

            for event in udev_poll(listener) {
                if !matches!(event.event_type(), EventType::Add | EventType::Change) {
                    continue;
                }
                
                let device = event.device();
                let Some(path_str) = device.devnode().and_then(|p| p.to_str()) else { continue; };
                let vendor = device.property_value("ID_VENDOR").and_then(|s| s.to_str());
                let serial = device.property_value("ID_SERIAL").and_then(|s| s.to_str());

                if vendor == Some("STMicroelectronics") && 
                   serial == Some("STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304") {
                    let _ = tx.send(path_str.to_string());
                }
            }
            Ok(())
        })();

        if let Err(e) = monitor_res {
            motor_ref.push_error(format!("Udev hotplug monitor died: {e:?}"));
        }
    });

    Ok(motor_ref)
}

struct MotorTask {
    path: Receiver<String>,
    vesc_packer: VescPacker,
    motor_ref: &'static MotorRef,
    vesc_ids: &'static VescIDs,
    speed_multiplier: Arc<AtomicCell<f32>>,
}

impl MotorTask {
    fn motor_task(&mut self) {
        let path_str = match self.path.recv() {
            Ok(x) => x,
            Err(_) => loop {
                std::thread::park();
            },
        };
        let mut motor_port;
        {
            let _guard = get_tokio_handle().enter();
            motor_port = match tokio_serial::new(&path_str, 115200)
                .timeout(std::time::Duration::from_millis(500))
                .open_native_async()
            {
                Ok(x) => x,
                Err(e) => {
                    self.motor_ref.push_error(format!("Failed to open motor port {path_str}: {e}"));
                    return;
                }
            };
        }
        if let Err(e) = motor_port.set_exclusive(true) {
            self.motor_ref.push_error(format!(
                "Failed to set motor port {} exclusive: {}",
                &path_str,
                e.to_string()
            ));
        }
        let mut motor_port = BufStream::new(motor_port);

        let master_can_id;
        loop {
            let mut tmp_buf = [0u8; 128];
            let task = async {
                let mut response = vec![];
                motor_port
                    .write_all(self.vesc_packer.pack(&GetValues))
                    .await?;
                motor_port.flush().await?;
                while response.len() < 63 || response.last() != Some(&3) {
                    let n = motor_port.read(&mut tmp_buf).await?;
                    response.extend_from_slice(&tmp_buf[..n]);
                }
                std::io::Result::Ok(response)
            };
            let task = async {
                tokio::select! {
                    res = task => res,
                    _ = tokio::time::sleep(std::time::Duration::from_secs(2)) => {
                        Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "Timed out waiting for response"))
                    }
                }
            };
            let response = match task.block_on() {
                Ok(resp) => resp,
                Err(e) => {
                    self.motor_ref.push_error(format!("Failed to read/write to motor port {path_str}: {e}"));
                    return;
                }
            };

            let Ok(buf) = MinLength::try_from(response.as_slice()) else {
                self.motor_ref.push_error(format!("Received too short of a message from motor port {path_str}"));
                std::thread::sleep(std::time::Duration::from_secs(1));
                continue;
            };

            let Ok(values) = GetValues::parse_response(&buf) else {
                self.motor_ref.push_error(format!("Received corrupt response from motor port {path_str}"));
                std::thread::sleep(std::time::Duration::from_secs(1));
                continue;
            };
            master_can_id = values.vesc_id;
            break;
        }

        let Some(&slave_can) = self.vesc_ids.can_ids.get(&master_can_id) else {
            self.motor_ref.push_error(format!("Found unknown master Can ID {master_can_id}"));
            return;
        };

        println!("[MOTORS] Opened motor master can: {master_can_id}. Slave can: {slave_can:?}");

        if let Some((can_id, _)) = slave_can {
            loop {
                let mut tmp_buf = [0u8; 128];
                let task = async {
                    let mut response = vec![];
                    motor_port
                        .write_all(self.vesc_packer.pack(&CanForwarded {
                            can_id,
                            payload: GetValues,
                        }))
                        .await?;
                    motor_port.flush().await?;
                    while response.len() < 63 || response.last() != Some(&3) {
                        let n = motor_port.read(&mut tmp_buf).await?;
                        response.extend_from_slice(&tmp_buf[..n]);
                    }
                    std::io::Result::Ok(response)
                };
                let task = async {
                    tokio::select! {
                        res = task => res,
                        _ = tokio::time::sleep(std::time::Duration::from_secs(2)) => {
                            Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "Timed out waiting for response"))
                        }
                    }
                };
                let response = match task.block_on() {
                    Ok(resp) => resp,
                    Err(e) => {
                        self.motor_ref.push_error(format!("Failed to read/write to motor port {path_str}: {e}"));
                        return;
                    }
                };

                let Ok(buf) = MinLength::try_from(response.as_slice()) else {
                    self.motor_ref.push_error(format!("Received too short of a message from motor port {path_str}"));
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                };

                let Ok(values) = GetValues::parse_response(&buf) else {
                    self.motor_ref.push_error(format!("Received corrupt response from motor port {path_str}"));
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                };
                self.motor_ref
                    .latest_telemetry
                    .lock()
                    .unwrap()
                    .insert(values.vesc_id, values);
                if can_id != values.vesc_id {
                    self.motor_ref.push_error(format!(
                        "Received can id {} instead of {} from sibling",
                        values.vesc_id, can_id
                    ));
                    return;
                }
                break;
            }
            println!("Opened motor {} and {}", master_can_id, can_id);
        } else {
            println!("Opened motor {}", master_can_id);
        }

        let master_mask = *self.vesc_ids.motor_masks.get(&master_can_id).unwrap();
        let slave_mask =
            slave_can.map(|(can_id, _)| *self.vesc_ids.motor_masks.get(&can_id).unwrap());

        let backoff = Backoff::new();
        let mut last_read_time = Instant::now();

        let mut tmp_buf = [0u8; 128];

        loop {
            let values = loop {
                match self.motor_ref.speeds.load() {
                    Some((left, right, timestamp)) => {
                        if timestamp.elapsed() > SPEED_COMMAND_EXPIRY {
                            eprintln!("[MOTORS.RS] SETTING SPEEDS TO ZERO, SPEED COMMAND EXPIRED");
                            break (0.0, 0.0);
                        }
                        break (left, right);
                    }
                    None => backoff.snooze(),
                }
            };
            backoff.reset();

            if last_read_time.elapsed() > Duration::from_millis(500) {
                last_read_time = Instant::now();
                let task = async {
                    let mut response = vec![];
                    motor_port
                        .write_all(self.vesc_packer.pack(&GetValues))
                        .await?;
                    motor_port.flush().await?;
                    while response.len() < 63 || response.last() != Some(&3) {
                        let n = motor_port.read(&mut tmp_buf).await?;
                        response.extend_from_slice(&tmp_buf[..n]);
                    }
                    std::io::Result::Ok(response)
                };

                let task = async {
                    tokio::select! {
                        res = task => res,
                        _ = tokio::time::sleep(std::time::Duration::from_millis(90)) => {
                            Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "Timed out waiting for response"))
                        }
                    }
                };

                let response = match task.block_on() {
                    Ok(resp) => resp,
                    Err(e) => {
                        self.motor_ref.push_error(format!("Failed to read/write to motor port {path_str}: {e}"));
                        return;
                    }
                };

                let Ok(buf) = MinLength::try_from(response.as_slice()) else {
                    self.motor_ref.push_error(format!("Received too short of a message from motor port {path_str}"));
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                };

                let Ok(values) = GetValues::parse_response(&buf) else {
                    self.motor_ref.push_error(format!("Received corrupt response from motor port {path_str}"));
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                };
                if values.temp_mos > 70.0 {
                    self.motor_ref.push_error(format!(
                        "TEMPERATURE WARNING {} => {:.1} °C",
                        values.vesc_id,
                        values.temp_mos
                    ));
                }
                if values.v_in < 24.0 {
                    self.motor_ref.push_error(format!("LOW VOLT WARNING {} => {:.1}", values.vesc_id, values.v_in));
                }
                self.motor_ref
                    .latest_telemetry
                    .lock()
                    .unwrap()
                    .insert(values.vesc_id, values);
                if let Some((can_id, true)) = slave_can {
                    let task = async {
                        let mut response = vec![];
                        motor_port
                            .write_all(self.vesc_packer.pack(&CanForwarded {
                                can_id,
                                payload: GetValues,
                            }))
                            .await?;
                        motor_port.flush().await?;
                        while response.len() < 63 || response.last() != Some(&3) {
                            let n = motor_port.read(&mut tmp_buf).await?;
                            response.extend_from_slice(&tmp_buf[..n]);
                        }
                        std::io::Result::Ok(response)
                    };

                    let task = async {
                        tokio::select! {
                            res = task => res,
                            _ = tokio::time::sleep(std::time::Duration::from_millis(90)) => {
                                Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "Timed out waiting for response"))
                            }
                        }
                    };

                    let response = match task.block_on() {
                        Ok(resp) => resp,
                        Err(e) => {
                            self.motor_ref.push_error(format!("Failed to read/write to motor port {path_str}: {e}"));
                            return;
                        }
                    };

                    let Ok(buf) = MinLength::try_from(response.as_slice()) else {
                        self.motor_ref.push_error(format!("Received too short of a message from motor port {path_str}"));
                        std::thread::sleep(std::time::Duration::from_secs(1));
                        continue;
                    };

                    let Ok(values) = GetValues::parse_response(&buf) else {
                        self.motor_ref.push_error(format!("Received corrupt response from motor port {path_str}"));
                        std::thread::sleep(std::time::Duration::from_secs(1));
                        continue;
                    };
                    self.motor_ref
                        .latest_telemetry
                        .lock()
                        .unwrap()
                        .insert(values.vesc_id, values);
                    if values.temp_mos > 70.0 {
                        self.motor_ref.push_error(format!("TEMPERATURE WARNING {can_id} => {:.1} °C", values.temp_mos));
                    }
                    if values.v_in < 24.0 {
                        self.motor_ref.push_error(format!("LOW VOLT WARNING {can_id} => {:.1}", values.v_in));
                    }
                }
            }

            let task = async {
                if let Some((can_id, true)) = slave_can {
                    motor_port
                        .write_all(self.vesc_packer.pack(&CanForwarded {
                            can_id,
                            payload: SetRPM(
                                slave_mask.unwrap().mask(values) * self.speed_multiplier.load(),
                            ),
                        }))
                        .await?;
                    motor_port.write_all(self.vesc_packer.pack(&Alive)).await?;
                }
                motor_port
                    .write_all(self.vesc_packer.pack(&SetRPM(
                        master_mask.mask(values) * self.speed_multiplier.load(),
                    )))
                    .await?;
                motor_port.flush().await
            };

            if let Err(e) = task.block_on() {
                self.motor_ref.push_error(format!("Failed to write to motor port: {e}"));
                break;
            }
        }

        if let Some((slave_can_id, _)) = slave_can {
            self.motor_ref.push_error(format!("Motors {} and {} closed", master_can_id, slave_can_id));
        } else {
            self.motor_ref.push_error(format!("Motor {} closed", master_can_id));
        }
    }
}