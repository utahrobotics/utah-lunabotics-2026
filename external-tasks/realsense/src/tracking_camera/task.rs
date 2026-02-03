use std::{
    collections::HashMap,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc,
    },
    time::Duration,
};

use crate::iceoryx_utils::{create_node, create_pose_frame_publisher};
use iceoryx2::{port::publisher::Publisher, service::ipc};
use iceoryx_types::{PoseMsg, T265Confidence};
use t265_rs::{Confidence, Pose};

pub struct TrackingCameraTask {
    pub target_ids: Vec<String>,
    /// calling pose_rx_handles.recv will give you a receiver for poses from an opened intel t265 once it has been connected over usb
    /// the reason for this thread channel is that the discovery of new usb devices runs in a different thread.
    pose_rx_handles: Receiver<Receiver<Pose>>,
    /// used by the discovery thread to send over a new handle for getting poses once a device is connected
    /// yes I know this is cursed but it works.
    pose_rx_tx: Sender<Receiver<Pose>>,
}

impl TrackingCameraTask {
    pub fn new(serial_numbers: &[&str]) -> Self {
        let (tx, rx) = channel();
        Self {
            target_ids: serial_numbers.iter().map(|s| s.to_string()).collect(),
            pose_rx_handles: rx,
            pose_rx_tx: tx,
        }
    }
    /// continually checks device manager until mutiple devices are available and then starts streaming poses over iceoryx2
    /// non blocking
    fn start_device_detection(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let pose_rx_tx = self.pose_rx_tx.clone();
        let target_ids = self.target_ids.clone();
        std::thread::spawn(move || {
            let mut manager = t265_rs::T265Manager::new().expect("couldn't start USB manager");

            // wait for devices
            loop {
                if let Ok(device_ids) = manager.discover_devices() {
                    if !device_ids.is_empty() {
                        for device_id in device_ids {
                            if target_ids.contains(&device_id) {
                                match manager.start_pose_stream(&device_id) {
                                    Ok(pose_rx) => {
                                        let _ = pose_rx_tx.send(pose_rx);
                                    }
                                    Err(e) => {
                                        eprintln!("Error starting pose stream for device:{device_id}: {e}");
                                    }
                                }
                            }
                        }
                    }
                }
                std::thread::sleep(Duration::from_millis(300));
            }
        });
        Ok(())
    }
}

pub fn enumerate_tracking_cameras(serial_numbers: &[&str]) {
    let mut task = TrackingCameraTask::new(serial_numbers);

    if let Err(e) = task.start_device_detection() {
        eprintln!("Failed to start T265 device detection: {}", e);
        return;
    }

    let node = Arc::new(create_node());

    loop {
        let pose_rx = match task.pose_rx_handles.recv() {
            Ok(rx) => {
                println!("T265 tracking camera connected");
                rx
            }
            Err(e) => {
                eprintln!("Pose receiver channel closed: {}", e);
                break;
            }
        };

        let node = Arc::clone(&node);
        std::thread::spawn(move || {
            let mut publishers: HashMap<String, Publisher<ipc::Service, PoseMsg, ()>> =
                HashMap::new();

            loop {
                match pose_rx.recv() {
                    Ok(pose) => {
                        let publisher =
                            publishers.entry(pose.device_id.clone()).or_insert_with(|| {
                                println!(
                                    "Creating pose publisher for T265 device: {}",
                                    pose.device_id
                                );
                                create_pose_frame_publisher(&node, &pose.device_id)
                            });

                        let Ok(serial_num) = pose.device_id.parse::<u64>() else {
                            eprintln!("serial {} was not a valid u64", pose.device_id);
                            continue;
                        };

                        let pose_msg = PoseMsg {
                            position: pose.translation,
                            quaternion: pose.rotation,
                            confidence: convert_confidence(pose.tracker_confidence),
                            serial_num
                        };
                        if let Err(e) = publisher.send_copy(pose_msg) {
                            eprintln!(
                                "Failed to publish T265 pose for device {}: {}",
                                pose.device_id, e
                            );
                        }
                    }
                    Err(e) => {
                        eprintln!("T265 pose receiver disconnected: {}", e);
                        break;
                    }
                }
            }
        });
    }
}

fn convert_confidence(confidence: Confidence) -> T265Confidence {
    match confidence {
        Confidence::Failed => T265Confidence::Failed,
        Confidence::Low => T265Confidence::Low,
        Confidence::Medium => T265Confidence::Medium,
        Confidence::High => T265Confidence::High,
    }
}
