use crate::depth_camera::DepthCameraTask;
use fxhash::{FxHashMap, FxHashSet};
use realsense_rust::{
    config::{Config, ConfigurationError},
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::{ActivePipeline, InactivePipeline},
};
use std::{
    collections::HashSet,
    sync::mpsc::{SyncSender, TryRecvError, TrySendError},
    time::Duration,
};

/// Auto-discovers any RealSense device that gets plugged in and spawns a depth camera task
/// for it. Tasks persist across disconnects so hot replug keeps working: when a device drops,
/// the task loops back and asks for a new pipeline, and the discovery loop supplies one as
/// soon as the device reappears.
pub fn enumerate_depth_cameras() {
    let (init_tx, init_rx) = std::sync::mpsc::channel::<&'static str>();

    let context = match realsense_rust::context::Context::new() {
        Ok(x) => x,
        Err(_e) => {
            eprintln!("Failed to get RealSense Context: {_e}");
            return;
        }
    };

    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(move || {
            let mut threads: FxHashMap<&'static str, SyncSender<ActivePipeline>> =
                FxHashMap::default();
            let mut pending: FxHashSet<&'static str> = FxHashSet::default();

            loop {
                // Drain any init requests coming from existing tasks.
                loop {
                    match init_rx.try_recv() {
                        Ok(serial) => {
                            pending.insert(serial);
                        }
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => return,
                    }
                }

                let mut product_mask = HashSet::new();
                product_mask.insert(Rs2ProductLine::Depth);

                for device in context.query_devices(product_mask) {
                    let Some(serial_cstr) = device.info(Rs2CameraInfo::SerialNumber) else {
                        eprintln!("Failed to get serial number for RealSense Camera");
                        continue;
                    };
                    let Ok(serial_str) = serial_cstr.to_str() else {
                        eprintln!("Failed to parse serial number {:?}", serial_cstr);
                        continue;
                    };

                    let serial: &'static str = match threads.get_key_value(serial_str) {
                        Some((k, _)) => *k,
                        None => {
                            let serial: &'static str =
                                Box::leak(serial_str.to_string().into_boxed_str());
                            let (tx, rx) = std::sync::mpsc::sync_channel(1);
                            let task_init_tx = init_tx.clone();
                            std::thread::Builder::new()
                                .stack_size(16 * 1024 * 1024)
                                .spawn(move || {
                                    let mut camera_task = DepthCameraTask {
                                        pipeline: rx,
                                        serial,
                                        init_tx: task_init_tx,
                                    };
                                    camera_task.run();
                                })
                                .expect("Failed to spawn camera task thread");
                            threads.insert(serial, tx);
                            println!("Discovered new RealSense camera {serial}");
                            serial
                        }
                    };

                    if !pending.contains(serial) {
                        continue;
                    }

                    if device.info(Rs2CameraInfo::ProductLine).is_none() {
                        continue;
                    }

                    let Some(pipeline_sender) = threads.get(serial) else {
                        pending.remove(serial);
                        continue;
                    };

                    let mut config = Config::new();
                    if let Err(e) = config.disable_all_streams() {
                        eprintln!("Failed to disable all streams for {serial}: {e}");
                        continue;
                    }
                    if let Err(e) = config.enable_device_from_serial(serial_cstr) {
                        eprintln!("Failed to bind config to serial {serial}: {e}");
                        continue;
                    }
                    if let Err(e) = enable_depth_stream(&mut config) {
                        eprintln!("Failed to enable depth stream for {serial}: {e}");
                        continue;
                    }

                    let pipeline = match InactivePipeline::try_from(&context) {
                        Ok(x) => x,
                        Err(e) => {
                            eprintln!("Failed to open pipeline for {serial}: {e}");
                            continue;
                        }
                    };
                    let pipeline = match pipeline.start(Some(config)) {
                        Ok(x) => x,
                        Err(e) => {
                            eprintln!("Failed to start pipeline for {serial}: {e}");
                            std::process::Command::new("device_reset").spawn();
                            continue;
                        }
                    };

                    match pipeline_sender.try_send(pipeline) {
                        Ok(()) => {
                            pending.remove(serial);
                        }
                        Err(TrySendError::Full(p)) => {
                            p.stop();
                            pending.remove(serial);
                        }
                        Err(TrySendError::Disconnected(p)) => {
                            p.stop();
                            threads.remove(serial);
                            pending.remove(serial);
                        }
                    }
                }

                std::thread::sleep(Duration::from_millis(500));
            }
        })
        .expect("Failed to spawn device discovery thread");
}

fn enable_depth_stream(config: &mut Config) -> Result<(), ConfigurationError> {
    config.enable_stream(Rs2StreamKind::Depth, None, 640, 480, Rs2Format::Z16, 30)?;
    Ok(())
}
