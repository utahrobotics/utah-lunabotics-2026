use crate::depth_camera::DepthCameraTask;
use fxhash::FxHashMap;
use realsense_rust::{
    config::Config,
    device::Device,
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::{ActivePipeline, InactivePipeline},
};
use std::sync::mpsc::SyncSender;

/// waits for a realsense to be plugged in and then starts a depth camera task that will publish depth frames
pub fn enumerate_depth_cameras(serial_numbers: &[&str]) {
    let (init_tx, init_rx) = std::sync::mpsc::channel::<&'static str>();
    // start one depth camera task thread per serial number
    let mut threads: FxHashMap<&str, SyncSender<(Device, ActivePipeline)>> = serial_numbers
        .into_iter()
        .filter_map(|serial| {
            let serial: &'static str = Box::leak((*serial).to_string().into_boxed_str());
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            let init_tx = init_tx.clone();

            std::thread::Builder::new()
                .stack_size(16 * 1024 * 1024)
                .spawn(move || {
                    let mut camera_task = DepthCameraTask {
                        pipeline: rx,
                        serial,
                        init_tx,
                    };
                    camera_task.run();
                })
                .expect("Failed to spawn camera task thread");
            Some((serial, tx))
        })
        .collect();

    let context = match realsense_rust::context::Context::new() {
        Ok(x) => x,
        Err(_e) => {
            eprintln!("Failed to get RealSense Context: {_e}");
            return;
        }
    };
    let device_hub = match context.create_device_hub() {
        Ok(x) => x,
        Err(_e) => {
            eprintln!("Failed to create RealSense DeviceHub: {_e}");
            return;
        }
    };

    // Waits until a depth camera task asks for initialization, then sets up the device enabling the right streams and sends the pipeline back over to the depth camera task
    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(move || loop {
            let Ok(target_serial) = init_rx.recv() else {
                break;
            };
            loop {
                let device = match device_hub.wait_for_device() {
                    Ok(x) => x,
                    Err(_e) => {
                        eprintln!("Failed to wait for RealSense device: {_e}");
                        break;
                    }
                };

                let Some(current_serial_cstr) = device.info(Rs2CameraInfo::SerialNumber) else {
                    eprintln!("Failed to get serial number for RealSense Camera");
                    continue;
                };
                let Ok(current_serial_str) = current_serial_cstr.to_str() else {
                    eprintln!("Failed to parse serial number {:?}", current_serial_cstr);
                    continue;
                };
                if target_serial != current_serial_str {
                    continue;
                }

                let current_serial = current_serial_str.to_string();

                let Some(pipeline_sender) = threads.get(current_serial_str) else {
                    eprintln!("Unexpected RealSense camera with serial {}", current_serial);
                    continue;
                };

                let Some(usb_cstr) = device.info(Rs2CameraInfo::UsbTypeDescriptor) else {
                    eprintln!(
                        "Failed to read USB type descriptor for RealSense Camera {}",
                        current_serial
                    );
                    continue;
                };
                let Ok(usb_str) = usb_cstr.to_str() else {
                    eprintln!(
                        "USB type descriptor for RealSense Camera {} is not utf-8",
                        current_serial
                    );
                    continue;
                };
                let Ok(_usb_val) = usb_str.parse::<f32>() else {
                    eprintln!(
                        "USB type descriptor for RealSense Camera {} is not f32",
                        current_serial
                    );
                    continue;
                };

                let pipeline_sender = pipeline_sender.clone();

                let mut config = Config::new();

                if let Err(e) = config.disable_all_streams() {
                    eprintln!("Failed to disable all streams: {}", e);
                    continue;
                }

                if let Err(e) =
                    config.enable_stream(Rs2StreamKind::Depth, None, 640, 480, Rs2Format::Z16, 30)
                {
                    eprintln!("Failed to enable depth stream: {}", e);
                    continue;
                }

                if let Err(e) =
                    config.enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::Any, 0)
                {
                    eprintln!("Failed to enable imu stream: {}", e);
                    continue;
                }

                let pipeline = match InactivePipeline::try_from(&context) {
                    Ok(x) => x,
                    Err(e) => {
                        eprintln!("Failed to open pipeline: {}", e);
                        continue;
                    }
                };
                let pipeline = match pipeline.start(Some(config)) {
                    Ok(x) => x,
                    Err(e) => {
                        eprintln!("Failed to start pipeline: {}", e);
                        continue;
                    }
                };

                if let Err(error) = pipeline_sender.send((device, pipeline)) {
                    error.0 .1.stop();
                    threads.remove(current_serial.as_str());
                }
                break;
            }
        })
        .expect("Failed to spawn device hub thread");
}
