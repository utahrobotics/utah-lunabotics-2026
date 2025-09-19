use std::{
    sync::mpsc::{Receiver, Sender},
    time::Duration,
};

use crate::{
    constants::DEPTH_FRAME_SIZE,
    iceoryx_utils::{create_depth_frame_publisher, create_node},
};
use iceoryx_types::IceoryxDepthFrame;
use realsense_rust::{
    device::Device,
    frame::{DepthFrame, PixelKind},
    kind::Rs2Format,
    pipeline::{ActivePipeline, FrameWaitError},
};

pub struct DepthCameraTask {
    pub pipeline: Receiver<(Device, ActivePipeline)>,
    pub serial: &'static str,
    pub init_tx: Sender<&'static str>,
}

impl DepthCameraTask {
    /// waits until a realsense device is connected, then signals for the device to be opened and the depth frame stream to be enabled,
    /// then waits for the pipeline, then once the pipeline arrives it starts grabbing depth frames and publishing them on an iceoryx2 topic
    /// exits only after a panic (I sure hope it never panics) or if the pipeline channel closes.
    pub fn run(&mut self) {
        loop {
            // ask the thread in enumerate.rs to create the pipeline using the rust realsense lib
            let _ = self.init_tx.send(self.serial);
            // get the device and pipeline once it has been initialized
            let (device, pipeline) = match self.pipeline.recv() {
                Ok(x) => {
                    println!("Received device and pipeline for camera {}", self.serial);
                    x
                }
                Err(_) => {
                    eprintln!("Pipeline channel closed for camera {}", self.serial);
                    return;
                }
            };
            // start publishing frames until the device is removed
            self.start_stream(device, pipeline);
        }
    }

    /// reads in depth frames and publishes them on realsense/serial_num/depths
    fn start_stream(&mut self, device: Device, mut pipeline: ActivePipeline) {
        let mut depth_format = None;

        for stream in pipeline.profile().streams() {
            let is_depth = match stream.format() {
                Rs2Format::Z16 => true,
                _format => {
                    eprintln!("Unexpected format {_format:?} for {}", self.serial);
                    continue;
                }
            };
            let intrinsics = match stream.intrinsics() {
                Ok(x) => x,
                Err(_e) => {
                    if is_depth {
                        eprintln!(
                            "Failed to get depth intrinsics for RealSense camera {}: {_e}",
                            self.serial
                        );
                    }
                    continue;
                }
            };
            if is_depth {
                depth_format = Some(intrinsics);
            }
        }

        let Some(depth_format) = depth_format else {
            eprintln!(
                "Depth stream missing after initialization of {}",
                self.serial
            );
            return;
        };

        let focal_length_px;
        if depth_format.fx() != depth_format.fy() {
            eprintln!("Depth camera {} has unequal fx and fy", self.serial);
            focal_length_px = (depth_format.fx() + depth_format.fy()) / 2.0;
        } else {
            focal_length_px = depth_format.fx();
        }

        let node = create_node();
        let depth_publisher = create_depth_frame_publisher::<DEPTH_FRAME_SIZE>(&node, self.serial);

        println!("RealSense Camera {} opened with (fx, fy) = ({:.0}, {:.0}), (width, height) = ({:.0}, {:.0})",
              self.serial, depth_format.fx(), depth_format.fy(), depth_format.width(), depth_format.height());

        loop {
            let frames = match pipeline.wait(Some(Duration::from_millis(2000))) {
                Ok(x) => x,
                Err(e) => {
                    eprintln!(
                        "Failed to get frame from RealSense Camera {}: {e}",
                        self.serial
                    );
                    if matches!(e, FrameWaitError::DidTimeoutBeforeFrameArrival) {
                        device.hardware_reset();
                    }
                    break;
                }
            };

            for frame in frames.frames_of_type::<DepthFrame>() {
                if !matches!(frame.get(0, 0), Some(PixelKind::Z16 { .. })) {
                    eprintln!("Unexpected depth pixel kind for camera {}", self.serial);
                }
                debug_assert_eq!(frame.bits_per_pixel(), 16);
                debug_assert_eq!(frame.width() * frame.height() * 2, frame.get_data_size());

                let slice;
                unsafe {
                    let data: *const _ = frame.get_data();
                    slice = std::slice::from_raw_parts(
                        data.cast::<u16>(),
                        frame.width() * frame.height(),
                    );
                }
                depth_publisher.send_copy(IceoryxDepthFrame {
                    depths: slice.try_into().unwrap(),
                });
            }
        }

        eprintln!("RealSense Camera {} closed", self.serial);
    }
}
