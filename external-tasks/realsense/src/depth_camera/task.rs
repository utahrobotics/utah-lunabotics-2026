use std::{
    process::Command,
    sync::mpsc::{Receiver, Sender},
    time::Duration,
};

use crate::{
    constants::DEPTH_FRAME_SIZE,
    iceoryx_utils::{
        create_depth_frame_publisher, create_imu_frame_publisher, create_node,
        create_pose_frame_publisher,
    },
};
use iceoryx2::{port::publisher::Publisher, service::ipc};
use iceoryx_types::{IceoryxDepthFrame, ImuMsg, PoseMsg};
use realsense_rust::{
    device::Device,
    frame::{self, DepthFrame, PixelKind},
    kind::Rs2Format,
    pipeline::{ActivePipeline, FrameWaitError},
};

pub struct DepthCameraTask {
    pub pipeline: Receiver<ActivePipeline>,
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
            let pipeline = match self.pipeline.recv() {
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
            self.start_stream(pipeline);
        }
    }

    /// reads in depth frames and publishes them on realsense/serial_num/depths
    fn start_stream(&mut self, mut pipeline: ActivePipeline) {
        let mut depth_format = None;
        let mut has_depth = false;

        for stream in pipeline.profile().streams() {
            let is_depth = match stream.format() {
                Rs2Format::Z16 => {
                    has_depth = true;
                    true
                }
                _format => {
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

        // It's valid for a device to provide only pose (e.g. T265) or only depth.
        // Record whether we saw depth/pose and continue — create publishers conditionally.
        let focal_length_px = if let Some(ref df) = depth_format {
            if df.fx() != df.fy() {
                eprintln!("Depth camera {} has unequal fx and fy", self.serial);
                (df.fx() + df.fy()) / 2.0
            } else {
                df.fx()
            }
        } else {
            0.0
        };

        let node = create_node();
        // create publishers only for streams that exist on this device
        let depth_publisher: Option<
            Publisher<ipc::Service, IceoryxDepthFrame<DEPTH_FRAME_SIZE>, ()>,
        > = if has_depth {
            Some(create_depth_frame_publisher::<DEPTH_FRAME_SIZE>(
                &node,
                self.serial,
            ))
        } else {
            None
        };
        let imu_publisher = create_imu_frame_publisher(&node, self.serial);

        if has_depth {
            if let Some(df) = depth_format {
                println!("RealSense Camera {} opened with (fx, fy) = ({:.0}, {:.0}), (width, height) = ({:.0}, {:.0})",
                      self.serial, df.fx(), df.fy(), df.width(), df.height());
            }
        } else {
            eprintln!(
                "RealSense Camera {} opened with no depth detected",
                self.serial
            );
        }

        loop {
            let frames = match pipeline.wait(Some(Duration::from_millis(2000))) {
                Ok(x) => x,
                Err(e) => {
                    eprintln!(
                        "Failed to get frame from RealSense Camera {}: {e}",
                        self.serial
                    );
                    if matches!(e, FrameWaitError::DidTimeoutBeforeFrameArrival) {
                        let _ = Command::new("usb-reset").arg("depth").spawn();
                        // device.hardware_reset();
                    }
                    break;
                }
            };

            for frame in frames.frames_of_type::<frame::AccelFrame>() {
                let accel = frame.acceleration();
                if let Err(e) = imu_publisher.send_copy(ImuMsg {
                    quaternion: [0.0; 4],
                    angular_velocity: [0.0; 3],
                    linear_acceleration: *accel,
                }) {
                    eprintln!("Failed to publish imu msg: {e}");
                }
            }

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
                let Ok(depth_scale) = frame.depth_units() else {
                    continue;
                };
                if let Some(pub_ref) = &depth_publisher {
                    if let Err(e) = pub_ref.send_copy(IceoryxDepthFrame {
                        depths: slice.try_into().unwrap(),
                        depth_scale,
                        focal_len: (focal_length_px, focal_length_px),
                    }) {
                        eprintln!("failed to publish depth fraime: {e}");
                    }
                }
            }
        }

        eprintln!("RealSense Camera {} closed", self.serial);
    }
}
