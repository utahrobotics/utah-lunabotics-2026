use std::{
    cell::OnceCell,
    num::NonZeroU32,
    ops::Deref,
    sync::mpsc::{Receiver, Sender},
    time::Duration,
};

use common::{THALASSIC_CELL_SIZE, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use crossbeam::atomic::AtomicCell;
use cu_spatial_payloads::EncodableIsometry;
use gputter::types::{AlignedMatrix4, AlignedVec4};
use iceoryx2::{port::subscriber::Subscriber, service::ipc};

use crate::iceoryx_utils::{
    create_cloud_publisher, create_isometry_subscriber, create_node, create_occupancy_publisher,
};

use iceoryx_types::{IceoryxOccupancyGrid, IceoryxPointCloud, PointXYZIR, MAX_POINT_CLOUD_POINTS};
use nalgebra::{Isometry3, Vector2, Vector4};
use realsense_rust::{
    device::Device,
    frame::{DepthFrame, PixelKind},
    kind::Rs2Format,
    pipeline::{ActivePipeline, FrameWaitError},
};
use thalassic::{
    DepthProjector, DepthProjectorBuilder, Occupancy, OccupancyGridPipeline,
    OccupancyGridPipelineBuilder, ThalassicPipelineRef,
};

use crate::constants::{
    MIN_KNOWN_NEIGHBORS_RATIO, NEIGHBORHOOD_RADIUS, OBSTACLE_THRESHOLD, ROBOT_RADIUS,
};

pub struct DepthCameraState {
    pub depth_projector: DepthProjector,
    pub point_cloud: Box<[AlignedVec4<f32>]>,
    pub cloud_publisher: iceoryx2::port::publisher::Publisher<ipc::Service, IceoryxPointCloud, ()>,
    pub occupancy_pipeline: Option<OccupancyGridPipeline>,
    pub occupancy_grid: Option<Box<[Occupancy]>>,
    pub occupancy_publisher:
        Option<iceoryx2::port::publisher::Publisher<ipc::Service, IceoryxOccupancyGrid, ()>>,
    pub realsense_isometry_subscriber: Subscriber<ipc::Service, [f64; 16], ()>,
}

pub struct DepthCameraTask {
    pub pipeline: Receiver<(Device, ActivePipeline)>,
    pub serial: &'static str,
    pub state: OnceCell<DepthCameraState>,
    pub thalassic_ref: ThalassicPipelineRef,
    pub init_tx: Sender<&'static str>,
    pub depth_enabled: bool,
    pub occupancy_enabled: bool,
    pub camera_node_isometry: AtomicCell<Isometry3<f64>>,
}

impl DepthCameraTask {
    pub fn run(&mut self) {
        loop {
            let _ = self.init_tx.send(self.serial);
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

            self.process_camera_session(device, pipeline);
        }
    }

    fn process_camera_session(&mut self, device: Device, mut pipeline: ActivePipeline) {
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

        let DepthCameraState {
            depth_projector,
            point_cloud,
            cloud_publisher,
            ref mut occupancy_pipeline,
            ref mut occupancy_grid,
            occupancy_publisher,
            realsense_isometry_subscriber,
        } = if let Some(state) = self.state.get_mut() {
            state
        } else {
            let focal_length_px;

            if depth_format.fx() != depth_format.fy() {
                eprintln!("Depth camera {} has unequal fx and fy", self.serial);
                focal_length_px = (depth_format.fx() + depth_format.fy()) / 2.0;
            } else {
                focal_length_px = depth_format.fx();
            }

            if !gputter::is_gputter_initialized() {
                if let Err(e) = gputter::init_gputter_blocking() {
                    eprintln!("Failed to initialize gputter GPU system: {}", e);
                    return;
                }
                println!("Initialized gputter GPU system for camera {}", self.serial);
            }

            let depth_projector_builder = DepthProjectorBuilder {
                image_size: Vector2::new(
                    NonZeroU32::new(depth_format.width() as u32).unwrap(),
                    NonZeroU32::new(depth_format.height() as u32).unwrap(),
                ),
                focal_length_px,
                principal_point_px: Vector2::new(depth_format.ppx(), depth_format.ppy()),
                max_depth: 2.5,
                stride: 1,
            };

            let depth_projector = depth_projector_builder.build(self.thalassic_ref.clone());
            let node = create_node();
            let cloud_publisher = create_cloud_publisher(&node, self.serial);
            let iso_subscriber = create_isometry_subscriber(&node);

            let (occupancy_pipeline, occupancy_grid, occupancy_publisher) =
                if self.occupancy_enabled {
                    let grid_dimensions = Vector2::new(
                        NonZeroU32::new(THALASSIC_WIDTH).unwrap(),
                        NonZeroU32::new(THALASSIC_HEIGHT).unwrap(),
                    );
                    let cell_count = grid_dimensions.x.get() * grid_dimensions.y.get();

                    let occupancy_builder = OccupancyGridPipelineBuilder {
                        occupancy_grid_dimensions: grid_dimensions,
                        cell_size: THALASSIC_CELL_SIZE,
                        min_points_for_occupied: 10,
                        neighborhood_radius: NEIGHBORHOOD_RADIUS,
                        min_known_neighbors_ratio: MIN_KNOWN_NEIGHBORS_RATIO,
                        obstacle_threshold: NonZeroU32::new(OBSTACLE_THRESHOLD).unwrap(),
                    };

                    let mut occupancy_pipeline = occupancy_builder.build();

                    occupancy_pipeline.occupancy_grid_ref = self.thalassic_ref.clone();

                    let occupancy_grid = std::iter::repeat(Occupancy(0))
                        .take(cell_count as usize)
                        .collect::<Box<[_]>>();

                    let occupancy_publisher = create_occupancy_publisher(&node, self.serial);

                    (
                        Some(occupancy_pipeline),
                        Some(occupancy_grid),
                        Some(occupancy_publisher),
                    )
                } else {
                    (None, None, None)
                };

            let _ = self.state.set(DepthCameraState {
                point_cloud: std::iter::repeat_n(
                    AlignedVec4::from(Vector4::default()),
                    depth_projector.get_pixel_count().get() as usize,
                )
                .collect(),
                depth_projector,
                cloud_publisher,
                occupancy_pipeline,
                occupancy_grid,
                occupancy_publisher,
                realsense_isometry_subscriber: iso_subscriber,
            });
            self.state.get_mut().unwrap()
        };

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
                if !self.depth_enabled && !self.occupancy_enabled {
                    break;
                }

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

                let depth_scale = match frame.depth_units() {
                    Ok(x) => x,
                    Err(_e) => {
                        eprintln!(
                            "Failed to get depth scale from RealSense Camera {}: {_e}",
                            self.serial
                        );
                        continue;
                    }
                };
                match realsense_isometry_subscriber.receive() {
                    Ok(Some(sample)) => {
                        self.camera_node_isometry.store(
                            EncodableIsometry {
                                inner: *(sample.deref()),
                            }
                            .to_na()
                            .unwrap_or_else(|| {
                                eprintln!(
                                    "failed to convert encodable isometry to nalgebra isometry3"
                                );
                                Isometry3::identity()
                            }),
                        );
                    }
                    Ok(None) => {
                        println!("no sample available");
                    }
                    Err(e) => {
                        eprintln!("failed to receive camera isometry: {e}");
                    }
                }
                let identity_transform: AlignedMatrix4<f32> = self
                    .camera_node_isometry
                    .load()
                    .to_homogeneous()
                    .cast::<f32>()
                    .into();

                depth_projector.project(slice, &identity_transform, depth_scale, Some(point_cloud));
                if self.depth_enabled {
                    let valid_points: Vec<_> = point_cloud
                        .iter()
                        .filter_map(|&point| {
                            if point.w != 0.0 {
                                Some(PointXYZIR {
                                    x: point.x,
                                    y: point.y,
                                    z: point.z,
                                    intensity: 1.0,
                                    time: 0.5,
                                    ring: 0,
                                })
                            } else {
                                None
                            }
                        })
                        .collect();

                    let chunks: Vec<_> = valid_points.chunks(MAX_POINT_CLOUD_POINTS).collect();

                    for (chunk_idx, chunk) in chunks.into_iter().enumerate() {
                        let mut iceoryx_cloud = IceoryxPointCloud::default();

                        for (i, &point) in chunk.iter().enumerate() {
                            iceoryx_cloud.points[i] = point;
                        }

                        iceoryx_cloud.publish_count = chunk.len() as u64;
                        iceoryx_cloud.is_last = true;

                        match cloud_publisher.loan_uninit() {
                            Ok(sample) => {
                                let initialized = sample.write_payload(iceoryx_cloud);
                                match initialized.send() {
                                    Ok(_) => {
                                        // println!(
                                        //     "Published {} points (chunk {}/{}) from camera {} {}",
                                        //     chunk.len(),
                                        //     chunk_idx + 1,
                                        //     total_chunks,
                                        //     self.serial,
                                        //     if is_final { "[FINAL]" } else { "" }
                                        // );
                                    }
                                    Err(_e) => {
                                        eprintln!(
                                            "Failed to send point cloud chunk {} from camera {}",
                                            chunk_idx + 1,
                                            self.serial
                                        );
                                    }
                                }
                            }
                            Err(_e) => {
                                eprintln!(
                                    "Failed to loan sample for camera {} chunk {}",
                                    self.serial,
                                    chunk_idx + 1
                                );
                            }
                        }
                    }
                }

                if self.occupancy_enabled {
                    if let (Some(ref mut pipeline), Some(ref mut grid), Some(ref publisher)) = (
                        occupancy_pipeline.as_mut(),
                        occupancy_grid.as_mut(),
                        occupancy_publisher.as_ref(),
                    ) {
                        if pipeline.will_process() {
                            pipeline.process(
                                ROBOT_RADIUS,
                                THALASSIC_CELL_SIZE,
                                grid,
                                &self
                                    .camera_node_isometry
                                    .load()
                                    .to_homogeneous()
                                    .cast::<f32>()
                                    .into(),
                            );

                            let mut iceoryx_occupancy = IceoryxOccupancyGrid::default();
                            iceoryx_occupancy.width = THALASSIC_WIDTH;
                            iceoryx_occupancy.height = THALASSIC_HEIGHT;

                            let copy_size = grid.len().min(iceoryx_occupancy.data.len());
                            for i in 0..copy_size {
                                iceoryx_occupancy.data[i] = grid[i].0;
                            }

                            match publisher.loan_uninit() {
                                Ok(sample) => {
                                    let initialized = sample.write_payload(iceoryx_occupancy);
                                    match initialized.send() {
                                        Ok(_) => {
                                            println!(
                                                "Published occupancy grid from camera {}",
                                                self.serial
                                            );
                                        }
                                        Err(_e) => {
                                            eprintln!(
                                                "Failed to send occupancy grid from camera {}",
                                                self.serial
                                            );
                                        }
                                    }
                                }
                                Err(_e) => {
                                    eprintln!(
                                        "Failed to loan occupancy sample for camera {}",
                                        self.serial
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }

        eprintln!("RealSense Camera {} closed", self.serial);
    }
}
