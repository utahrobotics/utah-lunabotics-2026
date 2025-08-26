use std::{
    cell::OnceCell,
    num::NonZeroU32,
    ops::Deref,
    sync::mpsc::{Receiver, Sender, SyncSender},
    time::Duration,
};

use common::{THALASSIC_CELL_SIZE, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use crossbeam::atomic::AtomicCell;
use cu_spatial_payloads::EncodableIsometry;
use fxhash::FxHashMap;
use gputter::{
    self,
    types::{AlignedMatrix4, AlignedVec4},
};
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use nalgebra::{Isometry3, Matrix4, Vector2, Vector4};
pub use realsense_rust;
use realsense_rust::{
    config::Config,
    device::Device,
    frame::{DepthFrame, PixelKind},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::{ActivePipeline, FrameWaitError, InactivePipeline},
};

use iceoryx_types::{IceoryxOccupancyGrid, IceoryxPointCloud, PointXYZIR, MAX_POINT_CLOUD_POINTS};
use thalassic::{
    DepthProjector, DepthProjectorBuilder, Occupancy, OccupancyGridPipeline,
    OccupancyGridPipelineBuilder, ThalassicPipelineRef,
};

pub struct DepthCameraInfo {
    pub serial: String,
    pub depth_enabled: bool,
    pub occupancy_enabled: bool,
}

struct ThalassicData;

impl Default for ThalassicData {
    fn default() -> Self {
        Self
    }
}

fn spawn_minimal_thalassic_pipeline() -> ThalassicPipelineRef {
    ThalassicPipelineRef::noop()
}

pub fn enumerate_depth_cameras(cameras: impl IntoIterator<Item = DepthCameraInfo>) {
    let thalassic_ref = spawn_minimal_thalassic_pipeline();
    let (init_tx, init_rx) = std::sync::mpsc::channel::<&'static str>();
    let mut threads: FxHashMap<&str, SyncSender<(Device, ActivePipeline)>> = cameras
        .into_iter()
        .filter_map(|camera_info| {
            let serial: &'static str = Box::leak(camera_info.serial.into_boxed_str());
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            let init_tx = init_tx.clone();
            let thalassic_ref = thalassic_ref.clone();

            std::thread::Builder::new()
                .stack_size(16 * 1024 * 1024) // 16 MB stack size
                .spawn(move || {
                    let mut camera_task = DepthCameraTask {
                        pipeline: rx,
                        serial,
                        state: OnceCell::new(),
                        thalassic_ref,
                        init_tx,
                        depth_enabled: camera_info.depth_enabled,
                        occupancy_enabled: camera_info.occupancy_enabled,
                        camera_node_isometry: AtomicCell::new(Isometry3::identity()),
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

                if let Err(e) = config.enable_stream(
                    Rs2StreamKind::Depth,
                    None,           // device index (None = any)
                    640,            // width - explicitly supported
                    480,            // height - explicitly supported
                    Rs2Format::Z16, // format
                    30,             // fps - supported at this resolution
                ) {
                    eprintln!("Failed to enable depth stream: {}", e);
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

struct DepthCameraState {
    depth_projector: DepthProjector,
    point_cloud: Box<[AlignedVec4<f32>]>,
    cloud_publisher: iceoryx2::port::publisher::Publisher<ipc::Service, IceoryxPointCloud, ()>,
    occupancy_pipeline: Option<OccupancyGridPipeline>,
    occupancy_grid: Option<Box<[Occupancy]>>,
    occupancy_publisher:
        Option<iceoryx2::port::publisher::Publisher<ipc::Service, IceoryxOccupancyGrid, ()>>,
    // subscribes to EncodableIsometry
    realsense_isometry_subscriber: Subscriber<ipc::Service, [f64; 16], ()>,
}

struct DepthCameraTask {
    pipeline: Receiver<(Device, ActivePipeline)>,
    serial: &'static str,
    state: OnceCell<DepthCameraState>,
    thalassic_ref: ThalassicPipelineRef,
    init_tx: Sender<&'static str>,
    depth_enabled: bool,
    occupancy_enabled: bool,
    camera_node_isometry: AtomicCell<Isometry3<f64>>,
}

impl DepthCameraTask {
    fn run(&mut self) {
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
                max_depth: 3.0,
            };

            let depth_projector = depth_projector_builder.build(self.thalassic_ref.clone());

            let node = NodeBuilder::new()
                .create::<ipc::Service>()
                .expect("Failed to create iceoryx2 node");

            let cloud_service_name = format!("realsense/{}/cloud", self.serial);
            let cloud_service = node
                .service_builder(
                    &ServiceName::new(&cloud_service_name).expect("Invalid service name"),
                )
                .publish_subscribe::<IceoryxPointCloud>()
                .open_or_create()
                .expect("Failed to create cloud service");

            let cloud_publisher = cloud_service
                .publisher_builder()
                .create()
                .expect("Failed to create cloud publisher");

            let from_service = node
                .service_builder(
                    &ServiceName::new("localizer/realsense_isometry")
                        .expect("invalid service name"),
                )
                .publish_subscribe::<[f64; 16]>()
                .enable_safe_overflow(true)
                .open_or_create()
                .expect("Realsense: failed to open localizer→realsense service");

            let iso_subscriber = from_service
                .subscriber_builder()
                .create()
                .expect("Realsense: failed to create subscriber");

            let (occupancy_pipeline, occupancy_grid, occupancy_publisher) = if self
                .occupancy_enabled
            {
                let grid_dimensions = Vector2::new(
                    NonZeroU32::new(THALASSIC_WIDTH).unwrap(),
                    NonZeroU32::new(THALASSIC_HEIGHT).unwrap(),
                );
                let cell_count = grid_dimensions.x.get() * grid_dimensions.y.get();

                let occupancy_builder = OccupancyGridPipelineBuilder {
                    occupancy_grid_dimensions: grid_dimensions,
                    cell_size: THALASSIC_CELL_SIZE,
                    min_points_for_occupied: 3,
                    max_points_threshold: 50,
                    neighborhood_radius: 2,
                };

                let mut occupancy_pipeline = occupancy_builder.build();

                occupancy_pipeline.occupancy_grid_ref = self.thalassic_ref.clone();

                let occupancy_grid = std::iter::repeat(Occupancy(0))
                    .take(cell_count as usize)
                    .collect::<Box<[_]>>();

                let occupancy_service_name = format!("realsense/{}/occupancy", self.serial);
                let occupancy_service = node
                    .service_builder(
                        &ServiceName::new(&occupancy_service_name).expect("Invalid service name"),
                    )
                    .publish_subscribe::<IceoryxOccupancyGrid>()
                    .open_or_create()
                    .expect("Failed to create occupancy service");

                let occupancy_publisher = occupancy_service
                    .publisher_builder()
                    .create()
                    .expect("Failed to create occupancy publisher");

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
                    let mut iceoryx_cloud = IceoryxPointCloud::default();
                    let mut point_count = 0;

                    for (_idx, &point) in point_cloud.iter().enumerate() {
                        if point.w != 0.0 && point_count < MAX_POINT_CLOUD_POINTS {
                            iceoryx_cloud.points[point_count] = PointXYZIR {
                                x: point.x,
                                y: point.y,
                                z: point.z,
                                intensity: 1.0,
                                time: 0.5,
                                ring: 0,
                            };
                            point_count += 1;
                        }
                    }

                    iceoryx_cloud.publish_count = point_count as u64;

                    if point_count > 0 {
                        match cloud_publisher.loan_uninit() {
                            Ok(sample) => {
                                let initialized = sample.write_payload(iceoryx_cloud);
                                match initialized.send() {
                                    Ok(_) => {
                                        println!(
                                            "Published {} points from camera {}",
                                            point_count, self.serial
                                        );
                                    }
                                    Err(_e) => {
                                        eprintln!(
                                            "Failed to send point cloud from camera {}",
                                            self.serial
                                        );
                                    }
                                }
                            }
                            Err(_e) => {
                                eprintln!("Failed to loan sample for camera {}", self.serial);
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
                            pipeline.process(grid);

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

fn main() {
    println!("Starting RealSense depth camera publisher with occupancy grid");

    std::env::set_var("STRIDE", "20");
    // Configure cameras with both point cloud and occupancy grid enabled
    let cameras = vec![DepthCameraInfo {
        serial: "311322302990".to_string(),
        depth_enabled: true,
        occupancy_enabled: true,
    }];

    enumerate_depth_cameras(cameras);

    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}
