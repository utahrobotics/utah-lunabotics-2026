use bincode::Encode;
use cu29::cutask::Freezable;
use cu29::prelude::*;
use nalgebra::{Isometry3, UnitQuaternion};
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::f64::consts::PI;
use std::fmt::Debug;
use std::sync::{Arc, Mutex, OnceLock, RwLock};
use wgsl_pcl::pipelines::depth_to_obstacle::{ClearAffectedCellsOptions, ObstacleExpanderOptions};
use wgsl_pcl::pipelines::filters::*;

use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use rerun::{ImageFormat, Points3D};
use simple_motion::StaticNode;
use wgsl_pcl::DepthToPclAndHeightPipeline;
use wgsl_pcl::gpu_types::AlignedMatrix4;
use wgsl_pcl::map_layout::MapLayout;
use wgsl_pcl::wgsl_setup::{get_device, init_gpu_blocking, is_gpu_initialized};

use crate::ROBOT_STATE;
use crate::rerun_viz::RECORDER;
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH};

pub static GLOBAL_MAP: OnceLock<Arc<RwLock<OccupancyGrid>>> = OnceLock::new();

#[allow(unused)]
struct ProcessRequest {
    layout: MapLayout,
    depths: Vec<u16>,
    depth_scale: f32,
    transform: AlignedMatrix4<f32>,
}

pub struct OccupancyGridTask {
    camera_node: StaticNode,
    depth_projector_pipeline: Arc<Mutex<DepthToPclAndHeightPipeline>>,
    global_layout: MapLayout,
    local_layout: MapLayout,
    output_buffer: Arc<Mutex<Option<OccupancyGrid>>>,
    /// technically an atomic would be enough but I dont trust my knowledge of atomic ordering enough to dare use them
    processing: Arc<Mutex<bool>>,
    thread_pool: Arc<ThreadPool>,
    rolling_map_start_position: Isometry3<f64>,
    max_distance_traveled_before_reset: f64,
    max_radians_rotated_before_reset: f64,
    _min_grad_for_obstacle: f32,
}

#[derive(Serialize, Encode, bincode::Decode, Clone, Debug)]
pub struct OccupancyGrid {
    pub max_x: f32,
    pub min_x: f32,
    pub max_y: f32,
    pub min_y: f32,
    pub cell_size: f32,
    pub gradient_map: Vec<f32>,
}

impl OccupancyGrid {
    pub fn set_gradient_at(
        &mut self,
        cell_x: usize,
        cell_y: usize,
        value: f32,
    ) -> Result<(), String> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        if cell_x >= cells_x || cell_y >= cells_y {
            return Err("Cell coordinates out of bounds".to_string());
        }
        let index = cell_x + cell_y * cells_x;
        if index >= self.gradient_map.len() {
            return Err("Index out of bounds".to_string());
        }
        self.gradient_map[index] = value;
        Ok(())
    }

    pub fn cells_x(&self) -> usize {
        ((self.max_x - self.min_x) / self.cell_size).ceil() as usize
    }

    pub fn cells_y(&self) -> usize {
        ((self.max_y - self.min_y) / self.cell_size).ceil() as usize
    }

    /// Get gradient value at cell coordinates
    pub fn gradient_at(&self, cell_x: usize, cell_y: usize) -> Option<f32> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        if cell_x >= cells_x || cell_y >= cells_y {
            return None;
        }
        let index = cell_x + cell_y * cells_x;
        self.gradient_map
            .get(index)
            .copied()
            .filter(|&val| val != f32::MIN)
    }

    /// Get gradient value at world coordinates
    pub fn gradient_closest_to(&self, x: f32, y: f32) -> Option<f32> {
        let (cell_x, cell_y) = self.world_to_cell(x, y)?;
        self.gradient_at(cell_x, cell_y)
    }

    /// returns average gradient around cell
    /// returns None if central cell is invalid
    pub fn gradient_around_cell(
        &self,
        cell_x: usize,
        cell_y: usize,
        kernel_size: usize,
    ) -> Option<f32> {
        if self.gradient_at(cell_x, cell_y).is_none() {
            return None;
        }
        let mut gradients = Vec::new();
        let half_kernel = kernel_size as isize / 2;
        for i in -half_kernel..=half_kernel {
            for j in -half_kernel..=half_kernel {
                let nx = cell_x as isize + i;
                let ny = cell_y as isize + j;
                if nx < 0 || ny < 0 {
                    continue;
                }
                if let Some(grad) = self.gradient_at(nx as usize, ny as usize) {
                    gradients.push(grad);
                }
            }
        }
        if gradients.is_empty() {
            None
        } else {
            Some(gradients.iter().sum::<f32>() / gradients.len() as f32)
        }
    }

    /// returns average gradient around world coordinate
    /// returns None if central cell is invalid
    pub fn gradient_around(&self, x: f32, y: f32, kernel_size: usize) -> Option<f32> {
        let (cell_x, cell_y) = self.world_to_cell(x, y)?;
        if self.gradient_at(cell_x, cell_y).is_none() {
            return None;
        }
        let mut gradients = Vec::new();
        let half_kernel = kernel_size as isize / 2;
        for i in -half_kernel..=half_kernel {
            for j in -half_kernel..=half_kernel {
                let nx = cell_x as isize + i;
                let ny = cell_y as isize + j;
                if nx < 0 || ny < 0 {
                    continue;
                }
                if let Some(grad) = self.gradient_at(nx as usize, ny as usize) {
                    gradients.push(grad);
                }
            }
        }
        if gradients.is_empty() {
            None
        } else {
            Some(gradients.iter().sum::<f32>() / gradients.len() as f32)
        }
    }

    /// Convert world coordinates to cell indices
    /// cell 0,0 is at (min_x, min_y)
    pub fn world_to_cell(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        if x < self.min_x || x >= self.max_x || y < self.min_y || y >= self.max_y {
            return None;
        }
        let cell_x = ((x - self.min_x) / self.cell_size).floor() as usize;
        let cell_y = ((y - self.min_y) / self.cell_size).floor() as usize;
        Some((cell_x, cell_y))
    }

    /// Convert cell indices to world coordinates (returns cell center)
    pub fn cell_to_world(&self, cell_x: usize, cell_y: usize) -> Option<(f32, f32)> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        if cell_x >= cells_x || cell_y >= cells_y {
            return None;
        }
        let x = self.min_x + (cell_x as f32 + 0.5) * self.cell_size;
        let y = self.min_y + (cell_y as f32 + 0.5) * self.cell_size;
        Some((x, y))
    }
}

impl Default for OccupancyGrid {
    fn default() -> Self {
        OccupancyGrid {
            max_x: 0.0,
            min_x: 0.0,
            max_y: 0.0,
            min_y: 0.0,
            cell_size: 0.0,
            gradient_map: Vec::new(),
        }
    }
}

impl Freezable for OccupancyGridTask {}

impl CuTask for OccupancyGridTask {
    type Input<'m> = input_msg!((Option<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>, Option<ImuMsg>));
    type Output<'m> = output_msg!(OccupancyGrid);

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let center = [
            (self.global_layout.max_x + self.global_layout.min_x) / 2.0,
            (self.global_layout.max_y + self.global_layout.min_y) / 2.0,
            0.0,
        ];
        let half_size = [
            (self.global_layout.max_x - self.global_layout.min_x) / 2.0,
            (self.global_layout.max_y - self.global_layout.min_y) / 2.0,
            0.1,
        ];
        if let Some(logger) = RECORDER.get() {
            logger
                .recorder
                .log_static(
                    "arena",
                    &rerun::Boxes3D::from_centers_and_half_sizes(vec![center], vec![half_size]),
                )
                .unwrap();
        }

        Ok(())
    }

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        if !is_gpu_initialized() {
            init_gpu_blocking().map_err(|e| CuError::new_with_cause("failed to init gpu", &*e))?;
        }
        let camera_name = config
            .and_then(|c| c.get::<String>("camera_node"))
            .unwrap_or_else(|| "upper_depth_camera".to_string());

        let focal_length_px = config
            .and_then(|c| c.get::<f64>("focal_length"))
            .expect("specify focal length") as f32;

        let ppx = config
            .and_then(|c| c.get::<f64>("ppx"))
            .expect("specify depth format ppx") as f32;
        let ppy = config
            .and_then(|c| c.get::<f64>("ppy"))
            .expect("specify depth format ppy") as f32;

        let max_x = config
            .and_then(|c| c.get::<f64>("heightmap_max_x"))
            .unwrap_or(5.0) as f32;
        let max_y = config
            .and_then(|c| c.get::<f64>("heightmap_max_y"))
            .unwrap_or(5.0) as f32;
        let min_x = config
            .and_then(|c| c.get::<f64>("heightmap_min_x"))
            .unwrap_or(-5.0) as f32;
        let min_y = config
            .and_then(|c| c.get::<f64>("heightmap_min_y"))
            .unwrap_or(-5.0) as f32;

        let cell_size = config
            .and_then(|c| c.get::<f64>("heightmap_cell_size"))
            .unwrap_or(0.1) as f32;

        let bilateral_filter_kernel_radius = config
            .and_then(|c| c.get::<u64>("bilateral_filter_kernel_radius"))
            .unwrap_or(5) as u32;
        let bilateral_filter_sigma_spatial = config
            .and_then(|c| c.get::<f64>("bilateral_filter_sigma_spatial"))
            .unwrap_or(0.7) as f32;
        let bilateral_filter_sigma_range = config
            .and_then(|c| c.get::<f64>("bilateral_filter_sigma_range"))
            .unwrap_or(0.1) as f32;
        let outlier_filter_kernel_radius = config
            .and_then(|c| c.get::<u64>("outlier_filter_kernel_radius"))
            .unwrap_or(2) as u32;
        let outlier_filter_std_dev_threshold = config
            .and_then(|c| c.get::<f64>("outlier_filter_std_dev_threshold"))
            .unwrap_or(2.0) as f32;

        let outlier_filter_max_unknown_neighbors_ratio = config
            .and_then(|c| c.get::<f64>("outlier_filter_max_unknown_neighbors_ratio "))
            .unwrap_or(0.5) as f32;

        let gradient_filter_kernel_radius = config
            .and_then(|c| c.get::<u64>("gradient_filter_kernel_radius"))
            .unwrap_or(2) as u32;

        let gaussian_blur_kernel_radius = config
            .and_then(|c| c.get::<u64>("gaussian_blur_kernel_radius"))
            .unwrap_or(5) as u32;

        let gaussian_sigma_spatial = config
            .and_then(|c| c.get::<f64>("gaussian_sigma_spatial"))
            .unwrap_or(6.0) as f32;

        let min_depth = config
            .and_then(|c| c.get::<f64>("min_depth"))
            .unwrap_or(0.3) as f32;

        let max_depth = config
            .and_then(|c| c.get::<f64>("max_depth"))
            .unwrap_or(3.0) as f32;

        let robot_radius_meters = config
            .and_then(|c| c.get::<f64>("robot_radius_meters"))
            .unwrap_or(0.3) as f32;

        let obstacle_gradient_threshold = config
            .and_then(|c| c.get::<f64>("obstacle_gradient_threshold"))
            .unwrap_or(0.2) as f32;

        // use bilateral by default, fall back on gaussian
        let use_bilateral = config
            .and_then(|c| c.get::<bool>("use_bilateral_filter"))
            .unwrap_or(true);
        let clear_affected_cells_enabled = config
            .and_then(|c| c.get::<bool>("clear_affected_cells"))
            .unwrap_or(true);
        let min_distance_to_clear = config
            .and_then(|c| c.get::<f64>("min_distance_to_clear"))
            .unwrap_or(0.8) as f32;
        let clear_affected_cells = if clear_affected_cells_enabled {
            Some(ClearAffectedCellsOptions {
                min_distance_to_clear,
            })
        } else {
            None
        };

        let max_distance_traveled_before_reset = config
            .and_then(|c| c.get::<f64>("max_distance_traveled_before_reset"))
            .unwrap_or(2.0);
        let max_radians_rotated_before_reset = config
            .and_then(|c| c.get::<f64>("max_radians_rotated_before_reset"))
            .unwrap_or(std::f64::consts::PI / 4.0);

        let camera_node = ROBOT_STATE
            .get()
            .ok_or_else(|| {
                CuError::from("RealSensePointCloudReceiver: ROBOT_STATE not initialized")
            })?
            .kinematic_root
            .get_node_with_name(&camera_name)
            .ok_or_else(|| {
                CuError::from(format!(
                    "RealSensePointCloudReceiver: camera node '{}' not found",
                    camera_name
                ))
            })?
            .clone();
        let global_layout = MapLayout::new(max_x, min_x, max_y, min_y, cell_size);
        let distance_buffer = max_depth + max_distance_traveled_before_reset as f32;
        let local_layout = MapLayout::new(
            distance_buffer,
            -distance_buffer,
            distance_buffer,
            -distance_buffer,
            cell_size,
        );
        let blur_filter_options = if use_bilateral {
            BlurFilterOptions::Bilateral(BilateralOptions {
                kernel_radius: bilateral_filter_kernel_radius,
                sigma_spatial: bilateral_filter_sigma_spatial,
                sigma_range: bilateral_filter_sigma_range,
            })
        } else {
            BlurFilterOptions::Gaussian(GaussianOptions {
                kernel_radius: gaussian_blur_kernel_radius,
                sigma: gaussian_sigma_spatial,
            })
        };

        let obstacle_expander_options = ObstacleExpanderOptions {
            expansion_radius_meters: robot_radius_meters,
            obstacle_gradient_threshold,
        };
        let outlier_filter_options = OutlierFilterOptions {
            kernel_radius: outlier_filter_kernel_radius,
            std_dev_threshold: outlier_filter_std_dev_threshold,
            max_unknown_neighbors_ratio: outlier_filter_max_unknown_neighbors_ratio,
        };

        let pipeline = Arc::new(Mutex::new(
            DepthToPclAndHeightPipeline::new(
                max_depth,
                (DEPTH_FRAME_WIDTH as u32, DEPTH_FRAME_HEIGHT as u32),
                focal_length_px,
                (ppx, ppy),
                get_device(),
                (8, 8),
                (16, 16),
                local_layout.clone(),
                blur_filter_options,
                outlier_filter_options,
                obstacle_expander_options,
                gradient_filter_kernel_radius,
                min_depth,
                clear_affected_cells,
            )
            .map_err(|e| {
                CuError::new_with_cause("failed to create depth to pcl and height pipeline", &e)
            })?,
        ));

        let thread_pool = Arc::new(
            ThreadPoolBuilder::new()
                .num_threads(1)
                .build()
                .map_err(|e| CuError::new_with_cause("failed to create thread pool", e))?,
        );
        GLOBAL_MAP.get_or_init(|| {
            Arc::new(RwLock::new(OccupancyGrid {
                max_x,
                min_x,
                max_y,
                min_y,
                cell_size,
                gradient_map: vec![f32::MIN; global_layout.cells_x() * global_layout.cells_y()],
            }))
        });

        Ok(Self {
            camera_node,
            depth_projector_pipeline: pipeline,
            local_layout,
            global_layout,
            output_buffer: Arc::new(Mutex::new(None)),
            processing: Arc::new(Mutex::new(false)),
            thread_pool,
            max_distance_traveled_before_reset,
            max_radians_rotated_before_reset,
            rolling_map_start_position: camera_node.get_global_isometry(),
            _min_grad_for_obstacle: obstacle_gradient_threshold,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        // First, check if we have a result ready and grab it

        let mut output_buf = self.output_buffer.lock().unwrap();

        if let Some(grid) = output_buf.take() {
            let camera_isometry = self.camera_node.get_global_isometry();
            let relative_rotation =
                camera_isometry.rotation.inverse() * self.rolling_map_start_position.rotation;
            if (camera_isometry.translation.vector
                - self.rolling_map_start_position.translation.vector)
                .norm()
                > self.max_distance_traveled_before_reset
                || relative_rotation.angle() > self.max_radians_rotated_before_reset
            {
                drop(output_buf); // appease the borrow checker by dropping immutable borrow to self
                let start = self.rolling_map_start_position.cast::<f32>();
                if let Some(global) = GLOBAL_MAP.get() {
                    let mut write_guard = global.write().map_err(|e| CuError::new_with_cause("failed to get global map write guard", e))?;
                    self.append_local_to_global(&grid, start, &mut *write_guard).map_err(|e| {
                        CuError::new_with_cause("failed to append local map to global", e)
                    })?;
                }
                self.rolling_map_start_position = camera_isometry;
                let mut pipeline_guard = self.depth_projector_pipeline.lock().unwrap();
                pipeline_guard.clear_map(get_device());
            }
            output.set_payload(grid);
        } else {
            output.clear_payload();
        }

        // are we currently processing?
        let mut processing = self.processing.lock().unwrap();
        if *processing {
            return Ok(());
        }

        // Get input data
        let Some(input_msg) = input.payload() else {
            return Ok(());
        };

        let Some(ref depth_frame) = input_msg.0 else {
            return Ok(());
        };

        // Mark as processing and spawn the work
        *processing = true;

        let mut iso = self.camera_node.get_global_isometry();

        iso.translation.vector -= self.rolling_map_start_position.translation.vector;
        let request = ProcessRequest {
            layout: self.local_layout,
            depths: depth_frame.depths.to_vec(),
            depth_scale: depth_frame.depth_scale,
            transform: iso.to_homogeneous().cast::<f32>().into(),
        };

        let pipeline = Arc::clone(&self.depth_projector_pipeline);
        let output_buffer = Arc::clone(&self.output_buffer);
        let processing_flag = Arc::clone(&self.processing);
        let layout = self.local_layout.clone();

        self.thread_pool.spawn_fifo(move || {
            let result = {
                let mut pipeline_guard = pipeline.lock().unwrap();
                pipeline_guard.process(
                    &request.depths,
                    get_device(),
                    request.transform,
                    request.depth_scale,
                )
            };

            match result {
                Ok((point_cloud, obstacle_map)) => {
                    if let Some(logger) = RECORDER.get() {
                        // Log the raw depth image
                        let depth_bytes: &[u8] = unsafe {
                            std::slice::from_raw_parts(
                                request.depths.as_ptr() as *const u8,
                                request.depths.len() * std::mem::size_of::<u16>(),
                            )
                        };
                        let _ = logger.recorder.log(
                            "realsense/depth_image",
                            &rerun::DepthImage::new(
                                depth_bytes,
                                ImageFormat::depth(
                                    [DEPTH_FRAME_WIDTH as u32, DEPTH_FRAME_HEIGHT as u32],
                                    rerun::ChannelDatatype::U16,
                                ),
                            )
                            .with_meter(1.0 / request.depth_scale)
                            .with_depth_range([0.0, 2.0 / request.depth_scale as f64]),
                        );
                        let _ = logger.recorder.log(
                            "realsense/pcl",
                            &Points3D::new(point_cloud.iter().map(|p| [p.x, p.y, p.z])),
                        );

                        let pipeline_guard = pipeline.lock().unwrap();

                        // same dimensions as height map
                        let raw_height_map =
                            pipeline_guard.get_raw_height_map(get_device()).unwrap();
                        let raw_gradient_map =
                            pipeline_guard.get_gradient_map(get_device()).unwrap();
                        let blur_filtered_height_map = pipeline_guard
                            .get_blur_filtered_height_map(get_device())
                            .unwrap();

                        log_map(
                            layout,
                            &obstacle_map,
                            raw_height_map,
                            raw_gradient_map,
                            blur_filtered_height_map,
                            logger,
                        );
                    }

                    let grid = OccupancyGrid {
                        max_x: layout.max_x,
                        min_x: layout.min_x,
                        max_y: layout.max_y,
                        min_y: layout.min_y,
                        cell_size: layout.cell_size,
                        gradient_map: obstacle_map,
                    };

                    let mut output_buf = output_buffer.lock().unwrap();
                    *output_buf = Some(grid);
                }
                Err(e) => {
                    eprintln!("GPU processing error: {:?}", e);
                }
            }

            *processing_flag.lock().unwrap() = false;
        });
        Ok(())
    }
}

impl OccupancyGridTask {
    /// also logs out the global map
    /// only needs self for access to the global layout
    fn append_local_to_global(
        &mut self,
        local: &OccupancyGrid,
        origin: Isometry3<f32>,
        global_map: &mut OccupancyGrid,
    ) -> Result<(), Box<dyn std::error::Error>> {
        for x in 0..local.cells_x() {
            for y in 0..local.cells_y() {
                let Some(local_coords) = local.cell_to_world(x, y) else {
                    continue;
                };
                let Some(gradient) = local.gradient_at(x, y) else {
                    continue;
                };
                let global_coords = origin.translation.vector
                    + nalgebra::Vector3::new(local_coords.0, local_coords.1, 0.0);

                if !self
                    .global_layout
                    .is_in_bounds(global_coords.data.0[0][0], global_coords.data.0[0][0])
                {
                    continue;
                }
                let Some((x, y)) = global_map
                    .world_to_cell(global_coords.data.0[0][0], global_coords.data.0[0][1])
                else {
                    continue;
                };
                global_map.set_gradient_at(x, y, gradient)?;
            }
        }

        if let Some(logger) = RECORDER.get() {
            // Log global obstacle map
            let mut global_obstacle_image_data =
                vec![0u8; global_map.cells_x() * global_map.cells_y() * 3];
            for cell_y in 0..global_map.cells_y() {
                for cell_x in 0..global_map.cells_x() {
                    let idx = cell_x + cell_y * global_map.cells_x();

                    if idx < global_map.gradient_map.len() {
                        let z = global_map.gradient_map[idx];
                        let pixel_idx = idx * 3;

                        if z == f32::MIN {
                            // Black for invalid cells
                            global_obstacle_image_data[pixel_idx] = 0;
                            global_obstacle_image_data[pixel_idx + 1] = 0;
                            global_obstacle_image_data[pixel_idx + 2] = 0;
                        } else {
                            let normalized = ((z + 1.0) / 4.0).clamp(0.0, 1.0);
                            global_obstacle_image_data[pixel_idx] = (normalized * 255.0) as u8;
                            global_obstacle_image_data[pixel_idx + 1] = 50;
                            global_obstacle_image_data[pixel_idx + 2] =
                                ((1.0 - normalized) * 255.0) as u8;
                        }
                    }
                }
            }
            let _ = logger.recorder.log(
                "realsense/global_obstacle_map",
                &rerun::Image::new(
                    global_obstacle_image_data,
                    ImageFormat::rgb8([
                        global_map.cells_x() as u32,
                        global_map.cells_y() as u32,
                    ]),
                ),
            );
        }

        Ok(())
    }
}

fn log_map(
    layout: MapLayout,
    local_obstacle_map: &Vec<f32>,
    raw_height_map: Vec<f32>,
    raw_gradient_map: Vec<f32>,
    blur_filtered_height_map: Vec<f32>,
    logger: &crate::rerun_viz::RecorderData,
) {
    let cells_x = layout.cells_x();
    let cells_y = layout.cells_y();

    // Create obstacle map as 2D image (RGB)
    let mut obstacle_image_data = vec![0u8; cells_x * cells_y * 3];
    for cell_y in 0..cells_y {
        for cell_x in 0..cells_x {
            let idx = cell_x + cell_y * cells_x;

            if idx < local_obstacle_map.len() {
                let z = local_obstacle_map[idx];
                let pixel_idx = idx * 3;

                if z == f32::MIN {
                    // Black for invalid cells
                    obstacle_image_data[pixel_idx] = 0;
                    obstacle_image_data[pixel_idx + 1] = 0;
                    obstacle_image_data[pixel_idx + 2] = 0;
                } else {
                    let normalized = ((z + 1.0) / 4.0).clamp(0.0, 1.0);
                    obstacle_image_data[pixel_idx] = (normalized * 255.0) as u8;
                    obstacle_image_data[pixel_idx + 1] = 50;
                    obstacle_image_data[pixel_idx + 2] = ((1.0 - normalized) * 255.0) as u8;
                }
            }
        }
    }

    let _ = logger.recorder.log(
        "realsense/local_obstacle_map",
        &rerun::Image::new(
            obstacle_image_data,
            ImageFormat::rgb8([cells_x as u32, cells_y as u32]),
        ),
    );

    // Log raw height map
    let mut raw_height_points = Vec::new();
    let mut raw_height_colors = Vec::new();
    for cell_y in 0..cells_y {
        for cell_x in 0..cells_x {
            let idx = cell_x + cell_y * cells_x;
            if idx < raw_height_map.len() {
                let z = raw_height_map[idx];
                if z == f32::MIN {
                    continue;
                }
                let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size;
                let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size;
                raw_height_points.push([x, y, z]);
                let normalized = ((z + 1.0) / 4.0).clamp(0.0, 1.0);
                raw_height_colors.push([
                    (normalized * 255.0) as u8,
                    100,
                    ((1.0 - normalized) * 255.0) as u8,
                ]);
            }
        }
    }
    let _ = logger.recorder.log(
        "realsense/raw_height_map",
        &Points3D::new(raw_height_points).with_colors(raw_height_colors),
    );

    // Log gradient map
    let mut gradient_points = Vec::new();
    let mut gradient_colors = Vec::new();
    for cell_y in 0..cells_y {
        for cell_x in 0..cells_x {
            let idx = cell_x + cell_y * cells_x;
            if idx < raw_gradient_map.len() {
                let gradient = raw_gradient_map[idx];
                if gradient == f32::MIN {
                    continue;
                }
                let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size;
                let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size;
                // Use gradient value as z for visualization
                gradient_points.push([x, y, gradient * 2.0]);
                let normalized = (gradient * 2.0).clamp(0.0, 1.0);
                gradient_colors.push([
                    (normalized * 255.0) as u8,
                    0,
                    ((1.0 - normalized) * 255.0) as u8,
                ]);
            }
        }
    }
    let _ = logger.recorder.log(
        "realsense/gradient_map",
        &Points3D::new(gradient_points).with_colors(gradient_colors),
    );

    // Log blur filtered height map
    let mut blur_height_points = Vec::new();
    let mut blur_height_colors = Vec::new();
    for cell_y in 0..cells_y {
        for cell_x in 0..cells_x {
            let idx = cell_x + cell_y * cells_x;
            if idx < blur_filtered_height_map.len() {
                let z = blur_filtered_height_map[idx];
                if z == f32::MIN {
                    continue;
                }
                let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size;
                let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size;
                blur_height_points.push([x, y, z]);
                let normalized = ((z + 1.0) / 4.0).clamp(0.0, 1.0);
                blur_height_colors.push([
                    (normalized * 255.0) as u8,
                    150,
                    ((1.0 - normalized) * 255.0) as u8,
                ]);
            }
        }
    }
    let _ = logger.recorder.log(
        "realsense/blur_filtered_height_map",
        &Points3D::new(blur_height_points).with_colors(blur_height_colors),
    );
}
