use bincode::Encode;
use cu29::cutask::Freezable;
use cu29::prelude::*;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::sync::{Arc, Mutex};
use wgsl_pcl::pipelines::depth_to_obstacle::{ClearAffectedCellsOptions, ObstacleExpanderOptions};
use wgsl_pcl::pipelines::filters::*;

use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use rerun::Points3D;
use simple_motion::StaticNode;
use wgsl_pcl::DepthToPclAndHeightPipeline;
use wgsl_pcl::gpu_types::AlignedMatrix4;
use wgsl_pcl::map_layout::MapLayout;
use wgsl_pcl::wgsl_setup::{get_device, init_gpu_blocking, is_gpu_initialized};

use crate::ROBOT_STATE;
use crate::rerun_viz::RECORDER;
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH};

struct ProcessRequest {
    depths: Vec<u16>,
    depth_scale: f32,
    transform: AlignedMatrix4<f32>,
}

pub struct OccupancyGridTask {
    camera_node: StaticNode,
    depth_projector_pipeline: Arc<Mutex<DepthToPclAndHeightPipeline>>,
    layout: MapLayout,
    output_buffer: Arc<Mutex<Option<OccupancyGrid>>>,
    /// technically an atomic would be enough but I dont trust my knowledge of atomic ordering enough to dare use them
    processing: Arc<Mutex<bool>>,
    thread_pool: Arc<ThreadPool>,
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
            (self.layout.max_x + self.layout.min_x) / 2.0,
            (self.layout.max_y + self.layout.min_y) / 2.0,
            0.0,
        ];
        let half_size = [
            (self.layout.max_x - self.layout.min_x) / 2.0,
            (self.layout.max_y - self.layout.min_y) / 2.0,
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
            .and_then(|c| c.get::<u64>("bilateral_filter_sigma_spatial"))
            .unwrap_or(6) as u32;
        let bilateral_filter_sigma_range = config
            .and_then(|c| c.get::<f64>("bilateral_filter_sigma_range"))
            .unwrap_or(0.1) as f32;
        let outlier_filter_kernel_radius = config
            .and_then(|c| c.get::<u64>("outlier_filter_kernel_radius"))
            .unwrap_or(2) as u32;
        let outlier_filter_std_dev_threshold = config
            .and_then(|c| c.get::<f64>("outlier_filter_std_dev_threshold"))
            .unwrap_or(2.0) as f32;

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
                min_distance_to_clear: min_distance_to_clear,
            })
        } else {
            None
        };

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
        let layout = MapLayout::new(max_x, min_x, max_y, min_y, cell_size);

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
            obstacle_gradient_threshold: obstacle_gradient_threshold,
        };
        let outlier_filter_options = OutlierFilterOptions {
            kernel_radius: outlier_filter_kernel_radius,
            std_dev_threshold: outlier_filter_std_dev_threshold,
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
                layout,
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

        Ok(Self {
            camera_node,
            depth_projector_pipeline: pipeline,
            layout,
            output_buffer: Arc::new(Mutex::new(None)),
            processing: Arc::new(Mutex::new(false)),
            thread_pool,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        // First, check if we have a result ready and grab it
        {
            let mut output_buf = self.output_buffer.lock().unwrap();
            if let Some(grid) = output_buf.take() {
                output.set_payload(grid);
            } else {
                output.clear_payload();
            }
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

        let request = ProcessRequest {
            depths: depth_frame.depths.to_vec(),
            depth_scale: depth_frame.depth_scale,
            transform: self
                .camera_node
                .get_global_isometry()
                .to_homogeneous()
                .cast::<f32>()
                .into(),
        };

        let pipeline = Arc::clone(&self.depth_projector_pipeline);
        let output_buffer = Arc::clone(&self.output_buffer);
        let processing_flag = Arc::clone(&self.processing);
        let layout = self.layout.clone();

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
                Ok((point_cloud, height_map)) => {
                    if let Some(logger) = RECORDER.get() {
                        let _ = logger.recorder.log(
                            "realsense/pcl",
                            &Points3D::new(point_cloud.iter().map(|p| [p.x, p.y, p.z])),
                        );

                        let mut heightmap_points = Vec::new();
                        let mut heightmap_colors = Vec::new();

                        let width = layout.max_x - layout.min_x;
                        let height = layout.max_y - layout.min_y;
                        let cells_x = (width / layout.cell_size).ceil() as usize;
                        let cells_y = (height / layout.cell_size).ceil() as usize;

                        for cell_y in 0..cells_y {
                            for cell_x in 0..cells_x {
                                let idx = cell_x + cell_y * cells_x;

                                if idx < height_map.len() {
                                    let z = height_map[idx];
                                    if z == f32::MIN {
                                        continue;
                                    }

                                    let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size;
                                    let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size;

                                    heightmap_points.push([x, y, z]);

                                    let normalized = ((z + 1.0) / 4.0).clamp(0.0, 1.0);
                                    let color = [
                                        (normalized * 255.0) as u8,
                                        50,
                                        ((1.0 - normalized) * 255.0) as u8,
                                    ];
                                    heightmap_colors.push(color);
                                }
                            }
                        }

                        let _ = logger.recorder.log(
                            "realsense/height_map",
                            &Points3D::new(heightmap_points).with_colors(heightmap_colors),
                        );
                    }

                    let grid = OccupancyGrid {
                        max_x: layout.max_x,
                        min_x: layout.min_x,
                        max_y: layout.max_y,
                        min_y: layout.min_y,
                        cell_size: layout.cell_size,
                        gradient_map: height_map,
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
