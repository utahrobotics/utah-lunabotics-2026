/// Summary:
/// Converts z16 depth images to a 2d occupancy grid/obstacle map using a series of compute shaders.
///
/// Steps:
///
/// 1. Uses known camera extrinsics and intrinsics to convert a depth image to a point cloud.
/// 2. Creates a height map by othographically projecting point cloud onto a grid of height map cells, where the value of each cell will be equal to the maximum z value ofthe points projected onto the cell.
/// 2. Removes statistical outliers from the newly created height map.
/// 3. Uses either gaussian or bilateral filtering to reduce noise in the map.
/// 4. Computes the avg gradient between k neighbors in the height map, disregarding cells with too many unknown neighbors.
/// 5. Marks gradients over a certain value as obstacles.
///
/// Notes:
///
/// The height map is smaller than the size of the arena to save memory, so the shaders operate on a local map around the robot,
/// and the local map is registered into the global map periodically, then cleared.
use cu29::cutask::Freezable;
use cu29::prelude::*;
use nalgebra::Isometry3;
use rayon::{ThreadPool, ThreadPoolBuilder};

use std::sync::{Arc, Mutex, OnceLock, RwLock};
use wgsl_pcl::pipelines::depth_to_obstacle::ClearAffectedCellsOptions;
use wgsl_pcl::pipelines::filters::*;

use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use rerun::{ImageFormat, Points2D, Points3D};
use simple_motion::StaticNode;
use wgsl_pcl::DepthToPclAndHeightPipeline;
use wgsl_pcl::gpu_types::AlignedMatrix4;
use wgsl_pcl::map_layout::MapLayout;
use wgsl_pcl::wgsl_setup::{get_device, init_gpu_blocking, is_gpu_initialized};

use serde::Deserialize;

use crate::ROBOT_STATE;
use crate::pathfinding::OccupancyGrid;
use crate::rerun_viz::RECORDER;
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH};

pub static GLOBAL_MAP: OnceLock<Arc<RwLock<OccupancyGrid>>> = OnceLock::new();
const PERMANENT_GRADIENT: f32 = 10.0;

#[allow(unused)]
struct ProcessRequest {
    layout: MapLayout,
    depths: Vec<u16>,
    depth_scale: f32,
    transform: AlignedMatrix4<f32>,
    origin: (f32, f32),
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
    max_linear_velocity: f64,
    max_angular_velocity: f64,
    max_acceleration: f64,
    _min_grad_for_obstacle: f32,
    artemis_obstacles_enabled: bool,
    ucf_obstacles_enabled: bool,
}

/// Arena obstacle configuration
#[derive(Deserialize, Debug)]
struct ArenaObstacles {
    walls: Vec<(f32, f32, f32, f32)>, // (min_x, min_y, max_x, max_y)
    pillars: Vec<(f32, f32, f32)>,    // (center_x, center_y, radius)
    boulders: Vec<(f32, f32, f32)>,   // (center_x, center_y, radius)
    craters: Vec<(f32, f32, f32)>,    // (center_x, center_y, radius)
}



/// Paint known arena obstacles permanently into the occupancy grid.
/// robot_radius is added to all obstacle extents to account for robot size.
fn paint_permanent_obstacles(
    grid: &mut OccupancyGrid,
    obstacles: &ArenaObstacles,
    robot_radius: f32,
) {
    
    // Paint rectangular walls (expand by robot_radius on all sides)
    for &(min_x, min_y, max_x, max_y) in &obstacles.walls {
        let expanded_min_x = min_x - robot_radius;
        let expanded_min_y = min_y - robot_radius;
        let expanded_max_x = max_x + robot_radius;
        let expanded_max_y = max_y + robot_radius;

        if let (Ok((x1, y1)), Ok((x2, y2))) = (
            grid.world_to_cell(expanded_min_x, expanded_min_y),
            grid.world_to_cell(expanded_max_x, expanded_max_y),
        ) {
            for x in x1..=x2 {
                for y in y1..=y2 {
                    let _ = grid.set_gradient_at(x, y, PERMANENT_GRADIENT);
                }
            }
        }
    }

    // Paint rectangular pillar obstacle (expand by robot_radius on all sides) 
    for&(cx, cy, w, h) in &obstacles.pillars_rect {
        let half_w = w / 2.0 + robot_radius;
        let half_h = h / 2.0 + robot_radius;
        let min_x = cx - half_w;
        let max_x = cx + half_w;
        let min_y = cy - half_h;
        let max_y = cy + half_h;

        if let (Ok((x1, y1)), Ok((x2, y2))) = (
            grid.world_to_cell(min_x, min_y),
            grid.world_to_cell(max_x, max_y),
        ) {
            for x in x1..=x2 {
                for y in y1..=y2 {
                    let _ = grid.set_gradient_at(x, y, PERMANENT_GRADIENT);
                }
            }
    }

    // Paint circular obstacles (pillars, boulders, craters), expanded by robot_radius
    let all_circles = obstacles
        .pillars
        .iter()
        .chain(obstacles.boulders.iter())
        .chain(obstacles.craters.iter());

    for &(cx, cy, radius) in all_circles {
        if let Ok((center_x, center_y)) = grid.world_to_cell(cx, cy) {
            let expanded_radius = radius + robot_radius;
            let radius_cells = (expanded_radius / grid.layout.cell_size).ceil() as isize;

            for dx in -radius_cells..=radius_cells {
                for dy in -radius_cells..=radius_cells {
                    if dx * dx + dy * dy <= radius_cells * radius_cells {
                        let x = (center_x as isize + dx).max(0) as usize;
                        let y = (center_y as isize + dy).max(0) as usize;
                        let _ = grid.set_gradient_at(x, y, PERMANENT_GRADIENT);
                    }
                }
            }
        }
    }
}

impl Freezable for OccupancyGridTask {}

impl CuTask for OccupancyGridTask {
    type Input<'m> = input_msg!((Option<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>, Option<ImuMsg>));
    type Output<'m> = output_msg!(OccupancyGrid);
    type Resources<'r> = ();

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

            // Log permanent obstacles to Rerun 
            if let Some(global_map) = GLOBAL_MAP.get() {
                if let Ok(grid) = global_map.read() {
                    let mut points = vec![];
                    let mut colors = vec![];
                    for cell_y in 0..grid.cells_y() {
                        for cell_x in 0..grid.cells_x() {
                            let idx = cell_x + cell_y * grid.cells_x();
                            if idx < grid.gradient_map.len() {
                                let gradient = grid.gradient_map[idx];
                                if gradient > PERMANENT_GRADIENT / 2.0 {
                                    // Only show permanent obstacles - obstacles should have gradient of 10. 
                                    if let Ok((world_x, world_y)) =
                                        grid.cell_to_world(cell_x, cell_y)
                                    {
                                        points.push([world_x, world_y]);
                                        colors.push([255, 0, 255]); // Bright magenta 
                                    }
                                }
                            }
                        }
                    }
                    let colors_len = colors.len(); //appease the borrowchecker
                    if !points.is_empty() {
                        logger
                            .recorder
                            .log_static(
                                "realsense/permanent_obstacles",
                                &Points2D::new(points).with_colors(colors),
                            )
                            .unwrap();
                    }
                }
            }
        }

        Ok(())
    }

    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        if !is_gpu_initialized() {
            init_gpu_blocking().map_err(|e| {
                CuError::new_with_cause("failed to init gpu", std::io::Error::other(e))
            })?;
        }
        let camera_name = config
            .and_then(|c| {
                c.get::<String>("camera_node")
                    .expect("failed to deserialize")
            })
            .unwrap_or_else(|| "upper_depth_camera".to_string());

        let max_linear_velocity = config
            .and_then(|c| {
                c.get::<f64>("max_linear_velocity")
                    .expect("failed to deserialize")
            })
            .expect("specify max speed");

        let max_acceleration = config
            .and_then(|c| {
                c.get::<f64>("max_acceleration")
                    .expect("failed to deserialize")
            })
            .expect("specify max accel");

        let max_angular_velocity = config
            .and_then(|c| {
                c.get::<f64>("max_angular_velocity")
                    .expect("failed to deserialize")
            })
            .expect("specify max speed");

        let focal_length_px = config
            .and_then(|c| c.get::<f64>("focal_length").expect("failed to deserialize"))
            .expect("specify focal length") as f32;

        let ppx = config
            .and_then(|c| c.get::<f64>("ppx").expect("failed to deserialize"))
            .expect("specify depth format ppx") as f32;
        let ppy = config
            .and_then(|c| c.get::<f64>("ppy").expect("failed to deserialize"))
            .expect("specify depth format ppy") as f32;

        let max_x = config
            .and_then(|c| {
                c.get::<f64>("heightmap_max_x")
                    .expect("failed to deserialize")
            })
            .unwrap_or(5.0) as f32;
        let max_y = config
            .and_then(|c| {
                c.get::<f64>("heightmap_max_y")
                    .expect("failed to deserialize")
            })
            .unwrap_or(5.0) as f32;
        let min_x = config
            .and_then(|c| {
                c.get::<f64>("heightmap_min_x")
                    .expect("failed to deserialize")
            })
            .unwrap_or(-5.0) as f32;
        let min_y = config
            .and_then(|c| {
                c.get::<f64>("heightmap_min_y")
                    .expect("failed to deserialize")
            })
            .unwrap_or(-5.0) as f32;

        let cell_size = config
            .and_then(|c| {
                c.get::<f64>("heightmap_cell_size")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.1) as f32;

        let bilateral_filter_kernel_radius = config
            .and_then(|c| {
                c.get::<u64>("bilateral_filter_kernel_radius")
                    .expect("failed to deserialize")
            })
            .unwrap_or(5) as u32;
        let bilateral_filter_sigma_spatial = config
            .and_then(|c| {
                c.get::<f64>("bilateral_filter_sigma_spatial")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.7) as f32;
        let bilateral_filter_sigma_range = config
            .and_then(|c| {
                c.get::<f64>("bilateral_filter_sigma_range")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.1) as f32;
        let outlier_filter_kernel_radius = config
            .and_then(|c| {
                c.get::<u64>("outlier_filter_kernel_radius")
                    .expect("failed to deserialize")
            })
            .unwrap_or(2) as u32;
        let outlier_filter_std_dev_threshold = config
            .and_then(|c| {
                c.get::<f64>("outlier_filter_std_dev_threshold")
                    .expect("failed to deserialize")
            })
            .unwrap_or(2.0) as f32;

        let outlier_filter_max_unknown_neighbors_ratio = config
            .and_then(|c| {
                c.get::<f64>("outlier_filter_max_unknown_neighbors_ratio ")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.5) as f32;

        let gradient_filter_kernel_radius = config
            .and_then(|c| {
                c.get::<u64>("gradient_filter_kernel_radius")
                    .expect("failed to deserialize")
            })
            .unwrap_or(2) as u32;

        let gaussian_blur_kernel_radius = config
            .and_then(|c| {
                c.get::<u64>("gaussian_blur_kernel_radius")
                    .expect("failed to deserialize")
            })
            .unwrap_or(5) as u32;

        let gaussian_sigma_spatial = config
            .and_then(|c| {
                c.get::<f64>("gaussian_sigma_spatial")
                    .expect("failed to deserialize")
            })
            .unwrap_or(6.0) as f32;

        let min_depth = config
            .and_then(|c| c.get::<f64>("min_depth").expect("failed to deserialize"))
            .unwrap_or(0.3) as f32;

        let max_depth = config
            .and_then(|c| c.get::<f64>("max_depth").expect("failed to deserialize"))
            .unwrap_or(3.0) as f32;

        let artemis_obstacles_enabled = config
            .and_then(|c| {
                c.get::<bool>("artemis_obstacles_enabled")
                    .expect("failed to deserialize")
            })
            .unwrap_or(true);
        let ucf_obstacles_enabled = config
            .and_then(|c| {
                c.get::<bool>("ucf_obstacles_enabled")
                    .expect("failed to deserialize")
            })
            .unwrap_or(false);

        // use bilateral by default, fall back on gaussian
        let use_bilateral = config
            .and_then(|c| {
                c.get::<bool>("use_bilateral_filter")
                    .expect("failed to deserialize")
            })
            .unwrap_or(true);
        let clear_affected_cells_enabled = config
            .and_then(|c| {
                c.get::<bool>("clear_affected_cells")
                    .expect("failed to deserialize")
            })
            .unwrap_or(true);
        let min_distance_to_clear = config
            .and_then(|c| {
                c.get::<f64>("min_distance_to_clear")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.8) as f32;
        let clear_affected_cells = if clear_affected_cells_enabled {
            Some(ClearAffectedCellsOptions {
                min_distance_to_clear,
            })
        } else {
            None
        };

        let max_distance_traveled_before_reset = config
            .and_then(|c| {
                c.get::<f64>("max_distance_traveled_before_reset")
                    .expect("failed to deserialize")
            })
            .unwrap_or(2.0);
        let max_radians_rotated_before_reset = config
            .and_then(|c| {
                c.get::<f64>("max_radians_rotated_before_reset")
                    .expect("failed to deserialize")
            })
            .unwrap_or(std::f64::consts::PI / 4.0);
        let robot_radius_meters = config
            .and_then(|c| {
                c.get::<f64>("robot_radius_meters")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.4) as f32;

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
                gradient_filter_kernel_radius,
                min_depth,
                clear_affected_cells,
            )
            .map_err(|e| {
                CuError::new_with_cause("failed to create depth to pcl and height pipeline", e)
            })?,
        ));

        let thread_pool = Arc::new(
            ThreadPoolBuilder::new()
                .num_threads(1)
                .build()
                .map_err(|e| CuError::new_with_cause("failed to create thread pool", e))?,
        );
        GLOBAL_MAP.get_or_init(|| {
            let mut grid = OccupancyGrid {
                layout: global_layout.clone(),
                gradient_map: vec![f32::MIN; global_layout.cells_x() * global_layout.cells_y()],
                origin: (0.0, 0.0),
            };

            // Load and paint arena obstacles
            if let Some(obstacles) = load_arena_obstacles(artemis_obstacles_enabled, ucf_obstacles_enabled) {
                paint_permanent_obstacles(&mut grid, &obstacles, robot_radius_meters);

            } else {
                eprintln!("[OccupancyGrid] No arena obstacles loaded");
            }

            Arc::new(RwLock::new(grid))
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
            max_angular_velocity,
            max_linear_velocity,
            max_acceleration,
            _min_grad_for_obstacle: 0.0,
            artemis_obstacles_enabled,
            ucf_obstacles_enabled,
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

        if let Some(grid) = output_buf.take()
            && let Some(global) = GLOBAL_MAP.get()
            && let Ok(mut write_guard) = global.try_write()
        // avoid deadlock if global map is being used elsewhere, this means the global map may be slightly behind at times but only for a short time
        {
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

                grid.append_to(&mut *write_guard).map_err(|e| {
                    CuError::new_with_cause(
                        "failed to append local map to global",
                        std::io::Error::other(e),
                    )
                })?;

                if let Some(logger) = RECORDER.get() {
                    let mut global_obstacle_map_points = vec![];
                    let mut global_obstacle_map_colors = vec![];
                    for cell_y in 0..write_guard.cells_y() {
                        for cell_x in 0..write_guard.cells_x() {
                            let idx = cell_x + cell_y * write_guard.cells_x();

                            if idx < write_guard.gradient_map.len() {
                                let gradient = write_guard.gradient_map[idx];

                                if gradient != f32::MIN {
                                    if let Ok((world_x, world_y)) =
                                        write_guard.cell_to_world(cell_x, cell_y)
                                    {
                                        global_obstacle_map_points.push([world_x, world_y]);

                                        let normalized = ((gradient + 1.0) / 4.0).clamp(0.0, 1.0);
                                        global_obstacle_map_colors.push([
                                            (normalized * 255.0) as u8,
                                            50,
                                            ((1.0 - normalized) * 255.0) as u8,
                                        ]);
                                    }
                                }
                            }
                        }
                    }
                    let _ = logger.recorder.log(
                        "obstacle_mapper/global_obstacle_map",
                        &Points2D::new(global_obstacle_map_points)
                            .with_colors(global_obstacle_map_colors),
                    );
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

        if let Some(state) = ROBOT_STATE.get()
            && let Some(linear_vel) = state.get_velocity()
            && let Some(angular_vel) = state.get_angular_velocity()
            && let Some(accel) = state.get_acceleration()
        {
            if accel.magnitude() > self.max_acceleration {
                eprintln!("Accel violation");
            }
            if linear_vel.magnitude() > self.max_linear_velocity
                || angular_vel.magnitude() > self.max_angular_velocity
                || accel.magnitude() > self.max_acceleration
            {
                eprintln!("Pausing obstacle mapper from speed limit violation");
                return Err(CuError::new_with_cause(
                    "max speed exceeded",
                    std::io::Error::other("max speed exceeded"),
                ));
            }
        }

        // Mark as processing and spawn the work
        *processing = true;

        let mut iso = self.camera_node.get_global_isometry();

        iso.translation.vector -= self.rolling_map_start_position.translation.vector;
        let request = ProcessRequest {
            layout: self.local_layout,
            depths: depth_frame.depths.to_vec(),
            depth_scale: depth_frame.depth_scale,
            transform: iso.to_homogeneous().cast::<f32>().into(),
            origin: (
                self.rolling_map_start_position.translation.vector.x as f32,
                self.rolling_map_start_position.translation.vector.y as f32,
            ),
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
                            request.origin,
                            logger,
                        );
                    }

                    let grid = OccupancyGrid {
                        layout,
                        gradient_map: obstacle_map,
                        origin: request.origin,
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

fn load_arena_obstacles(artemis_obstacles_enabled: bool, ucf_obstacles_enabled: bool) -> Option<ArenaObstacles> {
    if artemis_obstacles_enabled {
        let path = "arena_obstacles/artemis.ron";
        let contents = std::fs::read_to_string(path).ok()?;
        ron::de::from_str(&contents).ok()
    } else if ucf_obstacles_enabled {
        let path = "arena_obstacles/ucf.ron";
        let contents = std::fs::read_to_string(path).ok()?;
        ron::de::from_str(&contents).ok()
    } else {
        eprintln!("No arena obstacles enabled, skipping loading");
        None
    }
}

impl OccupancyGridTask {

    /// also logs out the global map
    /// only needs self for access to the global layout
    fn append_local_to_global(
        &mut self,
        local: &OccupancyGrid,
        global_map: &mut OccupancyGrid,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        for x in 0..local.cells_x() {
            for y in 0..local.cells_y() {
                // cell_to_world now returns world coordinates (already includes local map's origin)
                let Ok(world_coords) = local.cell_to_world(x, y) else {
                    continue;
                };
                let Ok(Some(gradient)) = local.gradient_at(x, y) else {
                    continue;
                };

                if !self
                    .global_layout
                    .is_in_bounds(world_coords.0, world_coords.1)
                {
                    continue;
                }
                let Ok((gx, gy)) = global_map.world_to_cell(world_coords.0, world_coords.1) else {
                    continue;
                };
                global_map.set_gradient_at(gx, gy, gradient)?;
            }
        }

        if let Some(logger) = RECORDER.get() {
            let mut global_obstacle_map_points = vec![];
            let mut global_obstacle_map_colors = vec![];
            for cell_y in 0..global_map.cells_y() {
                for cell_x in 0..global_map.cells_x() {
                    let idx = cell_x + cell_y * global_map.cells_x();

                    if idx < global_map.gradient_map.len() {
                        let gradient = global_map.gradient_map[idx];

                        if gradient != f32::MIN {
                            if let Ok((world_x, world_y)) = global_map.cell_to_world(cell_x, cell_y)
                            {
                                global_obstacle_map_points.push([world_x, world_y]);

                                let normalized = ((gradient + 1.0) / 4.0).clamp(0.0, 1.0);
                                global_obstacle_map_colors.push([
                                    (normalized * 255.0) as u8,
                                    50,
                                    ((1.0 - normalized) * 255.0) as u8,
                                ]);
                            }
                        }
                    }
                }
            }
            let _ = logger.recorder.log(
                "obstacle_mapper/global_obstacle_map",
                &Points2D::new(global_obstacle_map_points).with_colors(global_obstacle_map_colors),
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
    origin: (f32, f32),
    logger: &crate::rerun_viz::RecorderData,
) {
    let cells_x = layout.cells_x();
    let cells_y = layout.cells_y();

    let mut local_obstacle_points = Vec::new();
    let mut local_obstacle_colors = Vec::new();
    for cell_y in 0..cells_y {
        for cell_x in 0..cells_x {
            let idx = cell_x + cell_y * cells_x;

            if idx < local_obstacle_map.len() {
                let gradient = local_obstacle_map[idx];

                if gradient != f32::MIN {
                    let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size + origin.0;
                    let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size + origin.1;

                    local_obstacle_points.push([x, y]);

                    let normalized = ((gradient + 1.0) / 4.0).clamp(0.0, 1.0);
                    local_obstacle_colors.push([
                        (normalized * 255.0) as u8,
                        50,
                        ((1.0 - normalized) * 255.0) as u8,
                    ]);
                }
            }
        }
    }

    let _ = logger.recorder.log(
        "obstacle_mapper/local_obstacle_map",
        &Points2D::new(local_obstacle_points).with_colors(local_obstacle_colors),
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
                let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size + origin.0;
                let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size + origin.1;
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
                let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size + origin.0;
                let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size + origin.1;
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
                let x = layout.min_x + (cell_x as f32 + 0.5) * layout.cell_size + origin.0;
                let y = layout.min_y + (cell_y as f32 + 0.5) * layout.cell_size + origin.1;
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
}
