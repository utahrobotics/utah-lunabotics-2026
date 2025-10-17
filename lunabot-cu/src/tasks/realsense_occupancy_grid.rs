use std::num::NonZeroU32;

use bincode::Encode;
use common::{THALASSIC_CELL_COUNT, THALASSIC_CELL_SIZE, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use cu29::cutask::Freezable;
use cu29::prelude::*;

use gputter::types::{AlignedMatrix4, AlignedVec4};
use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use nalgebra::{Vector2, Vector4};
use rerun::{Color, Points3D};
use simple_motion::StaticNode;
use thalassic::{
    DepthProjector, DepthProjectorBuilder, HeightMapPipeline, HeightMapPipelineBuilder,
    PipelineSharedItems,
};

use crate::ROOT_NODE;
use crate::rerun_viz::RECORDER;
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH};

pub struct OccupancyGridTask {
    camera_node: StaticNode,
    depth_projector_pipeline: DepthProjector,
    height_map_pipeline: HeightMapPipeline,
}

#[derive(Serialize, Encode, bincode::Decode, Clone, Copy, Debug)]
pub struct OccupancyGrid {
    pub width: u32,
    pub height: u32,
    #[serde(serialize_with = "<[_]>::serialize")]
    pub height_map: [i32; THALASSIC_CELL_COUNT as usize],
}

impl Default for OccupancyGrid {
    fn default() -> Self {
        OccupancyGrid {
            width: 0,
            height: 0,
            height_map: [0; THALASSIC_CELL_COUNT as usize],
        }
    }
}

impl Freezable for OccupancyGridTask {}

impl CuTask for OccupancyGridTask {
    type Input<'m> = input_msg!((Option<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>, Option<ImuMsg>));
    type Output<'m> = output_msg!(OccupancyGrid);

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let width = THALASSIC_WIDTH;
        let height = THALASSIC_HEIGHT;

        let center = (
            (width as f32 * THALASSIC_CELL_SIZE) / 2.0,
            (height as f32 * THALASSIC_CELL_SIZE) / 2.0,
            0.0,
        );
        let half_size = (
            (width as f32 * THALASSIC_CELL_SIZE) / 2.0,
            (height as f32 * THALASSIC_CELL_SIZE) / 2.0,
            0.01,
        );
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

        let gaussian_kernel_size = config
            .and_then(|c| c.get::<u32>("gaussian_kernel_size"))
            .expect("specify kernel size for gaussian blur");

        let camera_node = ROOT_NODE
            .get()
            .ok_or_else(|| CuError::from("RealSensePointCloudReceiver: ROOT_NODE not initialized"))?
            .get_node_with_name(&camera_name)
            .ok_or_else(|| {
                CuError::from(format!(
                    "RealSensePointCloudReceiver: camera node '{}' not found",
                    camera_name
                ))
            })?
            .clone();

        if !gputter::is_gputter_initialized() {
            gputter::init_gputter_blocking().expect("Failed to initialize gputter");
            info!("Initialized gputter GPU system");
        }

        // has image dimentions and points that are shared between pipelines
        let pipeline_shared = PipelineSharedItems::noop();

        let depth_projector_pipeline = DepthProjectorBuilder {
            image_size: Vector2::new(
                NonZeroU32::new(DEPTH_FRAME_WIDTH).unwrap(),
                NonZeroU32::new(DEPTH_FRAME_HEIGHT).unwrap(),
            ),
            focal_length_px,
            principal_point_px: Vector2::new(ppx, ppy),
            max_depth: 2.5,
            stride: 1,
        }
        .build(pipeline_shared.clone());

        let grid_dimensions = Vector2::new(
            NonZeroU32::new(THALASSIC_WIDTH).unwrap(),
            NonZeroU32::new(THALASSIC_HEIGHT).unwrap(),
        );

        let height_map_pipeline = HeightMapPipelineBuilder {
            grid_dimentions: grid_dimensions,
            cell_size: THALASSIC_CELL_SIZE,
            max_point_count: depth_projector_pipeline.get_pixel_count(),
            kernel_size: NonZeroU32::new(gaussian_kernel_size).unwrap(),
            k_neighbors: 8,
            std_dev_thresh: 1.0,
        }
        .build(pipeline_shared.clone());

        Ok(Self {
            camera_node,
            depth_projector_pipeline,
            height_map_pipeline,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let Some(input_msg) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let Some(ref depth_frame) = input_msg.0 else {
            output.clear_payload();
            return Ok(());
        };

        // Apply bilateral filtering to clean up the depth data
        let mut filtered_depth_frame = depth_frame.clone();
        bilateral_filter(&mut filtered_depth_frame, 3, 1.5, 0.05); // radius=3, spatial_sigma=1.5, range_sigma=0.05m

        // TODO: utilize imu messages from the realsense here for more accurate pitch information

        let depth_camera_transform: AlignedMatrix4<f32> = self
            .camera_node
            .get_global_isometry()
            .to_homogeneous()
            .cast::<f32>()
            .into();

        let mut point_cloud: Box<[AlignedVec4<f32>]> = std::iter::repeat_n(
            AlignedVec4::from(Vector4::default()),
            self.depth_projector_pipeline.get_pixel_count().get() as usize,
        )
        .collect::<Vec<_>>()
        .into_boxed_slice();

        self.depth_projector_pipeline.project(
            &filtered_depth_frame.depths,
            &depth_camera_transform,
            filtered_depth_frame.depth_scale,
            Some(&mut point_cloud),
        );

        // self.depth_projector_pipeline.project(
        //     &depth_frame.depths,
        //     &depth_camera_transform,
        //     depth_frame.depth_scale,
        //     Some(&mut point_cloud),
        // );

        // only log every nth point so my laptop doesnt catch on fire
        if let Some(logger) = RECORDER.get() {
            let _ = logger.recorder.log(
                format!("realsense/pcl"),
                &Points3D::new(point_cloud.iter().enumerate().filter_map(|(i, p)| {
                    if p.w != 0.0 && i % 10 == 0 {
                        Some([p.x, p.y, p.z])
                    } else {
                        None
                    }
                })),
            );
        }

        let mut height_map_out = vec![0u32; THALASSIC_CELL_COUNT as usize];
        let point_count = self.depth_projector_pipeline.get_pixel_count().get();

        if self.height_map_pipeline.will_process() {
            self.height_map_pipeline.request_clear_cells();
            self.height_map_pipeline
                .process(point_count, &mut height_map_out);

            let mut positions = Vec::new();
            let mut colors = Vec::new();
            let mut labels = Vec::new();

            for (i, &height_fixed) in height_map_out.iter().enumerate() {
                let (x, y) = index_to_xy(i);
                let height_meters = f32::from_bits(height_fixed);

                positions.push(rerun::Position3D::new(
                    x as f32 * THALASSIC_CELL_SIZE,
                    y as f32 * THALASSIC_CELL_SIZE,
                    height_meters,
                ));

                labels.push(format!("{:.2}m", height_meters));

                let normalized_height = (height_meters + 1.0) / 2.0; // Assuming height range -1m to 1m
                let normalized_height = normalized_height.clamp(0.0, 1.0);
                let r = (255.0 * normalized_height) as u8;
                let b = (255.0 * (1.0 - normalized_height)) as u8;
                colors.push(Color::from_rgb(r, 0, b));
            }

            let _ = RECORDER.get().unwrap().recorder.log(
                "height_map",
                &Points3D::new(positions)
                    .with_colors(colors)
                    .with_labels(labels),
            );

            // currently we don't have a height map -> occupancy grid implementation yet
            // output.set_payload(OccupancyGrid {
            //     width: THALASSIC_WIDTH,
            //     height: THALASSIC_HEIGHT,
            //     height_map: height_map_out.try_into().map_err(|_| {
            //         CuError::new_with_cause(
            //             "height map out was the wrong size",
            //             std::io::Error::other("size mismatch"),
            //         )
            //     })?,
            // });
        }
        Ok(())
    }
}

fn index_to_xy(index: usize) -> (usize, usize) {
    (
        index % THALASSIC_WIDTH as usize,
        index / THALASSIC_WIDTH as usize,
    )
}

// Bilateral filter for depth images
// For each pixel: examine neighbors within radius
// Compute weight using pixel distance - Gaussian spatial weight
// Compute weight using depth difference - Gaussian range weight
// Combine weights to get final weight for each neighbor
// Find a weighted average of neighbor depths
fn bilateral_filter(frame: &mut IceoryxDepthFrame<DEPTH_FRAME_SIZE>, radius: i32, sigma_spatial: f32, sigma_range: f32) {
    let width = DEPTH_FRAME_WIDTH as i32;
    let height = DEPTH_FRAME_HEIGHT as i32;
    
    // temporary copy for reading 
    let mut temp_frame = frame.clone();
    
    // Precompute spatial weights for efficiency
    let mut spatial_weights = vec![0.0f32; (2 * radius + 1) as usize];
    for i in 0..spatial_weights.len() {
        let distance = (i as i32 - radius) as f32;
        spatial_weights[i] = (-distance * distance / (2.0 * sigma_spatial * sigma_spatial)).exp();
    }
    
    // get depth value at position, handling invalid depths (0)
    let get_depth = |depths: &[u16], x: i32, y: i32| -> Option<u16> {
        if x < 0 || x >= width || y < 0 || y >= height {
            return None;
        }
        let idx = (y * width + x) as usize;
        let depth = depths[idx];
        if depth == 0 { None } else { Some(depth) }
    };
    
    // compute range weight
    let range_weight = |center_depth: u16, neighbor_depth: u16| -> f32 {
        let diff = (center_depth as f32 - neighbor_depth as f32) * frame.depth_scale;
        (-diff * diff / (2.0 * sigma_range * sigma_range)).exp()
    };
    
    // Pass 1: Horizontal filtering
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            
            if let Some(center_depth) = get_depth(&temp_frame.depths, x, y) {
                let mut sum_weights = 0.0f32;
                let mut sum_depth = 0.0f32;
                
                for dx in -radius..=radius {
                    if let Some(neighbor_depth) = get_depth(&temp_frame.depths, x + dx, y) {
                        let spatial_w = spatial_weights[(dx + radius) as usize];
                        let range_w = range_weight(center_depth, neighbor_depth);
                        let weight = spatial_w * range_w;
                        
                        sum_weights += weight;
                        sum_depth += neighbor_depth as f32 * weight;
                    }
                }
                
                if sum_weights > 0.0 {
                    frame.depths[idx] = (sum_depth / sum_weights).round() as u16;
                } else {
                    frame.depths[idx] = center_depth;
                }
            }
            // If center depth is invalid (0), leave it as is
        }
    }
    
    // Update temp with horizontal filtered results for vertical pass
    temp_frame = frame.clone();
    
    // Pass 2: Vertical filtering
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            
            if let Some(center_depth) = get_depth(&temp_frame.depths, x, y) {
                let mut sum_weights = 0.0f32;
                let mut sum_depth = 0.0f32;
                
                for dy in -radius..=radius {
                    if let Some(neighbor_depth) = get_depth(&temp_frame.depths, x, y + dy) {

                        let spatial_w = spatial_weights[(dy + radius) as usize];
                        let range_w = range_weight(center_depth, neighbor_depth);
                        let weight = spatial_w * range_w;
                        
                        sum_weights += weight;
                        sum_depth += neighbor_depth as f32 * weight;
                    }
                }
                
                if sum_weights > 0.0 {
                    frame.depths[idx] = (sum_depth / sum_weights).round() as u16;
                } else {
                    frame.depths[idx] = center_depth;
                }
            }
            // If center depth is invalid (0), leave it as is
        }
    }
}
