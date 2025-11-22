use std::num::NonZeroU32;
use std::thread;
use std::time::Duration;

use bincode::Encode;
use common::{THALASSIC_CELL_COUNT, THALASSIC_CELL_SIZE, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use cu29::cutask::Freezable;
use cu29::prelude::*;

use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use nalgebra::{Vector2, Vector4};
use pcl::cubecl::wgpu::{WgpuDevice, WgpuRuntime};
use pcl::utils::get_and_init_cubecl;
use pcl::{cubecl, launch_depth_to_pcl};
use rerun::{Color, Points3D};
use simple_motion::StaticNode;

use crate::ROOT_NODE;
use crate::rerun_viz::RECORDER;
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH};

pub struct OccupancyGridTask {
    camera_node: StaticNode,
    ppx: f32,
    ppy: f32,
    device: WgpuDevice,
    focal_len_px: f32,
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

        let focal_len_px = config
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
        let device = get_and_init_cubecl();
        Ok(Self {
            camera_node,
            ppx,
            ppy,
            focal_len_px,
            device,
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
        let depths = depth_frame
            .depths
            .iter()
            .map(|d| *d as u32)
            .collect::<Vec<u32>>();
        let depth_scale = depth_frame.depth_scale;
        let (fx, fy) = depth_frame.focal_len;
        let points = launch_depth_to_pcl::<WgpuRuntime>(
            &depths,
            DEPTH_FRAME_WIDTH,
            DEPTH_FRAME_HEIGHT,
            self.ppx,
            self.ppy,
            self.focal_len_px,
            2.5,
            depth_scale,
            self.camera_node.get_global_isometry().cast(),
            8,
            &self.device,
        );
        let height_map = pcl::pcl_to_height::launch_pcl_to_height::<WgpuRuntime>(
            10.0,
            -10.0,
            10.0,
            -10.0,
            0.03,
            8,
            &points,
            &self.device,
        )
        .unwrap();

        if let Some(logger) = RECORDER.get() {
            let _ = logger.recorder.log(
                format!("realsense/pcl"),
                &Points3D::new(points.iter().enumerate().filter_map(|(i, p)| {
                    // if p.w != 0.0 && i % 10 == 0 {
                    Some([p.x, p.y, p.z])
                    // } else {
                    // None
                    // }
                })),
            );
            // use the magic numbers passed to the launch pcl to height for now instead of the thallassic constants
            let _ = logger.recorder.log(
                "heightmap",
                &Points3D::new(
                    height_map
                        .iter()
                        .enumerate()
                        .filter_map(|(i, h)| {
                            if *h > 0.0 {
                                let (x, y) = index_to_xy(i);
                                Some([
                                    x as f32 * 0.03 - 10.0 + 0.015,
                                    y as f32 * 0.03 - 10.0 + 0.015,
                                    *h as f32,
                                ])
                            } else {
                                None
                            }
                        })
                        .collect::<Vec<[f32; 3]>>(),
                ),
            );
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
fn bilateral_filter(
    frame: &mut IceoryxDepthFrame<DEPTH_FRAME_SIZE>,
    radius: i32,
    sigma_spatial: f32,
    sigma_range: f32,
) {
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
