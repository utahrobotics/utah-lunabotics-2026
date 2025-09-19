use std::num::NonZeroU32;

use common::{THALASSIC_CELL_COUNT, THALASSIC_CELL_SIZE, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use cu29::cutask::Freezable;
use cu29::prelude::*;

use gputter::types::{AlignedMatrix4, AlignedVec4};
use iceoryx_types::{IceoryxDepthFrame, IceoryxOccupancyGrid};
use nalgebra::{Vector2, Vector4};
use simple_motion::StaticNode;
use thalassic::{
    DepthProjector, DepthProjectorBuilder, Occupancy, OccupancyGridPipeline,
    OccupancyGridPipelineBuilder, PipelineSharedItems,
};

use crate::ROOT_NODE;
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH};

pub struct OccupancyGridSource {
    camera_node: StaticNode,
    depth_projector_pipeline: DepthProjector,
    occupancy_grid_pipeline: OccupancyGridPipeline,
    robot_radius: f64,
    min_known_neighbors_ratio: u32,
    neighborhood_radius: u32,
    obstacle_threshold: u32,
}

impl Freezable for OccupancyGridSource {}

impl CuTask for OccupancyGridSource {
    type Input<'m> = input_msg!(IceoryxDepthFrame<DEPTH_FRAME_SIZE>);
    type Output<'m> = output_msg!(IceoryxOccupancyGrid);

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let camera_name = config
            .and_then(|c| c.get::<String>("camera_node"))
            .unwrap_or_else(|| "upper_depth_camera".to_string());

        let robot_radius = config
            .and_then(|c| c.get::<f64>("robot_radius"))
            .unwrap_or_else(|| 0.5);

        let min_known_neighbors_ratio = config
            .and_then(|c| c.get::<u32>("min_known_neighbors_ratio"))
            .unwrap_or_else(|| 60);
        let neighborhood_radius = config
            .and_then(|c| c.get::<u32>("neighborhood_radius"))
            .unwrap_or_else(|| 20);
        let obstacle_threshold = config
            .and_then(|c| c.get::<u32>("obstacle_threshold"))
            .unwrap_or_else(|| 60);

        let focal_length_px = config
            .and_then(|c| c.get::<f64>("obstacle_threshold"))
            .expect("specify camera focal length") as f32;

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

        // experimentally find these
        let ppx = todo!();
        let ppy = todo!();

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

        let occupancy_grid_pipeline = OccupancyGridPipelineBuilder {
            occupancy_grid_dimensions: grid_dimensions,
            cell_size: THALASSIC_CELL_SIZE,
            min_points_for_occupied: 10,
            neighborhood_radius,
            min_known_neighbors_ratio,
            obstacle_threshold: NonZeroU32::new(obstacle_threshold).unwrap(),
        }
        .build();

        Ok(Self {
            camera_node,
            depth_projector_pipeline,
            occupancy_grid_pipeline,
            robot_radius,
            min_known_neighbors_ratio,
            neighborhood_radius,
            obstacle_threshold,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let Some(depth_frame) = input.payload() else {
            return Ok(());
        };

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
            &depth_frame.depths,
            &depth_camera_transform,
            depth_frame.depth_scale,
            Some(&mut point_cloud),
        );

        let mut occupancy_grid_out =
            std::iter::repeat_n(Occupancy::UNKNOWN, THALASSIC_CELL_COUNT as usize)
                .collect::<Vec<_>>();

        if self.occupancy_grid_pipeline.will_process() {
            self.occupancy_grid_pipeline.process(
                self.robot_radius as f32,
                THALASSIC_CELL_SIZE,
                &mut occupancy_grid_out,
                &depth_camera_transform,
            );
        }

        todo!()
    }
}
