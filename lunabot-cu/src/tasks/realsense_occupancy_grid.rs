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

        RECORDER
            .get()
            .unwrap()
            .recorder
            .log_static(
                "arena",
                &rerun::Boxes3D::from_centers_and_half_sizes(vec![center], vec![half_size]),
            )
            .unwrap();
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
            &depth_frame.depths,
            &depth_camera_transform,
            depth_frame.depth_scale,
            Some(&mut point_cloud),
        );

        let mut height_map_out = vec![0u32; THALASSIC_CELL_COUNT as usize];
        let point_count = self.depth_projector_pipeline.get_pixel_count().get();

        if self.height_map_pipeline.will_process() {
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
