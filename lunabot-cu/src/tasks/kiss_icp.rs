use cu_spatial_payloads::{EncodableIsometry, Transform3D};
use cu29::{
    CuResult,
    config::ComponentConfig,
    cutask::{CuMsg, CuTask, Freezable},
    input_msg, output_msg,
    prelude::*,
};

use iceoryx_types::IceoryxPointCloud;
use simple_icp::{config::Config, icp_pipeline::IcpPipeline};

use crate::rerun_viz::{self, RECORDER};
pub struct KissIcp {
    pipeline: simple_icp::icp_pipeline::IcpPipeline,
    pub accumulated_frames: Vec<simple_icp::point3d::Point3d>,
    pub frame_accumulation_index: usize,
    pub max_accumulation: usize,
}

impl Freezable for KissIcp {}

impl CuTask for KissIcp {
    type Input<'m> = input_msg!(IceoryxPointCloud);
    type Output<'m> = output_msg!(EncodableIsometry);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let voxel_size = config
            .and_then(|c| c.get::<f64>("voxel_size"))
            .unwrap_or(0.5) as f32;
        let max_range = config
            .and_then(|c| c.get::<f64>("max_range"))
            .unwrap_or(100.0) as f32;
        let min_range = config
            .and_then(|c| c.get::<f64>("min_range"))
            .unwrap_or(5.0) as f32;
        let max_points_per_voxel = config
            .and_then(|c| c.get::<i32>("max_points_per_voxel"))
            .unwrap_or(20) as u16;
        let initial_threshold = config
            .and_then(|c| c.get::<f64>("initial_threshold"))
            .unwrap_or(2.0);
        let min_motion_th = config
            .and_then(|c| c.get::<f64>("min_motion_th"))
            .unwrap_or(0.1);
        let enable_deskewing = config
            .and_then(|c| c.get::<bool>("enable_deskewing"))
            .unwrap_or(true);
        let max_num_iterations = config
            .and_then(|c| c.get::<i32>("max_num_iteration"))
            .unwrap_or(500) as u16;
        let convergence_criterion = config
            .and_then(|c| c.get::<f64>("convergence_criterion"))
            .unwrap_or(0.0001);
        let max_num_threads = config
            .and_then(|c| c.get::<i32>("max_num_threads"))
            .unwrap_or(4) as u8;

        let max_accumulation = config
            .and_then(|c| c.get::<i32>("max_accumulation"))
            .unwrap_or(2) as usize;

        let config = Config {
            voxel_size,
            max_range,
            min_range,
            max_points_per_voxel,
            min_motion_th,
            initial_threshold,
            max_num_iterations,
            convergence_criterion,
            max_num_threads,
            deskew: enable_deskewing,
        };
        let pipeline = IcpPipeline::new_with_config(config);
        Ok(Self {
            pipeline,
            max_accumulation,
            accumulated_frames: Vec::new(),
            frame_accumulation_index: 0,
        })
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(point_cloud_payload) = input.payload() {
            let mut should_process = false;
            for (_, point) in point_cloud_payload.points
                [..point_cloud_payload.publish_count as usize]
                .iter()
                .enumerate()
            {
                let x = point.x;
                let y = point.y;
                let z = point.z;
                let intensity = point.intensity;
                let icp_point = simple_icp::point3d::Point3d { x, y, z, intensity };
                self.accumulated_frames.push(icp_point);
                if self.frame_accumulation_index == self.max_accumulation {
                    should_process = true;
                }
            }

            if !should_process {
                self.frame_accumulation_index += 1;
                output.clear_payload();
                return Ok(());
            }
            self.pipeline.process_frame(&self.accumulated_frames);
            let map_points = self.pipeline.get_last_batch_points();
            let position = self.pipeline.t_origin_current;
            output.set_payload(EncodableIsometry::from_na(&position));

            self.log_accumulated_map(map_points)?;
            self.accumulated_frames.clear();
            self.frame_accumulation_index = 0;
        } else {
            output.clear_payload();
        }
        Ok(())
    }

    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
pub fn get_colors_for_points(
    points: &[simple_icp::point3d::Point3d],
    min_val: f32,
    max_val: f32,
    alpha: u8,
) -> Vec<(u8, u8, u8, u8)> {
    let g = colorous::TURBO;
    points
        .iter()
        .map(|j| {
            let c = g.eval_continuous(((j.intensity - min_val) / (max_val - min_val)).into());
            (c.r, c.g, c.b, alpha)
        })
        .collect()
}

impl KissIcp {
    fn log_accumulated_map(
        &self,
        voxel_map_batch: &[simple_icp::point3d::Point3d],
    ) -> CuResult<()> {
        let Some(rec) = RECORDER.get() else {
            return Ok(());
        };
        let colors = get_colors_for_points(voxel_map_batch, 0.0, 255.0, 255);
        rec.recorder
            .log(
                "kiss_icp/accumulated_map",
                &rerun::Points3D::new(voxel_map_batch.iter().map(|p| (p.x, p.y, p.z)))
                    .with_radii([0.05])
                    .with_colors(colors),
            )
            .unwrap();
        Ok(())
    }
}
