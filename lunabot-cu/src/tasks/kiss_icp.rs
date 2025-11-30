use bincode::{Decode, Encode};
use cu29::{
    CuResult,
    config::ComponentConfig,
    cutask::{CuMsg, CuTask, Freezable},
    input_msg, output_msg,
    prelude::*,
};

use iceoryx_types::IceoryxPointCloud;
use kalman_filter::SimpleVector;
use nalgebra::{Isometry, Quaternion, Unit};
use simple_icp::{config::Config, icp_pipeline::IcpPipeline};

use crate::{ROBOT_STATE, rerun_viz::RECORDER};

const REFERENCE_VARIANCE_THRESHOLD: f64 = 1.0;

pub struct KissIcp {
    pipeline: simple_icp::icp_pipeline::IcpPipeline,
    pub accumulated_frames: Vec<simple_icp::point3d::Point3d>,
    pub frame_accumulation_index: usize,
    pub max_accumulation: usize,
    icp_variance: [f64; 36],
    reference_offset: Option<Isometry<f64, Unit<Quaternion<f64>>, 3>>,
}

#[derive(Clone, Copy, Debug, Encode, Decode, Serialize)]
#[repr(C)]
pub struct IcpMeasurement {
    pub position: [f64; 3],
    pub orientation: [f64; 3],
    #[serde(serialize_with = "<[_]>::serialize")]
    pub variance: [f64; 36],
}

impl Default for IcpMeasurement {
    fn default() -> Self {
        Self {
            position: Default::default(),
            orientation: Default::default(),
            variance: [0.0; 36],
        }
    }
}

impl Freezable for KissIcp {}

impl CuTask for KissIcp {
    type Input<'m> = input_msg!(IceoryxPointCloud);
    type Output<'m> = output_msg!(IcpMeasurement);

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

        let diagonal = SimpleVector::<6>::new(
            0.05 as f64, // Position variance
            0.05 as f64,
            0.05 as f64,
            0.5 as f64, // Orientation variance
            0.5 as f64,
            0.5 as f64,
        );
        let variance: [f64; 36] = kalman_filter::SimpleSquareMatrix::<6>::from_diagonal(&diagonal)
            .as_slice()
            .try_into()
            .expect("Variance matrix in [kiss_icp.rs] was not 6x6");

        Ok(Self {
            pipeline,
            max_accumulation,
            accumulated_frames: Vec::new(),
            frame_accumulation_index: 0,
            icp_variance: variance,
            reference_offset: None,
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
            self.pipeline.process_frame(&self.accumulated_frames, 200.0);

            let map_points = self.pipeline.get_last_batch_points();
            let relative_position = self.pipeline.t_origin_current;

            if let Some(reference_offset) = self.reference_offset {
                let position = reference_offset * relative_position;

                let trans = position.translation;
                let rotat = position
                    .rotation
                    .axis()
                    .and_then(|vec| Some(vec.into_inner()))
                    .unwrap_or(SimpleVector::<3>::zeros())
                    * position.rotation.angle();

                let actual_message = IcpMeasurement {
                    position: [trans.x, trans.y, trans.z],
                    orientation: [rotat.x, rotat.y, rotat.z],
                    variance: self.icp_variance,
                };

                output.set_payload(actual_message);
            } else {
                let position_variance = ROBOT_STATE.get().unwrap().get_position_variance();
                let orientation_variance = ROBOT_STATE.get().unwrap().get_orientation_variance();

                // Get total variance, to determine if we have good values yet.
                let mut total_variance: f64 = 0.0;
                for variance in position_variance.diagonal().as_slice() {
                    total_variance += variance;
                }

                for variance in orientation_variance.diagonal().as_slice() {
                    total_variance += variance;
                }

                if total_variance < REFERENCE_VARIANCE_THRESHOLD {
                    // Think about this being substituted into the uses of reference offset
                    self.reference_offset = Some(
                        ROBOT_STATE
                            .get()
                            .unwrap()
                            .kinematic_root
                            .get_global_isometry()
                            * relative_position.inverse(),
                    );
                }
            }

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
