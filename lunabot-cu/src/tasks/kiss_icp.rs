use std::time::Duration;

use bincode::{Decode, Encode};
use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    CuResult,
    config::ComponentConfig,
    cutask::{CuMsg, CuTask, Freezable},
    input_msg, output_msg,
    prelude::*,
};

use iceoryx_types::IceoryxPointCloud;
use nalgebra::{Isometry3, SMatrix, SVector};
use simple_icp::{config::Config, icp_pipeline::IcpPipeline};

use crate::{ROBOT_STATE, rerun_viz::RECORDER};

pub struct KissIcp {
    pipeline: simple_icp::icp_pipeline::IcpPipeline,
    pub accumulated_frames: Vec<simple_icp::point3d::Point3d>,
    pub frame_accumulation_index: usize,
    pub max_accumulation: usize,
    icp_variance: [f64; 36],
    pub timestamped_poses: Vec<(std::time::Instant, Isometry3<f64>)>,
    pub last_pose_collection_time: std::time::Instant,
    pub time_between_pose_collections: Duration,
    pub min_reflectivity: f32,
}

#[derive(Clone, Copy, Debug, Encode, Decode, Serialize)]
#[repr(C)]
pub struct IcpMeasurement {
    pub pose: EncodableIsometry,
    #[serde(serialize_with = "<[_]>::serialize")]
    pub variance: [f64; 36],
}

impl Default for IcpMeasurement {
    fn default() -> Self {
        Self {
            pose: EncodableIsometry::default(),
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

        let min_reflectivity = config
            .and_then(|c| c.get::<f64>("min_reflectivity"))
            .unwrap_or(150.) as f32;
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

        let max_point_age = config.and_then(|c| c.get::<f64>("max_point_age_seconds"));

        let max_angle_between_poses = config
            .and_then(|c| c.get::<f64>("max_angle_between_poses"))
            .unwrap_or(0.10472); // 6 degrees in radians

        let max_distance_between_poses = config
            .and_then(|c| c.get::<f64>("max_distance_between_poses"))
            .unwrap_or(0.05);

        // time betwene poses collected for deskewing
        let time_between_pose_collections = config
            .and_then(|c| c.get::<u32>("time_between_pose_collections_ms"))
            .unwrap_or(2);

        let position_variance = config
            .and_then(|c| c.get::<f64>("position_variance"))
            .unwrap_or(0.10472);

        let orientation_variance = config
            .and_then(|c| c.get::<f64>("orientation_variance"))
            .unwrap_or(0.001);

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
            max_point_age_seconds: max_point_age,
            max_angle_between_poses,
            max_distance_between_poses,
        };
        let pipeline = IcpPipeline::new_with_config(config);

        let diagonal = SVector::<f64, 6>::new(
            position_variance,
            position_variance,
            position_variance,
            orientation_variance,
            orientation_variance,
            orientation_variance,
        );
        let variance: [f64; 36] = SMatrix::<f64, 6, 6>::from_diagonal(&diagonal)
            .as_slice()
            .try_into()
            .expect("Variance matrix in [kiss_icp.rs] was not 6x6");

        Ok(Self {
            pipeline,
            max_accumulation,
            accumulated_frames: Vec::new(),
            frame_accumulation_index: 0,
            icp_variance: variance,
            timestamped_poses: Vec::new(),
            last_pose_collection_time: std::time::Instant::now(),
            time_between_pose_collections: Duration::from_millis(
                time_between_pose_collections as u64,
            ),
            min_reflectivity,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(recorder) = RECORDER.get() {
            let _ = recorder.recorder.log_static(
                format!("kiss_icp/local/xyz"),
                &rerun::Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                    .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                    .with_labels(vec!["x", "y", "z"]),
            );
        }

        Ok(())
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(state) = ROBOT_STATE.get() {
            let current_time = std::time::Instant::now();
            if current_time.duration_since(self.last_pose_collection_time)
                >= self.time_between_pose_collections
            {
                let pose = state.kinematic_root.get_global_isometry();
                self.timestamped_poses.push((current_time, pose));
                self.last_pose_collection_time = current_time;
            }
        }
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

                let icp_point = simple_icp::point3d::Point3d {
                    x,
                    y,
                    z,
                    intensity,
                    global_timestamp: std::time::Instant::now(),
                };
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
            self.pipeline.process_frame(
                &mut self.accumulated_frames,
                self.min_reflectivity,
                &self.timestamped_poses,
            );

            if let Some(recorder) = RECORDER.get() {
                let _ = recorder.recorder.log(
                    format!("kiss_icp/local"),
                    &rerun::Transform3D::from_translation_rotation(
                        self.pipeline.t_origin_current.translation.vector.data.0[0],
                        rerun::Quaternion::from_xyzw(
                            self.pipeline
                                .t_origin_current
                                .cast::<f32>()
                                .rotation
                                .coords
                                .data
                                .0[0],
                        ),
                    ),
                );
            }

            let map_points = self.pipeline.get_last_batch_points();
            let relative_position = self.pipeline.t_origin_current;

            output.set_payload(IcpMeasurement {
                pose: EncodableIsometry::from_na(&relative_position),
                variance: self.icp_variance,
            });

            self.log_accumulated_map(map_points)?;

            self.accumulated_frames.clear();
            self.timestamped_poses.clear();
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
