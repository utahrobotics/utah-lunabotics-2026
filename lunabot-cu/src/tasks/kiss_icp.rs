use cu29::{
    config::ComponentConfig,
    cutask::{CuMsg, CuTask, Freezable},
    input_msg, output_msg,
    prelude::*,
    CuResult,
};
use cu_spatial_payloads::Transform3D;
use rerun::Rgba32;

use iceoryx_types::IceoryxPointCloud;

use crate::rerun_viz;

use kiss_icp_core::preprocessing::{preprocess as kiss_preprocess, voxel_downsample};
use kiss_icp_core::{
    deskew,
    threshold::AdaptiveThreshold,
    types::VoxelPoint,
    voxel_hash_map::{VoxelHashMap, VoxelHashMapArgs},
};
use nalgebra::{DMatrix, Isometry3, MatrixXx3, Vector3};
use rayon::iter::ParallelIterator;
use std::time::{Duration, Instant};

pub struct KissIcp {
    // Core KISS ICP components
    voxel_map: VoxelHashMap,
    adaptive_threshold: AdaptiveThreshold,

    // Configuration parameters
    voxel_size: f64,
    max_range: f64,
    min_range: f64,
    initial_threshold: f64,
    min_motion_th: f64,
    max_points_per_voxel: usize,
    max_iterations: usize,
    convergence_tolerance: f64,
    min_threshold: f64,
    max_threshold: f64,

    // State tracking
    current_pose: Isometry3<f64>,
    previous_pose: Isometry3<f64>,
    scan_start_pose: Isometry3<f64>,
    scan_finish_pose: Isometry3<f64>,
    is_initialized: bool,
    enable_deskewing: bool,
    last_map_log: Instant,
    map_log_interval: Duration,
}

impl Freezable for KissIcp {}

impl CuTask for KissIcp {
    type Input<'m> = input_msg!(IceoryxPointCloud);
    type Output<'m> = output_msg!(Transform3D<f64>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let voxel_size = config
            .and_then(|c| c.get::<f64>("voxel_size"))
            .unwrap_or(0.5);
        let max_range = config
            .and_then(|c| c.get::<f64>("max_range"))
            .unwrap_or(100.0);
        let min_range = config
            .and_then(|c| c.get::<f64>("min_range"))
            .unwrap_or(5.0);
        let initial_threshold = config
            .and_then(|c| c.get::<f64>("initial_threshold"))
            .unwrap_or(2.0);
        let min_motion_th = config
            .and_then(|c| c.get::<f64>("min_motion_th"))
            .unwrap_or(0.1);
        let max_points_per_voxel = config
            .and_then(|c| c.get::<i32>("max_points_per_voxel"))
            .unwrap_or(20) as usize;
        let max_iterations = config
            .and_then(|c| c.get::<i32>("max_iterations"))
            .unwrap_or(500) as usize;
        let convergence_tolerance = config
            .and_then(|c| c.get::<f64>("convergence_tolerance"))
            .unwrap_or(1e-6);
        let min_threshold = config
            .and_then(|c| c.get::<f64>("min_threshold"))
            .unwrap_or(0.05);
        let max_threshold = config
            .and_then(|c| c.get::<f64>("max_threshold"))
            .unwrap_or(5.0);
        let enable_deskewing = config
            .and_then(|c| c.get::<bool>("enable_deskewing"))
            .unwrap_or(true);
        let map_log_interval_secs = config
            .and_then(|c| c.get::<f64>("map_log_interval_secs"))
            .unwrap_or(1.0);

        let args = VoxelHashMapArgs {
            max_distance2: max_range * max_range,
            max_points_per_voxel,
        };

        Ok(Self {
            voxel_map: VoxelHashMap::new(args, voxel_size),
            adaptive_threshold: AdaptiveThreshold::new(initial_threshold, min_motion_th, max_range),
            voxel_size,
            max_range,
            min_range,
            initial_threshold,
            min_motion_th,
            max_points_per_voxel,
            max_iterations,
            convergence_tolerance,
            min_threshold,
            max_threshold,
            current_pose: Isometry3::identity(),
            previous_pose: Isometry3::identity(),
            scan_start_pose: Isometry3::identity(),
            scan_finish_pose: Isometry3::identity(),
            is_initialized: false,
            enable_deskewing,
            last_map_log: Instant::now(),
            map_log_interval: Duration::from_secs_f64(map_log_interval_secs),
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
            let mut raw_points = Vec::new();
            let mut timestamps = Vec::new();

            for (_, point) in point_cloud_payload.points
                [..point_cloud_payload.publish_count as usize]
                .iter()
                .enumerate()
            {
                let x = point.x;
                let y = point.y;
                let z = point.z;

                let voxel_point = Vector3::new(x as f64, y as f64, z as f64);
                raw_points.push(voxel_point);

                timestamps.push(point.time)
            }

            if raw_points.is_empty() {
                output.clear_payload();
                return Ok(());
            }

            let raw_matrix = self.points_to_voxel_matrix(&raw_points);
            let timestamps = point_cloud_payload.points
                [..point_cloud_payload.publish_count as usize]
                .iter()
                .map(|p| p.time as f64)
                .collect::<Vec<f64>>();
            let deskewed_points: Vec<VoxelPoint> = if self.enable_deskewing && self.is_initialized {
                self.scan_start_pose = self.previous_pose;
                self.scan_finish_pose = self.current_pose;

                deskew::scan(
                    &raw_matrix,
                    &timestamps,
                    self.scan_start_pose,
                    self.scan_finish_pose,
                )
                .collect()
            } else {
                (0..raw_matrix.nrows())
                    .map(|i| raw_matrix.row(i).transpose())
                    .collect()
            };

            if deskewed_points.is_empty() {
                output.clear_payload();
                return Ok(());
            }

            let deskewed_matrix = self.points_to_voxel_matrix(&deskewed_points);
            let filtered_points: Vec<VoxelPoint> =
                kiss_preprocess(&deskewed_matrix, self.min_range..self.max_range).collect();

            if filtered_points.is_empty() {
                output.clear_payload();
                return Ok(());
            }

            let filtered_matrix = self.points_to_voxel_matrix(&filtered_points);
            let downsampled_points: Vec<VoxelPoint> =
                voxel_downsample(&filtered_matrix, self.voxel_size).collect();

            if downsampled_points.is_empty() {
                output.clear_payload();
                return Ok(());
            }

            let downsampled_frame = self.points_to_voxel_matrix(&downsampled_points);

            if !self.is_initialized {
                self.voxel_map.add_points(&downsampled_frame);
                self.is_initialized = true;

                let transform = Transform3D::from(Isometry3::identity());
                output.set_payload(transform);
            } else {
                let correspondence_threshold = self.adaptive_threshold.compute_threshold();

                let estimated_pose = self.voxel_map.register_frame(
                    downsampled_frame.clone(),
                    self.current_pose,
                    correspondence_threshold,
                    self.initial_threshold,
                );

                let pose_delta = self.previous_pose.inverse() * estimated_pose;
                self.adaptive_threshold.update_model_deviation(pose_delta);

                self.previous_pose = self.current_pose;
                self.current_pose = estimated_pose;

                self.voxel_map
                    .update_with_pose(&downsampled_frame, estimated_pose);

                let transform = self.isometry_to_transform3d(&estimated_pose);
                output.set_payload(transform);
            }

            self.log_accumulated_map()?;
        } else {
            output.clear_payload();
        }
        Ok(())
    }

    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

impl KissIcp {
    fn points_to_voxel_matrix(&self, points: &[VoxelPoint]) -> MatrixXx3<f64> {
        if points.is_empty() {
            return MatrixXx3::zeros(0);
        }

        let mut matrix = MatrixXx3::zeros(points.len());
        for (i, point) in points.iter().enumerate() {
            matrix.row_mut(i).copy_from(&point.transpose());
        }
        matrix
    }

    fn isometry_to_transform3d(&self, pose: &Isometry3<f64>) -> Transform3D<f64> {
        Transform3D::from(*pose)
    }

    fn log_accumulated_map(&mut self) -> CuResult<()> {
        if self.last_map_log.elapsed() < self.map_log_interval
            || !self.is_initialized
            || self.voxel_map.is_empty()
        {
            return Ok(());
        }

        let Some(recorder_data) = rerun_viz::RECORDER.get() else {
            return Ok(());
        };

        let map_points: Vec<VoxelPoint> = self.voxel_map.get_point_cloud().copied().collect();
        if map_points.is_empty() {
            return Ok(());
        }

        let mut positions = Vec::with_capacity(map_points.len());
        let mut colors = Vec::with_capacity(map_points.len());

        for point in &map_points {
            positions.push([point.x as f32, point.y as f32, point.z as f32]);
            colors.push([100, 100, 255, 100]);
        }

        if let Err(e) = recorder_data.recorder.log(
            "kiss_icp/accumulated_map",
            &rerun::Points3D::new(positions)
                .with_colors(colors)
                .with_radii([0.02f32]),
        ) {
            warning!("Failed to log accumulated map to Rerun: {}", e.to_string());
        } else {
            info!(
                "Logged {} points from accumulated KISS-ICP map",
                map_points.len()
            );
        }

        self.last_map_log = Instant::now();
        Ok(())
    }
}
