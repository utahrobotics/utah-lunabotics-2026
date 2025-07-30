use crate::{
    rerun_viz,
    utils::{lerp, lerp_value, swing_twist_decomposition},
    ROOT_NODE,
};
use cu29::{
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuMsg, CuSinkTask, Freezable},
    input_msg,
    prelude::*,
    CuResult,
};
use cu_spatial_payloads::Transform3D;
use iceoryx_types::ImuMsg;
use nalgebra::{Isometry3, UnitQuaternion, UnitVector3, Vector3};
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::collections::HashMap;
use std::time::{Duration, Instant};

// Constants from the old localizer
const ACCELEROMETER_LERP_SPEED: f64 = 150.0;
const LOCALIZATION_DELTA: f64 = 1.0 / 60.0;

pub struct CuLocalizer {
    pub root_node: StaticNode,
    last_rerun_log: Instant,
}

impl Freezable for CuLocalizer {}

impl CuSinkTask for CuLocalizer {
    /// Imu data from l2, isometry from l2 kiss_icp, isometry from realsense kiss_icp, and a HashMap of camera IDs to estimated observer isometry from apriltags
    type Input<'m> = input_msg!('m,
        ImuMsg,
        Transform3D<f64>,
        Transform3D<f64>,
        Box<HashMap<String, Transform3D<f64>>>
    );

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if let Some(root_node) = ROOT_NODE.get() {
            return Ok(Self {
                root_node: root_node.clone(),
                last_rerun_log: Instant::now(),
            });
        } else {
            return Err(CuError::new_with_cause(
                "no root node found",
                std::io::Error::other("no root node found"),
            ));
        }
    }

    fn process(&mut self, clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        let start = clock.now().as_nanos();

        // Get current robot isometry
        let mut isometry = self.root_node.get_global_isometry();

        // Check for NaN/infinite values and return error if found
        if isometry.translation.x.is_nan()
            || isometry.translation.y.is_nan()
            || isometry.translation.z.is_nan()
        {
            return Err(CuError::new_with_cause(
                "Robot origin is NaN",
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Robot origin contains NaN values",
                ),
            ));
        } else if isometry.translation.x.is_infinite()
            || isometry.translation.y.is_infinite()
            || isometry.translation.z.is_infinite()
        {
            return Err(CuError::new_with_cause(
                "Robot origin is infinite",
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Robot origin contains infinite values",
                ),
            ));
        } else if isometry.rotation.w.is_nan()
            || isometry.rotation.i.is_nan()
            || isometry.rotation.j.is_nan()
            || isometry.rotation.k.is_nan()
        {
            return Err(CuError::new_with_cause(
                "Robot rotation is NaN",
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Robot rotation contains NaN values",
                ),
            ));
        } else if isometry.rotation.w.is_infinite()
            || isometry.rotation.i.is_infinite()
            || isometry.rotation.j.is_infinite()
            || isometry.rotation.k.is_infinite()
        {
            return Err(CuError::new_with_cause(
                "Robot rotation is infinite",
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Robot rotation contains infinite values",
                ),
            ));
        }

        // IMU-based orientation adjustment (down-axis alignment and gyro integration)

        let mut down_axis = -Vector3::y_axis();
        let mut angular_velocity_opt: Option<Vector3<f64>> = None;

        // Acquire IMU data if available
        if let Some(imu_msg) = input.0.payload() {
            let acceleration = Vector3::new(
                imu_msg.linear_acceleration[0] as f64,
                imu_msg.linear_acceleration[1] as f64,
                imu_msg.linear_acceleration[2] as f64,
            );

            let tmp_angular_velocity = Vector3::new(
                imu_msg.angular_velocity[0] as f64,
                imu_msg.angular_velocity[1] as f64,
                imu_msg.angular_velocity[2] as f64,
            );

            if tmp_angular_velocity.x.is_finite()
                && tmp_angular_velocity.y.is_finite()
                && tmp_angular_velocity.z.is_finite()
            {
                angular_velocity_opt = Some(tmp_angular_velocity);
            }

            // Align down axis using accelerometer data
            if acceleration.x.is_finite()
                && acceleration.y.is_finite()
                && acceleration.z.is_finite()
            {
                let acceleration_world =
                    UnitVector3::new_normalize(isometry.transform_vector(&acceleration));

                let angle = down_axis.angle(&acceleration_world)
                    * lerp_value(LOCALIZATION_DELTA, ACCELEROMETER_LERP_SPEED);

                if angle > 0.001 {
                    let cross = UnitVector3::new_normalize(down_axis.cross(&acceleration_world));
                    isometry.append_rotation_wrt_center_mut(&UnitQuaternion::from_axis_angle(
                        &cross, -angle,
                    ));
                }
            }
        }

        // Update down_axis after possible adjustment
        down_axis = isometry.rotation * down_axis;

        // Updated localization hierarchy: AprilTags (primary) -> L2 KISS ICP (secondary) -> RealSense KISS ICP (tertiary) -> IMU only
        let mut pose_updated = false;

        // Process camera-specific apriltag input if available (PRIMARY)
        if let Some(camera_transforms) = input.3.payload() {
            let mut all_observer_isometries = Vec::new();

            // Process each camera's detections
            for (camera_id, transform) in camera_transforms.iter() {
                // Get the camera node from the robot chain
                if let Some(camera_node) = self.root_node.get_node_with_name(camera_id) {
                    // Get the camera's isometry in the robot frame
                    let mut camera_isometry = camera_node.get_isometry_from_base();
                    // Create inverse to transform from camera frame to robot frame
                    camera_isometry.inverse_mut();

                    // Convert the camera's observer isometry to robot frame
                    let camera_observer_iso: Isometry3<f64> = transform.into();
                    let robot_frame_observer_iso = camera_observer_iso * camera_isometry;

                    all_observer_isometries.push(robot_frame_observer_iso);
                } else {
                    return Err(CuError::new_with_cause(
                        &format!("Camera node '{}' not found in robot chain", camera_id),
                        std::io::Error::new(std::io::ErrorKind::NotFound, "Camera node not found"),
                    ));
                }
            }

            if !all_observer_isometries.is_empty() {
                let combined_observer_iso = if all_observer_isometries.len() == 1 {
                    all_observer_isometries[0]
                } else {
                    let mut sum_translation = Vector3::zeros();
                    for iso in &all_observer_isometries {
                        sum_translation += iso.translation.vector;
                    }
                    let mean_translation = sum_translation / all_observer_isometries.len() as f64;

                    let mean_rotation = all_observer_isometries[0].rotation;

                    Isometry3::from_parts(mean_translation.into(), mean_rotation)
                };

                let down_axis = -Vector3::y_axis();

                isometry.translation.vector = lerp(
                    isometry.translation.vector,
                    combined_observer_iso.translation.vector,
                    LOCALIZATION_DELTA,
                    ACCELEROMETER_LERP_SPEED,
                );

                let (_, new_twist) =
                    swing_twist_decomposition(&combined_observer_iso.rotation, &down_axis);
                let (old_swing, _) = swing_twist_decomposition(&isometry.rotation, &down_axis);
                let new_rotation = old_swing * new_twist;

                if new_rotation.w.is_finite()
                    && new_rotation.i.is_finite()
                    && new_rotation.j.is_finite()
                    && new_rotation.k.is_finite()
                {
                    let dot_product = isometry.rotation.coords.dot(&new_rotation.coords);

                    let target_quat = if dot_product < 0.0 {
                        UnitQuaternion::new_normalize(-new_rotation.into_inner())
                    } else {
                        new_rotation
                    };

                    // Use lerp for the quaternion interpolation with proper direction
                    isometry.rotation = UnitQuaternion::new_normalize(lerp(
                        isometry.rotation.into_inner(),
                        target_quat.into_inner(),
                        LOCALIZATION_DELTA,
                        ACCELEROMETER_LERP_SPEED,
                    ));
                }

                // Mark that pose was updated by AprilTags
                pose_updated = true;
            }
        }

        // Try L2 KISS ICP if AprilTags didn't update pose (SECONDARY - preferred over RealSense)
        if !pose_updated {
            if let Some(l2_transform) = input.1.payload() {
                let l2_iso: Isometry3<f64> = l2_transform.into();

                info!("Using L2 KISS-ICP for localization");

                isometry.translation.vector = lerp(
                    isometry.translation.vector,
                    l2_iso.translation.vector,
                    LOCALIZATION_DELTA,
                    ACCELEROMETER_LERP_SPEED * 0.25, // Reduced smoothing for L2
                );

                // Use L2 rotation as well
                if l2_iso.rotation.w.is_finite()
                    && l2_iso.rotation.i.is_finite()
                    && l2_iso.rotation.j.is_finite()
                    && l2_iso.rotation.k.is_finite()
                {
                    let dot_product = isometry.rotation.coords.dot(&l2_iso.rotation.coords);
                    let target_quat = if dot_product < 0.0 {
                        UnitQuaternion::new_normalize(-l2_iso.rotation.into_inner())
                    } else {
                        l2_iso.rotation
                    };
                    isometry.rotation = UnitQuaternion::new_normalize(lerp(
                        isometry.rotation.into_inner(),
                        target_quat.into_inner(),
                        LOCALIZATION_DELTA,
                        ACCELEROMETER_LERP_SPEED * 0.25,
                    ));
                }

                pose_updated = true;
            }
        }

        // Try RealSense KISS ICP if both AprilTags and L2 didn't update pose (TERTIARY)
        if !pose_updated {
            if let Some(realsense_transform) = input.2.payload() {
                let realsense_iso: Isometry3<f64> = realsense_transform.into();

                info!("Using RealSense KISS-ICP for localization");

                isometry.translation.vector = lerp(
                    isometry.translation.vector,
                    realsense_iso.translation.vector,
                    LOCALIZATION_DELTA,
                    ACCELEROMETER_LERP_SPEED * 0.5, // Lower confidence than L2
                );

                // Use RealSense rotation as well
                if realsense_iso.rotation.w.is_finite()
                    && realsense_iso.rotation.i.is_finite()
                    && realsense_iso.rotation.j.is_finite()
                    && realsense_iso.rotation.k.is_finite()
                {
                    let dot_product = isometry.rotation.coords.dot(&realsense_iso.rotation.coords);
                    let target_quat = if dot_product < 0.0 {
                        UnitQuaternion::new_normalize(-realsense_iso.rotation.into_inner())
                    } else {
                        realsense_iso.rotation
                    };
                    isometry.rotation = UnitQuaternion::new_normalize(lerp(
                        isometry.rotation.into_inner(),
                        target_quat.into_inner(),
                        LOCALIZATION_DELTA,
                        ACCELEROMETER_LERP_SPEED * 0.5,
                    ));
                }

                pose_updated = true;
            }
        }

        // Fall back to IMU-only if no other localization sources are available
        if !pose_updated {
            if let Some(angular_velocity) = angular_velocity_opt {
                info!("Using IMU-only for localization");
                isometry.append_rotation_wrt_center_mut(&UnitQuaternion::from_axis_angle(
                    &down_axis,
                    -angular_velocity.y * LOCALIZATION_DELTA,
                ));
            }
        }

        self.root_node.set_isometry(isometry);

        if self.last_rerun_log.elapsed() >= Duration::from_secs_f64(1.0 / 60.0) {
            self.last_rerun_log = Instant::now();
            if let Some(recorder) = rerun_viz::RECORDER.get() {
                if let Err(e) = recorder.recorder.log(
                    rerun_viz::ROBOT_STRUCTURE,
                    &rerun::Transform3D::from_translation_rotation(
                        isometry.translation.vector.cast::<f32>().data.0[0],
                        rerun::Quaternion::from_xyzw(
                            isometry.rotation.as_vector().cast::<f32>().data.0[0],
                        ),
                    ),
                ) {
                    return Err(CuError::new_with_cause(
                        &format!("Failed to log robot transform: {e}"),
                        std::io::Error::new(std::io::ErrorKind::Other, "Rerun logging failed"),
                    ));
                }
            }
        }
        let elapsed = (clock.now().as_nanos() - start) / 1000;
        Ok(())
    }
}
