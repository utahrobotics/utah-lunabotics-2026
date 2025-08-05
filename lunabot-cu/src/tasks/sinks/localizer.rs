use std::collections::HashMap;

use cu_spatial_payloads::Transform3D;
use cu29::{
    CuError,
    cutask::{CuMsg, CuSinkTask, Freezable},
    input_msg,
};
use iceoryx_types::ImuMsg;
use nalgebra::{Isometry3, Transform3, UnitQuaternion, UnitVector3, Vector3};
use simple_motion::StaticNode;

use crate::{
    ROOT_NODE, rerun_viz,
    tasks::april_detection_handler::EncodableIsometry,
    utils::{lerp, lerp_value, swing_twist_decomposition},
};

const ACCELEROMETER_LERP_SPEED: f64 = 200.0;
const LOCALIZATION_DELTA: f64 = 1.0 / 60.0;

pub struct Localizer {
    root_node: StaticNode,
    last_rerun_log: u64,
    kiss_icp_correction: Option<Transform3<f64>>,
}

impl Freezable for Localizer {}

impl CuSinkTask for Localizer {
    // IMU from l2, apriltag detections
    type Input<'m> = input_msg!('m,
        ImuMsg, // l2 imu
        Transform3D<f64>, // realsense kiss icp
        Transform3D<f64>, // l2 icp
        Box<HashMap<String, EncodableIsometry>>);

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        if let Some(root_node) = ROOT_NODE.get() {
            return Ok(Self {
                root_node: root_node.clone(),
                last_rerun_log: 0,
                kiss_icp_correction: None,
            });
        } else {
            return Err(CuError::new_with_cause(
                "no root node found",
                std::io::Error::other("no root node found"),
            ));
        }
    }

    fn process<'i>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        input: &Self::Input<'i>,
    ) -> cu29::CuResult<()> {
        let imu_components = if let Some(imu_raw) = input.0.payload() {
            let acceleration = Vector3::new(
                imu_raw.linear_acceleration[0] as f64,
                imu_raw.linear_acceleration[1] as f64,
                imu_raw.linear_acceleration[2] as f64,
            );
            self.compute_imu_swing_twist(acceleration)
        } else {
            None
        };

        let apriltag_components = if let Some(estimated_camera_isometries) = input.3.payload() {
            self.compute_apriltag_swing_twist(estimated_camera_isometries)
        } else {
            None
        };

        let fused_isometry = self.fuse_sensor_data(&imu_components, &apriltag_components);
        let final_isometry = if let Some(unitree_icp_out) = input.2.payload() {
            let icp_isometry: Isometry3<f64> = unitree_icp_out.into();

            match (fused_isometry, &self.kiss_icp_correction) {
                (_, Some(correction)) => {
                    let corrected_icp_translation = correction
                        .transform_point(&icp_isometry.translation.vector.into())
                        .coords;

                    let rotation_matrix = correction.matrix().fixed_view::<3, 3>(0, 0).into_owned();
                    let corrected_icp_rotation =
                        UnitQuaternion::from_matrix(&rotation_matrix) * icp_isometry.rotation;

                    Some(Isometry3::from_parts(
                        corrected_icp_translation.into(),
                        corrected_icp_rotation,
                    ))
                }
                (Some(fused), None) => Some(fused),
                (None, None) => Some(icp_isometry),
            }
        } else {
            fused_isometry
        };

        if let Some(iso) = final_isometry {
            println!("setting root nodes iso");
            self.root_node.set_isometry(iso);
        }

        if clock.now().as_nanos() - self.last_rerun_log >= 16_666_667 {
            // log at 60 hz
            let isometry = self.root_node.get_global_isometry();
            self.last_rerun_log = clock.now().as_nanos();
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

        Ok(())
    }
}

#[derive(Debug, Clone)]
struct OrientationComponents {
    swing: UnitQuaternion<f64>,
    twist: UnitQuaternion<f64>,
    full_rotation: UnitQuaternion<f64>,
    translation: Vector3<f64>,
}

impl Localizer {
    /// Compute swing-twist components from IMU data
    /// Returns swing (pitch/roll from gravity) and twist (unreliable yaw)
    fn compute_imu_swing_twist(&self, acceleration: Vector3<f64>) -> Option<OrientationComponents> {
        let mut isometry = self.root_node.get_global_isometry();
        let down_axis = Vector3::z_axis();

        if acceleration.x.is_finite() && acceleration.y.is_finite() && acceleration.z.is_finite() {
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

            let (swing, twist) = swing_twist_decomposition(&isometry.rotation, &down_axis);

            Some(OrientationComponents {
                swing,
                twist,
                full_rotation: isometry.rotation,
                translation: isometry.translation.vector,
            })
        } else {
            None
        }
    }

    /// Compute swing-twist components from AprilTag data
    fn compute_apriltag_swing_twist(
        &self,
        camera_transforms: &Box<HashMap<String, EncodableIsometry>>,
    ) -> Option<OrientationComponents> {
        let mut isometry = Isometry3::identity();
        let mut all_observer_isometries = Vec::new();

        for (camera_id, transform) in camera_transforms.iter() {
            if let Some(camera_node) = self.root_node.get_node_with_name(camera_id) {
                let mut camera_isometry = camera_node.get_isometry_from_base();
                camera_isometry.inverse_mut();

                let camera_observer_iso: Isometry3<f64> = transform.to_na()?;
                let robot_frame_observer_iso = camera_observer_iso * camera_isometry;

                all_observer_isometries.push(robot_frame_observer_iso);
            } else {
                eprintln!("camera node: {} not found", camera_id);
                return None;
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

                // TODO: make this an actual avg
                let mean_rotation = all_observer_isometries[0].rotation;

                Isometry3::from_parts(mean_translation.into(), mean_rotation)
            };

            let down_axis = Vector3::z_axis();

            // Apply lerp to translation
            isometry.translation.vector = lerp(
                isometry.translation.vector,
                combined_observer_iso.translation.vector,
                LOCALIZATION_DELTA,
                ACCELEROMETER_LERP_SPEED,
            );

            // Decompose the AprilTag rotation
            let (swing, twist) =
                swing_twist_decomposition(&combined_observer_iso.rotation, &down_axis);

            Some(OrientationComponents {
                swing,
                twist,
                full_rotation: combined_observer_iso.rotation,
                translation: isometry.translation.vector,
            })
        } else {
            None
        }
    }

    /// Fuse IMU and AprilTag sensor data using swing-twist decomposition
    /// Takes swing (pitch/roll) from IMU and twist (yaw) from AprilTags
    fn fuse_sensor_data(
        &mut self,
        imu_components: &Option<OrientationComponents>,
        apriltag_components: &Option<OrientationComponents>,
    ) -> Option<Isometry3<f64>> {
        match (imu_components, apriltag_components) {
            (Some(imu), Some(apriltag)) => {
                let fused_rotation = imu.swing * apriltag.twist;

                if fused_rotation.w.is_finite()
                    && fused_rotation.i.is_finite()
                    && fused_rotation.j.is_finite()
                    && fused_rotation.k.is_finite()
                {
                    let current_rotation = self.root_node.get_global_isometry().rotation;
                    let dot_product = current_rotation.coords.dot(&fused_rotation.coords);

                    let target_quat = if dot_product < 0.0 {
                        UnitQuaternion::new_normalize(-fused_rotation.into_inner())
                    } else {
                        fused_rotation
                    };

                    let interpolated_rotation = UnitQuaternion::new_normalize(lerp(
                        current_rotation.into_inner(),
                        target_quat.into_inner(),
                        LOCALIZATION_DELTA,
                        ACCELEROMETER_LERP_SPEED,
                    ));

                    Some(Isometry3::from_parts(
                        apriltag.translation.into(),
                        interpolated_rotation,
                    ))
                } else {
                    Some(Isometry3::from_parts(
                        apriltag.translation.into(),
                        apriltag.full_rotation,
                    ))
                }
            }
            (Some(imu), None) => Some(Isometry3::from_parts(
                imu.translation.into(),
                imu.full_rotation,
            )),
            (None, Some(apriltag)) => Some(Isometry3::from_parts(
                apriltag.translation.into(),
                apriltag.full_rotation,
            )),
            (None, None) => None,
        }
    }

    /// Returns the transformation needed to transform src to dst.
    /// Used to find the correction between where kiss_icp thinks the robot
    /// is relative to where the algorithm started mapping, to where the robot
    /// actually is in world coords
    fn transformation_between(relative: Isometry3<f64>, actual: Isometry3<f64>) -> Transform3<f64> {
        let rotation_correction = relative.rotation.rotation_to(&actual.rotation);
        let translation_correction = actual.translation.vector - relative.translation.vector;
        Transform3::from_matrix_unchecked(
            rotation_correction
                .to_homogeneous()
                .append_translation(&translation_correction),
        )
    }
}
