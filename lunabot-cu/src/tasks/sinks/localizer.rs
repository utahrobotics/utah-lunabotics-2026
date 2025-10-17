use std::{collections::HashMap, f32::consts::E};

use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    CuError,
    clock::{CuTime, Instant},
    cutask::{CuMsg, CuSinkTask, Freezable},
    input_msg,
};
use embedded_common::FromPicoV3;
use iceoryx_types::ImuMsg;
use iceoryx2::{
    node::NodeBuilder,
    port::publisher::Publisher,
    prelude::{ServiceName, UnableToDeliverStrategy},
    service::ipc,
};
use nalgebra::{Isometry3, UnitQuaternion, UnitVector3, Vector3};
use simple_motion::StaticNode;

use crate::{
    ROOT_NODE,
    rerun_viz::{self, RECORDER},
    utils::{lerp, lerp_value, swing_twist_decomposition},
};

use kalman_filter::*;

const ACCELEROMETER_LERP_SPEED: f64 = 150.0;
const ICP_FILTER_SPEED: f64 = 100.0;
const LOCALIZATION_DELTA: f64 = 1.0 / 60.0;

pub struct Localizer {
    root_node: StaticNode,
    last_rerun_log: Instant,
    kiss_icp_correction: Option<Isometry3<f64>>,
    last_icp_reading: Option<(Isometry3<f64>, u64)>,
    last_imu_orientation: Option<(OrientationComponents, u64)>,
    // publishes EncodableIsometry at 60hz
    root_node_publisher: Publisher<ipc::Service, [f64; 16], ()>,
    realsense_node_publisher: Publisher<ipc::Service, [f64; 16], ()>,
    /// Stores all information of kalman filter, including past values, and update logic.
    kalman_filter: KalmanFilter<6>,
    most_recent_update: CuTime,
}

impl Freezable for Localizer {}

impl CuSinkTask for Localizer {
    // IMU from l2, apriltag detections
    type Input<'m> = input_msg!('m,
        ImuMsg, // l2 imu
        EncodableIsometry, // l2 kiss icp
        FromPicoV3,
        Box<HashMap<String, EncodableIsometry>> // apriltag detections
    );

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("Localizer: node create", e))?;

        let service = node
            .service_builder(
                &ServiceName::new("localizer/root_isometry")
                    .map_err(|e| CuError::new_with_cause("Localizer: invalid service name", e))?,
            )
            .publish_subscribe::<[f64; 16]>()
            .enable_safe_overflow(true)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("Localizer: service", e))?;

        let publisher = service
            .publisher_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("Localizer: publisher", e))?;

        let service_realsense = node
            .service_builder(
                &ServiceName::new("localizer/realsense_isometry")
                    .map_err(|e| CuError::new_with_cause("Localizer: invalid service name", e))?,
            )
            .publish_subscribe::<[f64; 16]>()
            .enable_safe_overflow(true)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("Localizer: service", e))?;

        let publisher_realsense = service_realsense
            .publisher_builder()
            .unable_to_deliver_strategy(UnableToDeliverStrategy::Block)
            .create()
            .map_err(|e| CuError::new_with_cause("Localizer: publisher", e))?;

        let kalman_filter = KalmanFilter::new(
            SimpleVector::from_element(0.0),
            SimpleSquareMatrix::from_diagonal_element(100.0),
            |vec, mat, dt| {
                (
                    vec,
                    // e^a * e^b = e^(a+b), so the operation is consistent regardless of time division
                    mat * SimpleSquareMatrix::from_diagonal_element(f64::powf(E.into(), dt)),
                )
            },
        );

        if let Some(root_node) = ROOT_NODE.get() {
            return Ok(Self {
                root_node: root_node.clone(),
                last_rerun_log: Instant::now(),
                kiss_icp_correction: None,
                last_icp_reading: None,
                last_imu_orientation: None,
                root_node_publisher: publisher,
                realsense_node_publisher: publisher_realsense,
                kalman_filter,
                most_recent_update: CuTime::default(),
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
            let iso = self.compute_imu_swing_twist(acceleration);
            self.last_imu_orientation = iso.clone().map(|iso| (iso, clock.now().as_nanos()));
            iso
        } else {
            None
        };

        let apriltag_components = if let Some(estimated_camera_isometries) = input.3.payload() {
            self.compute_apriltag_swing_twist(estimated_camera_isometries)
        } else {
            None
        };

        let fused_isometry = self.fuse_sensor_data(&imu_components, &apriltag_components);
        // if icp and apriltag readings are within 1 ms then calculate the icp correction
        if let Some(fused_isometry) = fused_isometry
            && apriltag_components.is_some()
            && let Some(icp) = self.last_icp_reading
            && (clock.now().as_nanos() - icp.1) < 50_000_000
        {
            let correction = Self::transformation_between(icp.0, fused_isometry);
            if let Some(rec) = RECORDER.get() {
                rec.recorder
                    .log(
                        "kiss_icp",
                        &rerun::Transform3D::from_translation_rotation(
                            correction.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                correction.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    )
                    .unwrap();
            }
            self.kiss_icp_correction = Some(correction);
        }

        // compute final isometry, take swing from imu always (if last reading is < 1ms ago) and twist from corrected icp, as well as translation from corrected icp.
        let final_isometry = if let Some(kiss_icp) = input.1.payload() {
            let kiss_icp_iso: Isometry3<f64> = kiss_icp.to_na().unwrap_or(Isometry3::identity());
            self.last_icp_reading = Some((kiss_icp_iso, clock.now().as_nanos()));

            let corrected_icp = if let Some(correction) = self.kiss_icp_correction {
                correction * kiss_icp_iso
            } else {
                kiss_icp_iso
            };

            if let Some((imu_components, imu_time)) = &self.last_imu_orientation {
                if clock.now().as_nanos() - imu_time < 50_000_000 {
                    let down_axis = -Vector3::z_axis();
                    let (_icp_swing, icp_twist) =
                        swing_twist_decomposition(&corrected_icp.rotation, &down_axis);
                    let combined_rotation = imu_components.swing * icp_twist;

                    Some(Isometry3::from_parts(
                        corrected_icp.translation,
                        combined_rotation,
                    ))
                } else {
                    Some(corrected_icp)
                }
            } else {
                Some(corrected_icp)
            }
        } else {
            fused_isometry
        };

        let dt: f64 = (clock.now() - self.most_recent_update).as_nanos() as f64 / 1e9;
        self.most_recent_update = clock.now();

        self.kalman_filter.step_time(dt);

        if let Some(iso) = final_isometry {
            // Convert proper isometry into vector representation used by kalman filter for interpolation.
            let measurement_vector = iso_to_vec(iso);
            let variance_matrix = SimpleSquareMatrix::from_diagonal_element(0.1);

            // Enter measurement into filter
            self.kalman_filter
                .apply_measurement(&measurement_vector, &variance_matrix);

            // Report kalman filter state (converted to isometry) as robot position
            self.root_node
                .set_isometry(vec_to_iso(self.kalman_filter.get_current_state()));
        }

        if self.last_rerun_log.elapsed().as_nanos() > 16_666_667 {
            let isometry = self.root_node.get_global_isometry();
            let encodeable_isometry = EncodableIsometry::from_na(&isometry);
            if let Err(e) = self
                .root_node_publisher
                .send_copy(encodeable_isometry.inner)
            {
                eprintln!("localizer publish err: {e}");
            }
            let realsense_iso = self
                .root_node
                .get_node_with_name("upper_depth_camera")
                .unwrap()
                .get_global_isometry();
            if let Err(e) = self
                .realsense_node_publisher
                .send_copy(EncodableIsometry::from_na(&realsense_iso).inner)
            {
                eprintln!("localizer publish err: {e}");
            }
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
                let isometry = self
                    .root_node
                    .get_node_with_name("l2_front")
                    .unwrap()
                    .get_global_isometry();
                recorder
                    .recorder
                    .log_static(
                        "l2_node",
                        &rerun::Arrows3D::from_vectors([
                            [0.2, 0.0, 0.0],
                            [0.0, 0.2, 0.0],
                            [0.0, 0.0, 0.2],
                        ])
                        .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                        .with_labels(vec!["x", "y", "z"]),
                    )
                    .unwrap();
                recorder
                    .recorder
                    .log(
                        "l2_node",
                        &rerun::Transform3D::from_translation_rotation(
                            isometry.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                isometry.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    )
                    .unwrap();
            }
        }

        Ok(())
    }
}

fn vec_to_iso(vec: SimpleVector<6>) -> Isometry3<f64> {
    let translation: Vector3<f64> = Vector3::<f64>::new(vec.x, vec.y, vec.z);

    let rotation: Vector3<f64> = Vector3::<f64>::new(vec.w, vec.a, vec.b);

    Isometry3::<f64>::new(translation, rotation)
}

fn iso_to_vec(iso: Isometry3<f64>) -> SimpleVector<6> {
    let rotation_vector = iso.rotation.scaled_axis();

    SimpleVector::<6>::new(
        iso.translation.x,
        iso.translation.y,
        iso.translation.z,
        rotation_vector.x,
        rotation_vector.y,
        rotation_vector.z,
    )
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

            isometry.translation = combined_observer_iso.translation;

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
    fn transformation_between(relative: Isometry3<f64>, actual: Isometry3<f64>) -> Isometry3<f64> {
        actual * relative.inverse()
    }
}
