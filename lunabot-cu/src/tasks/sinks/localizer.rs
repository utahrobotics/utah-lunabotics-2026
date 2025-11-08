use std::{collections::HashMap, f64::consts::PI};

use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    CuError,
    clock::{CuTime, Instant},
    cutask::{CuMsg, CuSinkTask, Freezable},
    input_msg,
};
use embedded_common::FromPicoV3;
use iceoryx2::{
    node::NodeBuilder,
    port::publisher::Publisher,
    prelude::{ServiceName, UnableToDeliverStrategy},
    service::ipc,
};
use nalgebra::{Dim, Isometry3, Matrix, RawStorage, RawStorageMut, UnitQuaternion, UnitVector3, Vector3};
use simple_motion::StaticNode;
use crate::tasks::ImuMeasurement;
use crate::tasks::IcpMeasurement;
use crate::tasks::AprilTagMeasurement;

use crate::{
    ROOT_NODE, utils::{lerp, lerp_value, swing_twist_decomposition}
};

use kalman_filter::*;

const ACCELEROMETER_LERP_SPEED: f64 = 150.0;
const LOCALIZATION_DELTA: f64 = 1.0 / 60.0;
/// Represents the variance assigned to values that are completely unknown. Should be extremely large.
const UNKNOWN_PRIOR_VARIANCE: f64 = 1e64;

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
    kalman_filter: KalmanFilter<15>,
    most_recent_update: CuTime,
}

impl Freezable for Localizer {}

impl CuSinkTask for Localizer {
    // IMU from l2, apriltag detections
    type Input<'m> = input_msg!('m,
        ImuMeasurement, // l2 imu
        IcpMeasurement, // l2 kiss icp
        FromPicoV3,
        Vec<AprilTagMeasurement> // apriltag detections
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
            Self::evolution_function
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

        // Step filter forward in time
        let dt: f64 = ((clock.now() - self.most_recent_update).as_nanos() as f64) / 1e9;
        self.kalman_filter.step_time(dt);


        // Apply all data
        if let Some(imu_measurement) = input.0.payload() {
            // Assemble measurement
            let mut state: SimpleVector<15> = SimpleVector::from_element(0.0);
            state.view_mut((6,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&imu_measurement.acceleration));
            state.view_mut((9,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&imu_measurement.orientation));
            state.view_mut((12,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&imu_measurement.angular_velocity));

            let mut covariance_matrix: SimpleSquareMatrix<15> = SimpleSquareMatrix::from_diagonal_element(UNKNOWN_PRIOR_VARIANCE);
            let measurement_matrix = array_to_matrix_9x9(&imu_measurement.variance);
            
            matrix_copy(
                &mut covariance_matrix.view_mut((6,6), (9,9)), 
                &measurement_matrix.view((0,0), (9,9))
            );

            self.kalman_filter.apply_measurement(&state, &covariance_matrix);
        }

        if let Some(icp_measurement) = input.1.payload() {
            // Assemble measurement
            let mut state: SimpleVector<15> = SimpleVector::from_element(0.0);
            state.view_mut((0,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&icp_measurement.position));
            state.view_mut((9,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&icp_measurement.orientation));

            let mut covariance_matrix: SimpleSquareMatrix<15> = SimpleSquareMatrix::from_diagonal_element(UNKNOWN_PRIOR_VARIANCE);
            let measurement_matrix = array_to_matrix_6x6(&icp_measurement.variance);
            
            matrix_copy(
                &mut covariance_matrix.view_mut((0,0), (3,3)), 
                &measurement_matrix.view((0,0), (3,3))
            );
            matrix_copy(
                &mut covariance_matrix.view_mut((0,9), (3,3)), 
                &measurement_matrix.view((0,3), (3,3))
            );
            matrix_copy(
                &mut covariance_matrix.view_mut((9,0), (3,3)), 
                &measurement_matrix.view((3,0), (3,3))
            );
            matrix_copy(
                &mut covariance_matrix.view_mut((9,9), (3,3)), 
                &measurement_matrix.view((3,3), (3,3))
            );

            self.kalman_filter.apply_measurement(&state, &covariance_matrix);
        }

        if let Some(tags) = input.3.payload() {
            for tag in tags {
                // Assemble measurement
                let mut state: SimpleVector<15> = SimpleVector::from_element(0.0);
                state.view_mut((0,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&tag.position));
                state.view_mut((9,0), (3,1)).set_column(0, &SimpleVector::<3>::from_column_slice(&tag.orientation));

                let mut covariance_matrix: SimpleSquareMatrix<15> = SimpleSquareMatrix::from_diagonal_element(UNKNOWN_PRIOR_VARIANCE);
                let measurement_matrix = array_to_matrix_6x6(&tag.variance);
                
                matrix_copy(
                    &mut covariance_matrix.view_mut((0,0), (3,3)), 
                    &measurement_matrix.view((0,0), (3,3))
                );
                matrix_copy(
                    &mut covariance_matrix.view_mut((0,9), (3,3)), 
                    &measurement_matrix.view((0,3), (3,3))
                );
                matrix_copy(
                    &mut covariance_matrix.view_mut((9,0), (3,3)), 
                    &measurement_matrix.view((3,0), (3,3))
                );
                matrix_copy(
                    &mut covariance_matrix.view_mut((9,9), (3,3)), 
                    &measurement_matrix.view((3,3), (3,3))
                );

                self.kalman_filter.apply_measurement(&state, &covariance_matrix);
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

fn matrix_copy<R: Dim, C: Dim, S1: RawStorageMut<f64, R, C>, S2: RawStorage<f64, R, C>>(target: &mut Matrix<f64, R, C, S1>, src: &Matrix<f64, R, C, S2>) {
    for i in 1..target.ncols() {
        target.set_column(i, &src.column(i));
    }
}

fn array_to_matrix_9x9(array: &[f64; 81]) -> SimpleSquareMatrix<9> {
    let mut result = SimpleSquareMatrix::default();

    for i in 0..81 {
        result[i] = array[i];
    }

    result
}

fn array_to_matrix_6x6(array: &[f64; 36]) -> SimpleSquareMatrix<6> {
    let mut result = SimpleSquareMatrix::default();

    for i in 0..36 {
        result[i] = array[i];
    }

    result
}

#[derive(Debug, Clone)]
struct OrientationComponents {
    swing: UnitQuaternion<f64>,
    twist: UnitQuaternion<f64>,
    full_rotation: UnitQuaternion<f64>,
    translation: Vector3<f64>,
}

impl Localizer {
    /// The evolution function used by the kalman filter. Steps the
    /// state and variance forward by a given timestep in seconds, and
    /// returns the result. Does not use any external data.
    /// 
    /// Predictions for state and variance use simple laws of motion.
    fn evolution_function(
        prev_state: SimpleVector<15>, 
        prev_variance: SimpleSquareMatrix<15>, 
        dt: f64
    ) -> (SimpleVector<15>, SimpleSquareMatrix<15>) {
        // State:
        let mut result_state: SimpleVector<15> = SimpleVector::from_element(0.0);

        // Decompose prev state
        let prev_position = prev_state.view((0,0), (3,1));
        let prev_velocity = prev_state.view((3,0), (3,1));
        let prev_acceleration = prev_state.view((6,0), (3,1));
        let prev_orientation = prev_state.view((9,0), (3,1));
        let prev_angular_velocity = prev_state.view((12,0), (3,1));

        
        // Translational
        let mut result_position = result_state.view_mut((0,0), (3,1));
        matrix_copy(
            &mut result_position, 
            &(prev_position + prev_velocity*dt + 0.5*prev_acceleration*dt*dt)
        );
        
        let mut result_velocity = result_state.view_mut((3,0), (3,1));
        matrix_copy(
            &mut result_velocity, 
            &(prev_velocity + prev_acceleration*dt)
        );
        
        let mut result_acceleration = result_state.view_mut((6,0), (3,1));
        matrix_copy(
            &mut result_acceleration, 
            &prev_acceleration
        );
        
        // Angular
        let mut result_orientation = result_state.view_mut((9,0), (3,1));
        let mut temp_result_orientation = prev_orientation + prev_angular_velocity*dt;
        if temp_result_orientation.magnitude_squared() > PI*PI {
            temp_result_orientation -= temp_result_orientation.normalize() * -2.0*PI;
        }
        matrix_copy(
            &mut result_orientation, 
            &temp_result_orientation
        );
        
        let mut result_angular_velocity = result_state.view_mut((12,0), (3,1));
        matrix_copy(
            &mut result_angular_velocity, 
            &prev_angular_velocity
        );


        // Variances:
        let mut result_variance = prev_variance;

        // Formula: x = x + v*dt => o_x^2 = o_x^2 + o_v^2 * dt^2
        // Translational
        //   s_v += s_a * dt^2
        let calculated_velocity_variance = result_variance.view((3,3), (3,3)) + result_variance.view((6,6), (3,3)) * dt*dt;
        matrix_copy(&mut result_variance.view_mut((3,3), (3,3)), &calculated_velocity_variance);
        //   s_x += s_v * dt^2
        let calculated_position_variance = result_variance.view((0,0), (3,3)) + result_variance.view((3,3), (3,3)) * dt*dt;
        matrix_copy(&mut result_variance.view_mut((0,0), (3,3)), &calculated_position_variance);

        // Angular
        //   s_theta += s_o * dt^2
        let calculated_orientation_variance = result_variance.view((9,9), (3,3)) + result_variance.view((12,12), (3,3)) * dt*dt;
        matrix_copy(&mut result_variance.view_mut((9,9), (3,3)), &calculated_orientation_variance);


        (result_state, result_variance)
    }

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
