use std::f64::consts::PI;

use crate::rerun_viz;
use crate::rerun_viz::RECORDER;
use crate::tasks::AprilTagMeasurement;
use crate::tasks::IcpMeasurement;
use crate::tasks::ImuMeasurement;
use crate::utils::RobotState;
use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    CuError,
    clock::{CuTime, Instant},
    cutask::{CuMsg, CuSinkTask, Freezable},
    input_msg,
};
use embedded_common::FromPicoV3;

use nalgebra::Const;
use nalgebra::SMatrixViewMut;
use nalgebra::{Isometry3, Vector3};
use rerun::Quaternion;
use simple_motion::StaticNode;

use crate::ROBOT_STATE;

use kalman_filter::*;

/// Represents the variance assigned to values that are completely unknown. Should be extremely large.
const UNKNOWN_PRIOR_VARIANCE: f64 = 1e64;

pub struct Localizer {
    robot_state: RobotState,
    last_rerun_log: Instant,
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
        Vec<AprilTagMeasurement>, // apriltag detections
        EncodableIsometry // reading from the t265, estimation of the robots isometry
    );

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let kalman_filter = KalmanFilter::new(
            SimpleVector::from_element(0.0),
            SimpleSquareMatrix::from_diagonal_element(100.0),
            Self::evolution_function,
        );

        if let Some(robot_state) = ROBOT_STATE.get() {
            return Ok(Self {
                robot_state: robot_state.clone(),
                last_rerun_log: Instant::now(),
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

    fn start(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
        if let Some(logger) = RECORDER.get() {
            let _ = logger.recorder.log_static(
                format!("t265/xyz"),
                &rerun::Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                    .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                    .with_labels(vec!["x", "y", "z"]),
            );
        }

        Ok(())
    }

    fn process<'i>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        input: &Self::Input<'i>,
    ) -> cu29::CuResult<()> {
        if let Some(pose_msg) = input.4.payload()
            && let Some(logger) = RECORDER.get()
            && let Some(pose_msg) = pose_msg.to_na()
        {
            let _ = logger.recorder.log(
                "t265",
                &rerun::Transform3D::from_translation_rotation(
                    pose_msg.translation.vector.data.0[0],
                    Quaternion::from_xyzw(pose_msg.rotation.as_vector().cast::<f32>().data.0[0]),
                ),
            );
        }

        if let Some(imu_measurement) = input.0.payload()
            && let Some(logger) = RECORDER.get()
        {
            let _ = logger.recorder.log(
                "imu_corercted",
                &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                    imu_measurement.acceleration[0] as f32,
                    imu_measurement.acceleration[1] as f32,
                    imu_measurement.acceleration[2] as f32,
                )]),
            );
            //self.root_node.set_isometry(pose_msg);
        }

        // Step filter forward in time
        let dt: f64 = ((clock.now() - self.most_recent_update).as_nanos() as f64) / 1e9;
        self.most_recent_update = clock.now();
        self.kalman_filter.step_time(dt);

        // Apply all data
        if let Some(imu_measurement) = input.0.payload() {
            // Assemble measurement
            let mut state: SimpleVector<15> = SimpleVector::zeros();
            state
                .fixed_rows_mut::<3>(6)
                .copy_from_slice(&imu_measurement.acceleration);
            state
                .fixed_rows_mut::<3>(9)
                .copy_from_slice(&imu_measurement.orientation);
            state
                .fixed_rows_mut::<3>(12)
                .copy_from_slice(&imu_measurement.angular_velocity);

            let mut covariance_matrix: SimpleSquareMatrix<15> =
                SimpleSquareMatrix::from_diagonal_element(UNKNOWN_PRIOR_VARIANCE);
            let measurement_matrix = array_to_matrix_9x9(&imu_measurement.variance);

            covariance_matrix
                .view_mut((6, 6), (9, 9))
                .copy_from(&measurement_matrix.view((0, 0), (9, 9)));

            self.kalman_filter
                .apply_measurement(&state, &covariance_matrix);
        }

        if let Some(icp_measurement) = input.1.payload() {
            // Assemble measurement
            let mut state: SimpleVector<15> = SimpleVector::zeros();
            state
                .fixed_rows_mut::<3>(0)
                .copy_from_slice(&icp_measurement.position);
            state
                .fixed_rows_mut::<3>(9)
                .copy_from_slice(&icp_measurement.orientation);

            let mut covariance_matrix: SimpleSquareMatrix<15> =
                SimpleSquareMatrix::from_diagonal_element(UNKNOWN_PRIOR_VARIANCE);
            let measurement_matrix = array_to_matrix_6x6(&icp_measurement.variance);

            covariance_matrix
                .view_mut((0, 0), (3, 3))
                .copy_from(&measurement_matrix.view((0, 0), (3, 3)));
            covariance_matrix
                .view_mut((0, 9), (3, 3))
                .copy_from(&measurement_matrix.view((0, 3), (3, 3)));
            covariance_matrix
                .view_mut((9, 0), (3, 3))
                .copy_from(&measurement_matrix.view((3, 0), (3, 3)));
            covariance_matrix
                .view_mut((9, 9), (3, 3))
                .copy_from(&measurement_matrix.view((3, 3), (3, 3)));

            self.kalman_filter
                .apply_measurement(&state, &covariance_matrix);
        }

        if let Some(tags) = input.3.payload() {
            for tag in tags {
                let estimated_isometry_of_observer =
                    tag.estimated_isometry
                        .to_na()
                        .ok_or(CuError::new_with_cause(
                            "Invalid Isometry",
                            std::io::Error::other("Invalid encoded isometry"),
                        ))?;
                let mut state: SimpleVector<15> = SimpleVector::zeros();
                state
                    .fixed_rows_mut::<3>(0)
                    .copy_from(&estimated_isometry_of_observer.translation.vector);
                state
                    .fixed_rows_mut::<3>(9)
                    .copy_from(&estimated_isometry_of_observer.rotation.scaled_axis());

                let mut covariance_matrix: SimpleSquareMatrix<15> =
                    SimpleSquareMatrix::from_diagonal_element(UNKNOWN_PRIOR_VARIANCE);
                let measurement_matrix = array_to_matrix_6x6(&tag.variance);

                covariance_matrix
                    .view_mut((0, 0), (3, 3))
                    .copy_from(&measurement_matrix.view((0, 0), (3, 3)));
                covariance_matrix
                    .view_mut((0, 9), (3, 3))
                    .copy_from(&measurement_matrix.view((0, 3), (3, 3)));
                covariance_matrix
                    .view_mut((9, 0), (3, 3))
                    .copy_from(&measurement_matrix.view((3, 0), (3, 3)));
                covariance_matrix
                    .view_mut((9, 9), (3, 3))
                    .copy_from(&measurement_matrix.view((3, 3), (3, 3)));

                self.kalman_filter
                    .apply_measurement(&state, &covariance_matrix);
            }
        }

        // Log and update chain at 60 Hz
        if let Some(logger) = RECORDER.get()
            && self.last_rerun_log.elapsed().as_millis() > 1000 / 60
        {
            self.last_rerun_log = Instant::now();

            //   Push data to rest of robot
            // Convert to isometry for the kinematics object
            let current_state = self.kalman_filter.get_current_state();
            let mut iso_vector = SimpleVector::<6>::from_element(0.0);
            iso_vector[(0, 0)] = current_state[(0, 0)];
            iso_vector[(1, 0)] = current_state[(1, 0)];
            iso_vector[(2, 0)] = current_state[(2, 0)];
            iso_vector[(3, 0)] = current_state[(9, 0)];
            iso_vector[(4, 0)] = current_state[(10, 0)];
            iso_vector[(5, 0)] = current_state[(11, 0)];
            let current_iso = vec_to_iso(iso_vector);
            self.robot_state.kinematic_root.set_isometry(current_iso);

            {
                let mut global_variance_lock = self.robot_state.kalman_variances.write();
                let mut global_variance_view: SMatrixViewMut<f64, 15, 15> =
                    global_variance_lock.as_mut().unwrap().as_view_mut();
                global_variance_view.copy_from(&self.kalman_filter.get_current_covariance());
            }

            {
                let mut global_state_lock = self.robot_state.kalman_state.write();
                let mut global_state_view: SMatrixViewMut<f64, 15, 1> =
                    global_state_lock.as_mut().unwrap().as_view_mut();
                global_state_view.copy_from(&self.kalman_filter.get_current_state());
            }

            //   Log data to rerun
            // Variance
            let _ = logger.recorder.log(
                "kalman_state/state",
                &rerun::Tensor::new(self.kalman_filter.get_current_covariance().data.as_slice()),
            );

            // State
            let _ = logger.recorder.log(
                "kalman_state/position",
                &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                    current_state[0] as f32,
                    current_state[1] as f32,
                    current_state[2] as f32,
                )]),
            );
            let _ = logger.recorder.log(
                "kalman_state/velocity",
                &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                    current_state[3] as f32,
                    current_state[4] as f32,
                    current_state[5] as f32,
                )]),
            );
            let _ = logger.recorder.log(
                "kalman_state/acceleration",
                &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                    current_state[6] as f32,
                    current_state[7] as f32,
                    current_state[8] as f32,
                )]),
            );
            let _ = logger.recorder.log(
                "kalman_state/orientation",
                &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                    current_state[9] as f32,
                    current_state[10] as f32,
                    current_state[11] as f32,
                )]),
            );
            let _ = logger.recorder.log(
                "kalman_state/angular_velocity",
                &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                    current_state[12] as f32,
                    current_state[13] as f32,
                    current_state[14] as f32,
                )]),
            );

            // Robot position
            if let Err(e) = logger.recorder.log(
                rerun_viz::ROBOT_STRUCTURE,
                &rerun::Transform3D::from_translation_rotation(
                    current_iso.translation.vector.cast::<f32>().data.0[0],
                    rerun::Quaternion::from_xyzw(
                        current_iso.rotation.as_vector().cast::<f32>().data.0[0],
                    ),
                ),
            ) {
                return Err(CuError::new_with_cause(
                    &format!("Failed to log robot transform: {e}"),
                    std::io::Error::new(std::io::ErrorKind::Other, "Rerun logging failed"),
                ));
            };
            // TODO add proper logging and display for other state components
        }

        Ok(())
    }
}

fn vec_to_iso(vec: SimpleVector<6>) -> Isometry3<f64> {
    let translation: Vector3<f64> = Vector3::<f64>::new(vec.x, vec.y, vec.z);

    let rotation: Vector3<f64> = Vector3::<f64>::new(vec.w, vec.a, vec.b);

    Isometry3::<f64>::new(translation, rotation)
}

#[allow(unused)]
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

fn array_to_matrix_9x9(array: &[f64; 81]) -> SimpleSquareMatrix<9> {
    SimpleSquareMatrix::<9>::from_row_slice(array)
}

fn array_to_matrix_6x6(array: &[f64; 36]) -> SimpleSquareMatrix<6> {
    SimpleSquareMatrix::<6>::from_row_slice(array)
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
        dt: f64,
    ) -> (SimpleVector<15>, SimpleSquareMatrix<15>) {
        // State:
        let mut result_state: SimpleVector<15> = SimpleVector::from_element(0.0);

        // Decompose prev state
        let prev_position = prev_state.view((0, 0), (3, 1));
        let prev_velocity = prev_state.view((3, 0), (3, 1));
        let prev_acceleration = prev_state.view((6, 0), (3, 1));
        let prev_orientation = prev_state.view((9, 0), (3, 1));
        let prev_angular_velocity = prev_state.view((12, 0), (3, 1));

        // Translational
        result_state
            .fixed_rows_mut::<3>(0)
            .copy_from(&(prev_position + prev_velocity * dt + 0.5 * prev_acceleration * dt * dt));

        result_state
            .fixed_rows_mut::<3>(3)
            .copy_from(&(prev_velocity + prev_acceleration * dt));

        result_state
            .fixed_rows_mut::<3>(6)
            .copy_from(&prev_acceleration);

        // Angular
        let mut temp_result_orientation = prev_orientation + prev_angular_velocity * dt;
        if temp_result_orientation.magnitude_squared() > PI * PI {
            temp_result_orientation -= temp_result_orientation.normalize() * -2.0 * PI;
        }
        result_state
            .fixed_rows_mut::<3>(9)
            .copy_from(&temp_result_orientation);

        result_state
            .fixed_rows_mut::<3>(12)
            .copy_from(&prev_angular_velocity);

        // Variances:
        let mut result_variance = prev_variance;

        // Formula: x = x + v*dt => o_x^2 = o_x^2 + o_v^2 * dt^2
        // Translational
        //   s_v += s_a * dt^2
        let calculated_velocity_variance =
            result_variance.view((3, 3), (3, 3)) + result_variance.view((6, 6), (3, 3)) * dt * dt;
        result_variance
            .view_mut((3, 3), (3, 3))
            .copy_from(&calculated_velocity_variance);
        //   s_x += s_v * dt^2
        let calculated_position_variance =
            result_variance.view((0, 0), (3, 3)) + result_variance.view((3, 3), (3, 3)) * dt * dt;
        result_variance
            .view_mut((0, 0), (3, 3))
            .copy_from(&calculated_position_variance);

        // Angular
        //   s_theta += s_o * dt^2
        let calculated_orientation_variance =
            result_variance.view((9, 9), (3, 3)) + result_variance.view((12, 12), (3, 3)) * dt * dt;
        result_variance
            .view_mut((9, 9), (3, 3))
            .copy_from(&calculated_orientation_variance);

        (result_state, result_variance)
    }

    /// Returns the transformation needed to transform src to dst.
    /// Used to find the correction between where kiss_icp thinks the robot
    /// is relative to where the algorithm started mapping, to where the robot
    /// actually is in world coords
    #[allow(unused)]
    fn transformation_between(relative: Isometry3<f64>, actual: Isometry3<f64>) -> Isometry3<f64> {
        actual * relative.inverse()
    }
}
