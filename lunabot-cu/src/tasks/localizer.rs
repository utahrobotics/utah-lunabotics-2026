///! # Overview:
///!
///! ## How sensors are used:
///! AprilTags -> global pose reference (absolute position & orientation)
///!
///! ICP (5hz) -> local pose tracking (less drift prone so long as you stay in one room and arent moving down long featureless corridors)
///!
///! Intel T265 (200hz) -> short-term motion (velocity from pose deltas more drift prone than ICP)
///!
///! IMU -> currently logged, not yet fused (since the l2's imus are garbage)
///!
///!
///! ## Multiplicative Extended Kalman Filter
///! Based on this: https://matthewhampsey.github.io/blog/2020/07/18/mekf
///!
///! State representation: [x, y, z, vx, vy, vz, δθx, δθy, δθz, ωx, ωy, ωz]
///!
///!  - where δθxyz is a small orientation error (magnitude will never get close to pi which solves the instability problem)
///!
///! Basically the kalman filter state holds the small orientation error, and we keep track of a reference quaternion in the localizer where: q_true = q_reference * q_error
///!  - q_reference is propagated using angular velocity in the kalman state
///!  - on each update, the error is folded into the reference, and the error gets set back to zero, keeping the error from ever getting too big.
///!  
///!  
///! ## Process noise
///! How much you don't trust your motion model. higher noise = motion model trusted more than sensors
///!
use std::sync::OnceLock;

use crate::rerun_viz;
use crate::rerun_viz::RECORDER;
use crate::robot_state::RobotState;
use crate::tasks::AprilTagMeasurement;
use crate::tasks::IcpMeasurement;
use crate::tasks::ImuMeasurement;
use crate::tasks::T265Msg;
use common::FromLunabot;
use cu29::cutask::CuTask;
use cu29::output_msg;
use cu29::{
    CuError,
    clock::CuTime,
    cutask::{CuMsg, Freezable},
    input_msg,
};
use embedded_common::FromPicoV3;

use kfilter::kalman::{EKF, KalmanFilter, KalmanPredictInput, KalmanUpdate};
use kfilter::measurement::LinearMeasurement;
use kfilter::system::StepReturn;
use kfilter::system::System;
use nalgebra::UnitQuaternion;
use nalgebra::{Isometry3, SMatrix, SVector, Vector3, Vector6};

use crate::ROBOT_STATE;

const STATE_DIM: usize = 12;

/// Input dimension: just dt
const INPUT_DIM: usize = 1;

/// Position/orientation measurement dimension
const POSE_MEAS_DIM: usize = 6;

/// Velocity measurement dimension  
const VEL_MEAS_DIM: usize = 6;

// These oncelocks must only be set in the new() function

/// Process noise for position (grows with time)
static PROCESS_NOISE_POSITION: OnceLock<f64> = OnceLock::new();

/// Process noise for velocity
static PROCESS_NOISE_VELOCITY: OnceLock<f64> = OnceLock::new();

/// Process noise for orientation
static PROCESS_NOISE_ORIENTATION: OnceLock<f64> = OnceLock::new();

/// Process noise for angular velocity
static PROCESS_NOISE_ANGULAR_VEL: OnceLock<f64> = OnceLock::new();

/// State vector type
type StateVec = SVector<f64, STATE_DIM>;

/// Input vector type (just dt)
type InputVec = SVector<f64, INPUT_DIM>;

/// Covariance matrix type
type CovMat = SMatrix<f64, STATE_DIM, STATE_DIM>;

/// Pose measurement type (position + orientation)
type PoseMeasurement = LinearMeasurement<f64, STATE_DIM, POSE_MEAS_DIM>;

/// Velocity measurement type (linear + angular velocity)
type VelocityMeasurement = LinearMeasurement<f64, STATE_DIM, VEL_MEAS_DIM>;

/// MEKF step function: constant velocity motion model for ERROR state
///
/// State: [x, y, z, vx, vy, vz, orientationerrorx, orientationerrory, orientationerrorz, wx, wy, wz]
/// Input: [dt]
///
/// In MEKF, the orientation error state (orientationerrorx/y/z) represents the small rotation
/// from the reference quaternion to the true orientation. During prediction:
/// - The reference quaternion is propagated externally with angular velocity
/// - The error state stays at zero (it's reset after each update)
/// - Only process noise is added to the error covariance

fn step_function(state: StateVec, input: InputVec) -> StepReturn<f64, STATE_DIM> {
    if PROCESS_NOISE_POSITION.get().is_none() {
        eprintln!("Process noise constants not initialized");
        return StepReturn {
            state,
            jacobian: SMatrix::<f64, STATE_DIM, STATE_DIM>::identity(),
            covariance: SMatrix::<f64, STATE_DIM, STATE_DIM>::zeros(),
        };
    }
    let dt = input[0];

    let position = state.fixed_rows::<3>(0);
    let velocity = state.fixed_rows::<3>(3);
    let orientation_error = state.fixed_rows::<3>(6);
    let angular_velocity = state.fixed_rows::<3>(9);

    let mut new_state = StateVec::zeros();

    // Position update
    new_state
        .fixed_rows_mut::<3>(0)
        .copy_from(&(position + velocity * dt));

    // Velocity stays constant
    new_state.fixed_rows_mut::<3>(3).copy_from(&velocity);

    // the error state is reset to zero after each update,
    // and the reference quaternion handles the actual rotation propagation.
    new_state
        .fixed_rows_mut::<3>(6)
        .copy_from(&orientation_error);

    // Angular velocity stays constant
    new_state
        .fixed_rows_mut::<3>(9)
        .copy_from(&angular_velocity);

    let mut jacobian = SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();

    // pos depends on velocity
    jacobian[(0, 3)] = dt;
    jacobian[(1, 4)] = dt;
    jacobian[(2, 5)] = dt;

    let wx = angular_velocity[0];
    let wy = angular_velocity[1];
    let wz = angular_velocity[2];

    jacobian[(6, 7)] = wz * dt;
    jacobian[(6, 8)] = -wy * dt;

    jacobian[(7, 6)] = -wz * dt;
    jacobian[(7, 8)] = wx * dt;

    jacobian[(8, 6)] = wy * dt;
    jacobian[(8, 7)] = -wx * dt;

    let mut covariance = SMatrix::<f64, STATE_DIM, STATE_DIM>::zeros();
    for i in 0..3 {
        covariance[(i, i)] = *PROCESS_NOISE_POSITION.get().unwrap() * dt * dt;
    }
    for i in 3..6 {
        covariance[(i, i)] = *PROCESS_NOISE_VELOCITY.get().unwrap() * dt;
    }
    for i in 6..9 {
        covariance[(i, i)] = *PROCESS_NOISE_ORIENTATION.get().unwrap() * dt * dt;
    }
    for i in 9..12 {
        covariance[(i, i)] = *PROCESS_NOISE_ANGULAR_VEL.get().unwrap() * dt;
    }

    StepReturn {
        state: new_state,
        jacobian,
        covariance,
    }
}

/// Compute the rotation error from reference quaternion to measured quaternion.
/// Returns a small rotation vector (axis-angle) representing: ref * error = measured
/// Therefore: error = ref.inverse() * measured
///
/// In the context of a MEKF: we always compute errors relative to the reference,
/// keeping the error small and avoiding the π singularity.
fn quaternion_error(
    reference: &UnitQuaternion<f64>,
    measured: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let delta = reference.inverse() * measured;
    delta.scaled_axis()
}

/// Compose a small rotation error onto a reference quaternion.
/// Returns: reference * error_rotation
///
/// Used after filter updates to fold the estimated error into the reference.
fn compose_error(reference: &UnitQuaternion<f64>, error: &Vector3<f64>) -> UnitQuaternion<f64> {
    let error_quat = UnitQuaternion::from_scaled_axis(*error);
    reference * error_quat
}

/// Create observation matrix H for pose measurements (position + orientation)
/// Selects indices [0,1,2,6,7,8] from state
fn pose_observation_matrix() -> SMatrix<f64, POSE_MEAS_DIM, STATE_DIM> {
    let mut h = SMatrix::<f64, POSE_MEAS_DIM, STATE_DIM>::zeros();
    // Position: state[0:3] -> measurement[0:3]
    h[(0, 0)] = 1.0;
    h[(1, 1)] = 1.0;
    h[(2, 2)] = 1.0;
    // Orientation: state[6:9] -> measurement[3:6]
    h[(3, 6)] = 1.0;
    h[(4, 7)] = 1.0;
    h[(5, 8)] = 1.0;
    h
}

/// Create observation matrix H for velocity measurements (linear + angular)
/// Selects indices [3,4,5,9,10,11] from state
fn velocity_observation_matrix() -> SMatrix<f64, VEL_MEAS_DIM, STATE_DIM> {
    let mut h = SMatrix::<f64, VEL_MEAS_DIM, STATE_DIM>::zeros();
    // Linear velocity: state[3:6] -> measurement[0:3]
    h[(0, 3)] = 1.0;
    h[(1, 4)] = 1.0;
    h[(2, 5)] = 1.0;
    // Angular velocity: state[9:12] -> measurement[3:6]
    h[(3, 9)] = 1.0;
    h[(4, 10)] = 1.0;
    h[(5, 11)] = 1.0;
    h
}

/// Convert state vector and reference quaternion to Isometry3.
/// The full orientation is: reference_quaternion * error_rotation
fn state_to_isometry(state: &StateVec, reference_quat: &UnitQuaternion<f64>) -> Isometry3<f64> {
    let translation = Vector3::new(state[0], state[1], state[2]);
    let error = Vector3::new(state[6], state[7], state[8]);
    let full_rotation = compose_error(reference_quat, &error);
    Isometry3::from_parts(translation.into(), full_rotation)
}

/// returns the transformation that maps `from` frame to `to` frame
fn compute_frame_offset(from: &Isometry3<f64>, to: &Isometry3<f64>) -> Isometry3<f64> {
    to * from.inverse()
}

pub struct Localizer {
    robot_state: RobotState,
    last_rerun_log: std::time::Instant,

    /// Extended Kalman Filter with 12 error states, 1 input (dt)
    /// State: [pos, vel, orientation_error, angular_vel]
    ekf: EKF<f64, STATE_DIM, INPUT_DIM>,

    /// Reference quaternion for MEKF — the "linearization point" for orientation.
    /// The true orientation is: reference_quaternion * error_rotation
    /// Updated after each filter cycle by composing the error and resetting.
    reference_quaternion: UnitQuaternion<f64>,

    /// Measurement model for pose (position + orientation error)
    pose_measurement: PoseMeasurement,

    /// Measurement model for velocity (linear + angular)
    t265_velocity_measurement: VelocityMeasurement,

    /// Time of most recent update
    most_recent_update: CuTime,

    /// Transformation to convert ICP poses from ICP frame to filter's global frame
    /// Updated when AprilTag is seen
    icp_to_global: Option<Isometry3<f64>>,

    /// Transformation to convert T265 poses from T265 frame to ICP frame
    /// Either identity or computed from T265 to ICP transformation
    /// Re calculated on each ICP measurement
    t265_to_icp: Option<Isometry3<f64>>,

    /// Most recent T265-to-global transformation, useful if only T265 + apriltags are available
    t265_to_global: Option<Isometry3<f64>>,

    /// Most recent ICP pose in raw ICP frame (for computing offset when AprilTag seen)
    last_raw_icp_pose: Option<Isometry3<f64>>,

    /// Previous T265 pose
    prev_t265_pose: Option<Isometry3<f64>>,

    /// Previous T265 timestamp
    prev_t265_time: Option<CuTime>,

    /// Whether we've received our first global reference (AprilTag)
    has_global_reference: bool,
}

impl Localizer {
    /// Reset the orientation error state by composing it onto the reference quaternion.
    /// This is called after each filter update to keep the error small.
    fn reset_orientation_error(&mut self) {
        let state = self.ekf.state();
        let error = Vector3::new(state[6], state[7], state[8]);

        self.reference_quaternion = compose_error(&self.reference_quaternion, &error);

        let state_mut = self.ekf.system.state_mut();
        state_mut[6] = 0.0;
        state_mut[7] = 0.0;
        state_mut[8] = 0.0;
    }

    /// Propagate the reference quaternion forward in time using angular velocity.
    /// Called during the predict step.
    fn propagate_reference_quaternion(&mut self, dt: f64) {
        let state = self.ekf.state();
        let angular_vel = Vector3::new(state[9], state[10], state[11]);

        // Integrate angular velocity: q_new = q * exp(ω * dt / 2)
        // Using scaled_axis which expects the full rotation angle
        let delta_rotation = UnitQuaternion::from_scaled_axis(angular_vel * dt);
        self.reference_quaternion = self.reference_quaternion * delta_rotation;
    }

    /// orientation is the ERROR relative to reference quaternion.
    fn create_pose_measurement(&self, measured_pose: &Isometry3<f64>) -> Vector6<f64> {
        let orient_error = quaternion_error(&self.reference_quaternion, &measured_pose.rotation);
        Vector6::new(
            measured_pose.translation.x,
            measured_pose.translation.y,
            measured_pose.translation.z,
            orient_error.x,
            orient_error.y,
            orient_error.z,
        )
    }
}

impl Freezable for Localizer {}

impl CuTask for Localizer {
    type Input<'m> = input_msg!('m,
        ImuMeasurement,           // Shitty IMU data - ignored for now
        IcpMeasurement,           // Kiss ICP pose
        FromPicoV3,               // Ignored for now
        Vec<AprilTagMeasurement>, // AprilTag detections - global reference
        T265Msg         // T265 pose - for velocity extraction
    );
    type Resources<'r> = ();

    type Output<'m> = output_msg!(FromLunabot);

    fn new(config: Option<&cu29::prelude::ComponentConfig>,_resources: Self::Resources<'_>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        // load up values from config
        let process_noise_position = config
            .unwrap()
            .get::<f64>("process_noise_position").expect("failed to deserialize")
            .expect("please supply process noise position");

        let process_noise_velocity = config
            .unwrap()
            .get::<f64>("process_noise_velocity").expect("failed to deserialize")
            .expect("please supply process noise velocity");
        let process_noise_orientation = config
            .unwrap()
            .get::<f64>("process_noise_orientation").expect("failed to deserialize")
            .expect("please supply process noise orientation");

        let process_noise_angular_vel = config
            .unwrap()
            .get::<f64>("process_noise_angular_velocity").expect("failed to deserialize")
            .expect("please supply process noise angular velocity");

        let initial_covariance = config
            .unwrap()
            .get::<f64>("initial_covariance")
            .expect("failed to deserialize")
            .expect("please supply initial covariance");
        PROCESS_NOISE_POSITION
            .set(process_noise_position).expect("failed to set oncelock");
        PROCESS_NOISE_VELOCITY
            .set(process_noise_velocity).expect("failed to set oncelock");
        PROCESS_NOISE_ORIENTATION
            .set(process_noise_orientation).expect("failed to set oncelock");
        PROCESS_NOISE_ANGULAR_VEL
            .set(process_noise_angular_vel).expect("failed to set oncelock");

        // Initial state: all zeros
        let initial_state = StateVec::zeros();

        // Initial covariance: high uncertainty
        let initial_covariance = CovMat::from_diagonal_element(initial_covariance);

        // Create EKF with our step function
        let ekf = EKF::new_ekf_with_input(step_function, initial_state, initial_covariance);

        // Create pose measurement model
        let pose_h = pose_observation_matrix();
        let pose_r = SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::identity(); // Will be updated per measurement
        let pose_measurement = PoseMeasurement::new(pose_h, pose_r, Vector6::zeros());

        // Create velocity measurement model with T265 noise
        let vel_h = velocity_observation_matrix();
        let vel_r = SMatrix::<f64, VEL_MEAS_DIM, VEL_MEAS_DIM>::identity();

        let t265_velocity_measurement = VelocityMeasurement::new(vel_h, vel_r, Vector6::zeros());

        if let Some(robot_state) = ROBOT_STATE.get() {
            Ok(Self {
                robot_state: robot_state.clone(),
                last_rerun_log: std::time::Instant::now(),
                ekf,
                reference_quaternion: UnitQuaternion::identity(),
                pose_measurement,
                t265_velocity_measurement,

                most_recent_update: CuTime::default(),
                icp_to_global: None,
                t265_to_icp: None,
                t265_to_global: None,
                last_raw_icp_pose: None,
                prev_t265_pose: None,
                prev_t265_time: None,
                has_global_reference: false,
            })
        } else {
            Err(CuError::new_with_cause(
                "no root node found",
                std::io::Error::other("no root node found"),
            ))
        }
    }

    fn start(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
        // Log static geometry for visualization
        if let Some(logger) = RECORDER.get() {
            let axes =
                rerun::Arrows3D::from_vectors([[0.5, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.5]])
                    .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                    .with_labels(vec!["x", "y", "z"]);

            let _ = logger.recorder.log_static("localizer/t265_raw", &axes);
            let _ = logger.recorder.log_static("localizer/icp_raw", &axes);
            let _ = logger.recorder.log_static("localizer/icp_global", &axes);
        }
        Ok(())
    }

    fn process<'i>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'i>,
    ) -> cu29::CuResult<()> {
        let now = clock.now();

        // Predict forward in time
        let dt = ((now - self.most_recent_update).as_nanos() as f64) / 1e9;
        self.most_recent_update = now;

        if dt > 0.0 && dt < 1.0 {
            // MEKF: Propagate reference quaternion with angular velocity
            self.propagate_reference_quaternion(dt);

            // Predict error state (error stays near zero, covariance grows)
            let input_vec = InputVec::new(dt);
            if let Err(e) = self.ekf.predict(input_vec) {
                eprintln!("EKF predict failed: {:?}", e);
            }
        }

        // Process AprilTags - they provide global reference
        // - Stores the ICP-to-global offset
        if let Some(tags) = input.3.payload() {
            for tag in tags {
                if let Some(global_pose) = tag.estimated_isometry.to_na() {
                    // First AprilTag establishes global reference
                    if !self.has_global_reference {
                        self.has_global_reference = true;
                        println!("First AprilTag seen - global reference established");
                    }

                    if let Some(raw_t265) = self.prev_t265_pose {
                        self.t265_to_global = Some(compute_frame_offset(&raw_t265, &global_pose));
                    }

                    // Update ICP-to-global offset if we have a recent ICP pose
                    if let Some(raw_icp) = &self.last_raw_icp_pose {
                        self.icp_to_global = Some(compute_frame_offset(raw_icp, &global_pose));
                        if let Some(logger) = RECORDER.get() {
                            let _ = logger.recorder.log(
                                "kiss_icp",
                                &rerun::Transform3D::from_translation_rotation(
                                    self.icp_to_global
                                        .unwrap()
                                        .translation
                                        .vector
                                        .cast::<f32>()
                                        .data
                                        .0[0],
                                    rerun::Quaternion::from_xyzw(
                                        self.icp_to_global
                                            .unwrap()
                                            .rotation
                                            .as_vector()
                                            .cast::<f32>()
                                            .data
                                            .0[0],
                                    ),
                                ),
                            );
                        }
                    }

                    // MEKF - pose measurement with orientation error relative to reference
                    let pose_vec = self.create_pose_measurement(&global_pose);

                    // Extract variance from AprilTag measurement
                    let mut pose_r =
                        SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&tag.variance);

                    // just in case
                    for i in 0..POSE_MEAS_DIM {
                        if pose_r[(i, i)] < 1e-6 {
                            pose_r[(i, i)] = 1e-6;
                        }
                    }
                    self.pose_measurement.R = pose_r;
                    self.pose_measurement.z = pose_vec;

                    if let Err(e) = self.ekf.update(&self.pose_measurement) {
                        eprintln!("EKF update with AprilTag failed: {:?}", e);
                    }

                    self.reset_orientation_error();

                    if let Some(logger) = RECORDER.get() {
                        let _ = logger.recorder.log(
                            "localizer/apriltag_update",
                            &rerun::TextLog::new(format!(
                                "AprilTag at ({:.2}, {:.2}, {:.2})",
                                global_pose.translation.x,
                                global_pose.translation.y,
                                global_pose.translation.z
                            )),
                        );
                    }
                }
            }
        }

        // Process ICP - apply as pose measurement
        //  - uses offset calculated by AprilTag if available
        //  - updates T265-to-ICP transform for velocity extraction
        if let Some(icp_msg) = input.1.payload() {
            if let Some(raw_icp_pose) = icp_msg.pose.to_na() {
                self.last_raw_icp_pose = Some(raw_icp_pose);

                if let Some(logger) = RECORDER.get() {
                    let _ = logger.recorder.log(
                        "localizer/icp_raw",
                        &rerun::Transform3D::from_translation_rotation(
                            raw_icp_pose.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                raw_icp_pose.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    );
                }
                if let Some(most_recent_t265) = &self.prev_t265_pose {
                    self.t265_to_icp = Some(compute_frame_offset(
                        &most_recent_t265,
                        &(&self.icp_to_global.unwrap_or(Isometry3::identity()) * raw_icp_pose),
                    ));
                }
                let offset = self.icp_to_global.unwrap_or(Isometry3::identity());

                let global_icp_pose = offset * raw_icp_pose;

                if let Some(logger) = RECORDER.get() {
                    let _ = logger.recorder.log(
                        "localizer/icp_global",
                        &rerun::Transform3D::from_translation_rotation(
                            global_icp_pose.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                global_icp_pose.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    );
                }

                let pose_vec = self.create_pose_measurement(&global_icp_pose);

                // get variance from ICP measurement
                let mut pose_r =
                    SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&icp_msg.variance);

                // you never know lmao
                for i in 0..POSE_MEAS_DIM {
                    if pose_r[(i, i)] < 1e-6 {
                        pose_r[(i, i)] = 1e-6;
                    }
                }

                self.pose_measurement.R = pose_r;
                self.pose_measurement.z = pose_vec;

                if let Err(e) = self.ekf.update(&self.pose_measurement) {
                    eprintln!("EKF update with ICP failed: {:?}", e);
                }

                self.reset_orientation_error();
            }
        }

        // Step 4: Process T265 - extract velocities from pose deltas
        //  - readings are transformed to global frame using t265_to_icp and icp_to_global if available
        if let Some(t265_msg) = input.4.payload() {
            if let Some(current_t265_raw) = t265_msg.pose.to_na() {
                if let Some(logger) = RECORDER.get() {
                    let _ = logger.recorder.log(
                        "localizer/t265_raw",
                        &rerun::Transform3D::from_translation_rotation(
                            current_t265_raw.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                current_t265_raw.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    );
                }

                // prioritize using t265 -> icp  transform if available, otherwise use t265 -> global, otherwise identity
                let t265_offset = self
                    .t265_to_icp
                    .unwrap_or(self.t265_to_global.unwrap_or(Isometry3::identity()));

                let current_t265_global = t265_offset * current_t265_raw;

                if let (Some(prev_t265), Some(prev_time)) =
                    (&self.prev_t265_pose, self.prev_t265_time)
                {
                    let prev_t265_global = t265_offset * prev_t265;
                    let delta_time = ((now - prev_time).as_nanos() as f64) / 1e9;

                    // all of the velocities are in GLOBAL frame if self.icp_to_global is set
                    if delta_time > 1e-6 && delta_time < 0.5 {
                        let linear_vel = (current_t265_global.translation.vector
                            - prev_t265_global.translation.vector)
                            / delta_time;

                        let delta_rotation =
                            prev_t265_global.rotation.inverse() * current_t265_global.rotation;

                        let delta_axis_angle = delta_rotation.scaled_axis();
                        let angular_vel = delta_axis_angle / delta_time;

                        // If the t265 has a loop closure over a long distance, the velocities will be huge and wrong.
                        let angular_speed = angular_vel.magnitude();
                        let linear_speed = linear_vel.magnitude();

                        const MAX_ANGULAR_SPEED: f64 = 10.0;
                        const MAX_LINEAR_SPEED: f64 = 10.0;

                        if angular_speed < MAX_ANGULAR_SPEED && linear_speed < MAX_LINEAR_SPEED {
                            let vel_vec = Vector6::new(
                                linear_vel.x,
                                linear_vel.y,
                                linear_vel.z,
                                angular_vel.x,
                                angular_vel.y,
                                angular_vel.z,
                            );

                            self.t265_velocity_measurement.z = vel_vec;

                            let velocity_variance = t265_msg.velocity_variance;
                            let angular_velocity_variance = t265_msg.angular_velocity_variance;
                            let mut vel_r = SMatrix::<f64, VEL_MEAS_DIM, VEL_MEAS_DIM>::zeros();
                            for i in 0..3 {
                                vel_r[(i, i)] = velocity_variance;
                                vel_r[(3 + i, 3 + i)] = angular_velocity_variance;
                            }

                            if let Err(e) = self.ekf.update(&self.t265_velocity_measurement) {
                                eprintln!("EKF update with T265 velocity failed: {:?}", e);
                            }
                            self.reset_orientation_error();
                        } else {
                            eprintln!(
                                "Rejected T265 velocity: linear={:.2} m/s, angular={:.2} rad/s (likely discontinuity, tracking loss, or loop closure event)",
                                linear_speed, angular_speed
                            );
                        }
                    }
                }

                // store the current RAW pose
                // (we apply t265_offset each time so it can be updated by ICP)
                self.prev_t265_pose = Some(current_t265_raw);
                self.prev_t265_time = Some(now);
            }
        }

        if let Some(imu_measurement) = input.0.payload() {
            if let Some(logger) = RECORDER.get() {
                let _ = logger.recorder.log(
                    "imu_corrected",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        imu_measurement.acceleration[0] as f32,
                        imu_measurement.acceleration[1] as f32,
                        imu_measurement.acceleration[2] as f32,
                    )]),
                );
            }

            // apply IMU as velocity measurement
            // gravity has already been subtracted, and it is in robot base frame
            // TODO
        }

        // Output and update robot chain at 60 hz
        if self.last_rerun_log.elapsed().as_millis() > 1000 / 60 {
            self.last_rerun_log = std::time::Instant::now();

            let state = self.ekf.state();
            // MEKF: Get full isometry using reference quaternion + error
            let current_iso = state_to_isometry(state, &self.reference_quaternion);

            self.robot_state.kinematic_root.set_isometry(current_iso);

            self.robot_state.kalman_state.store(Some(*state));
            self.robot_state
                .kalman_variances
                .store(Some(*self.ekf.covariance()));

            if let Some(logger) = RECORDER.get() {
                let state = self.ekf.state();

                let _ = logger.recorder.log(
                    "kalman_state/position",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        state[0] as f32,
                        state[1] as f32,
                        state[2] as f32,
                    )]),
                );

                let _ = logger.recorder.log(
                    "kalman_state/velocity",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        state[3] as f32,
                        state[4] as f32,
                        state[5] as f32,
                    )]),
                );
                let _ = logger.recorder.log(
                    "kalman_state/angular_velocity",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        state[9] as f32,
                        state[10] as f32,
                        state[11] as f32,
                    )]),
                );

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
                }
            }

            output.set_payload(FromLunabot::RobotIsometry {
                origin: current_iso.translation.vector.cast::<f32>().data.0[0],
                quat: current_iso.rotation.as_vector().cast::<f32>().data.0[0],
            });
        }

        Ok(())
    }
}
