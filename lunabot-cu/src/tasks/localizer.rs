//! Localizer module for multi-sensor fusion using Kalman filtering.
//!
//! This module fuses pose estimates from multiple sensors:
//! - **Kiss ICP**: ~5 Hz, accurate but relative to start frame
//! - **T265**: High frequency velocity estimates (derived from pose deltas)
//! - **AprilTags**: Sparse but globally-referenced position fixes
//!
//! # Approach
//!
//! We use the kfilter library with a 12-state Extended Kalman Filter:
//! - Position [x, y, z]
//! - Linear velocity [vx, vy, vz]  
//! - Orientation (axis-angle) [rx, ry, rz]
//! - Angular velocity [wx, wy, wz]
//!
//! ## Handling Relative vs Global Frames
//!
//! Both ICP and T265 provide poses relative to their own start frames, which drift
//! from global. Our strategy:
//!
//! 1. **T265**: Transform to global frame, then extract velocities in global coordinates
//! 2. **ICP**: Track an `icp_to_global` offset, updated when AprilTag is seen
//! 3. **AprilTag**: Provides direct global position measurements
//!
//! When an AprilTag is detected, we:
//! 1. Apply it as a high-confidence position/orientation measurement
//! 2. Update the ICP offset: `icp_to_global = apriltag_pose * icp_pose.inverse()`
//!
//! This naturally handles drift: the Kalman filter weights measurements by their
//! covariance, so AprilTag's low-covariance measurements pull the estimate
//! toward ground truth.
//!
//! ## Tuning workflow:
//! Symptom                              Fix
//! Estimate is too noisy/jittery        Increase process noise OR decrease measurement variances
//! Estimate lags behind reality         Decrease measurement variances (trust sensors more)
//! Jumps when AprilTag seen             Increase AprilTag variance (less aggressive correction)
//! Doesn't correct drift fast enough    Decrease AprilTag variance OR increase process noise
//! T265 velocity making it unstable     Increase T265_VELOCITY_VARIANCE
//! ICP not contributing enough          Decrease ICP variance in kiss_icp.rs

use std::f64::consts::PI;

use crate::rerun_viz;
use crate::rerun_viz::RECORDER;
use crate::tasks::AprilTagMeasurement;
use crate::tasks::IcpMeasurement;
use crate::tasks::ImuMeasurement;
use crate::utils::RobotState;
use common::FromLunabot;
use cu_spatial_payloads::EncodableIsometry;
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
use nalgebra::UnitQuaternion;
use nalgebra::{Isometry3, SMatrix, SVector, Vector3, Vector6};

use crate::ROBOT_STATE;

// ============================================================================
// Constants
// ============================================================================

// Higher values = Less trust in the motion model, more responsive to measurements
// Lower values = More trust in the motion model, smoother but slower to correct
/// State dimension: position(3) + velocity(3) + orientation(3) + angular_velocity(3)
const STATE_DIM: usize = 12;

/// Input dimension: just dt
const INPUT_DIM: usize = 1;

/// Position/orientation measurement dimension
const POSE_MEAS_DIM: usize = 6;

/// Velocity measurement dimension  
const VEL_MEAS_DIM: usize = 6;

/// Process noise for position (grows with time)
const PROCESS_NOISE_POSITION: f64 = 0.4;

/// Process noise for velocity
const PROCESS_NOISE_VELOCITY: f64 = 0.4;

/// Process noise for orientation
const PROCESS_NOISE_ORIENTATION: f64 = 0.4;

/// Process noise for angular velocity
const PROCESS_NOISE_ANGULAR_VEL: f64 = 0.4;

/// T265 velocity measurement variance (T265 is precise for short-term motion)
const T265_VELOCITY_VARIANCE: f64 = 0.5;

/// T265 angular velocity measurement variance
const T265_ANGULAR_VELOCITY_VARIANCE: f64 = 0.5;

/// IMU linear acceleration measurement variance (IMU is noisy but high frequency)
const IMU_LINEAR_ACCELERATION_VARIANCE: f64 = 0.1;

/// IMU angular velocity measurement variance
const IMU_ANGULAR_VELOCITY_VARIANCE: f64 = 0.1;

/// Initial covariance for unknown states
const INITIAL_COVARIANCE: f64 = 100.0;

// ============================================================================
// Type Aliases
// ============================================================================

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

// ============================================================================
// Helper Functions
// ============================================================================

/// EKF step function: constant velocity motion model
///
/// State: [x, y, z, vx, vy, vz, rx, ry, rz, wx, wy, wz]
/// Input: [dt]
///
/// Transition:
/// - position += velocity * dt
/// - velocity stays constant
/// - orientation += angular_velocity * dt (with wrapping)
/// - angular_velocity stays constant

fn step_function(state: StateVec, input: InputVec) -> StepReturn<f64, STATE_DIM> {
    let dt = input[0];

    let position = state.fixed_rows::<3>(0);
    let velocity = state.fixed_rows::<3>(3);
    let orientation = state.fixed_rows::<3>(6);
    let angular_velocity = state.fixed_rows::<3>(9);

    let mut new_state = StateVec::zeros();

    // Position update
    new_state
        .fixed_rows_mut::<3>(0)
        .copy_from(&(position + velocity * dt));

    // Velocity stays constant
    new_state.fixed_rows_mut::<3>(3).copy_from(&velocity);

    // Orientation update with PROPER normalization
    let raw_new_orientation = Vector3::new(
        orientation[0] + angular_velocity[0] * dt,
        orientation[1] + angular_velocity[1] * dt,
        orientation[2] + angular_velocity[2] * dt,
    );
    // Normalize to prevent unbounded growth and keep representation consistent
    let normalized_orientation = normalize_axis_angle(raw_new_orientation);
    new_state
        .fixed_rows_mut::<3>(6)
        .copy_from(&normalized_orientation);

    // Angular velocity stays constant
    new_state
        .fixed_rows_mut::<3>(9)
        .copy_from(&angular_velocity);

    // Jacobian (unchanged)
    let mut jacobian = SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();
    jacobian[(0, 3)] = dt;
    jacobian[(1, 4)] = dt;
    jacobian[(2, 5)] = dt;
    jacobian[(6, 9)] = dt;
    jacobian[(7, 10)] = dt;
    jacobian[(8, 11)] = dt;

    // Process covariance (unchanged)
    let mut covariance = SMatrix::<f64, STATE_DIM, STATE_DIM>::zeros();
    for i in 0..3 {
        covariance[(i, i)] = PROCESS_NOISE_POSITION * dt * dt;
    }
    for i in 3..6 {
        covariance[(i, i)] = PROCESS_NOISE_VELOCITY * dt;
    }
    for i in 6..9 {
        covariance[(i, i)] = PROCESS_NOISE_ORIENTATION * dt * dt;
    }
    for i in 9..12 {
        covariance[(i, i)] = PROCESS_NOISE_ANGULAR_VEL * dt;
    }

    StepReturn {
        state: new_state,
        jacobian,
        covariance,
    }
}

/// Compute the shortest-path rotation from one orientation to another.
/// Returns the axis-angle vector representing the rotation that takes `from` to `to`.
fn orientation_difference(from: &Vector3<f64>, to: &Vector3<f64>) -> Vector3<f64> {
    let from_q = UnitQuaternion::from_scaled_axis(*from);
    let to_q = UnitQuaternion::from_scaled_axis(*to);

    // Compute: delta such that from * delta = to
    // Therefore: delta = from.inverse() * to
    let delta = from_q.inverse() * to_q;

    // scaled_axis() returns the axis-angle, which is guaranteed to be
    // the shortest path (magnitude in [0, π])
    delta.scaled_axis()
}

/// Normalize axis-angle to keep magnitude in [0, π] with consistent axis direction.
/// This prevents the magnitude from growing unboundedly during prediction.
fn normalize_axis_angle(v: Vector3<f64>) -> Vector3<f64> {
    let mag = v.magnitude();
    if mag < 1e-10 {
        return Vector3::zeros();
    }

    // Use quaternion round-trip to get canonical representation
    let q = UnitQuaternion::from_scaled_axis(v);
    q.scaled_axis()
}

/// Adjust a pose measurement so that its orientation component will produce
/// the correct (shortest-path) innovation when subtracted from the state.
///
/// The Kalman filter computes innovation as (z - H*x). For orientation,
/// we need to ensure this gives the shortest angular path, not a 350° spin.
fn adjust_pose_for_orientation_continuity(
    raw_pose: &Vector6<f64>,
    current_state: &StateVec,
) -> Vector6<f64> {
    let meas_orient = Vector3::new(raw_pose[3], raw_pose[4], raw_pose[5]);
    let state_orient = Vector3::new(current_state[6], current_state[7], current_state[8]);

    // Get the proper (shortest-path) orientation difference
    let orient_diff = orientation_difference(&state_orient, &meas_orient);

    // Adjust measurement so that: adjusted - state = orient_diff
    // Therefore: adjusted = state + orient_diff
    let adjusted_orient = state_orient + orient_diff;

    Vector6::new(
        raw_pose[0],
        raw_pose[1],
        raw_pose[2],
        adjusted_orient[0],
        adjusted_orient[1],
        adjusted_orient[2],
    )
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

/// Convert an Isometry3 to a 6D pose vector [x, y, z, rx, ry, rz]
fn isometry_to_pose_vec(iso: &Isometry3<f64>) -> Vector6<f64> {
    let rot = iso.rotation.scaled_axis();
    Vector6::new(
        iso.translation.x,
        iso.translation.y,
        iso.translation.z,
        rot.x,
        rot.y,
        rot.z,
    )
}

/// Convert state vector to Isometry3
fn state_to_isometry(state: &StateVec) -> Isometry3<f64> {
    let translation = Vector3::new(state[0], state[1], state[2]);
    let rotation = Vector3::new(state[6], state[7], state[8]);
    Isometry3::new(translation, rotation)
}

/// Compute the transformation that maps `from` frame to `to` frame
/// Usage: `result * pose_in_from_frame = pose_in_to_frame`
fn compute_frame_offset(from: &Isometry3<f64>, to: &Isometry3<f64>) -> Isometry3<f64> {
    to * from.inverse()
}

// ============================================================================
// Localizer Struct
// ============================================================================

pub struct Localizer {
    robot_state: RobotState,
    last_rerun_log: std::time::Instant,

    /// Extended Kalman Filter with 12 states, 1 input (dt)
    ekf: EKF<f64, STATE_DIM, INPUT_DIM>,

    /// Measurement model for pose (position + orientation)
    pose_measurement: PoseMeasurement,

    /// Measurement model for velocity (linear + angular)
    t265_velocity_measurement: VelocityMeasurement,

    /// Measurement model for velocity from IMU (linear acceleration + angular velocity)
    imu_velocity_measurement: VelocityMeasurement,

    /// Time of most recent update
    most_recent_update: CuTime,

    /// Transformation to convert ICP poses from ICP frame to filter's global frame
    /// Updated when AprilTag is seen
    icp_to_global: Option<Isometry3<f64>>,

    /// Transformation to convert T265 poses from T265 frame to ICP frame
    /// Either identity or computed from T265 to ICP transformation
    /// Re calculated on each ICP measurement
    t265_to_icp: Isometry3<f64>,

    /// Most recent ICP pose in raw ICP frame (for computing offset when AprilTag seen)
    last_raw_icp_pose: Option<Isometry3<f64>>,

    /// Previous T265 pose
    prev_t265_pose: Option<Isometry3<f64>>,

    /// Previous T265 timestamp
    prev_t265_time: Option<CuTime>,

    /// Whether we've received our first global reference (AprilTag)
    has_global_reference: bool,
}

impl Freezable for Localizer {}

impl CuTask for Localizer {
    type Input<'m> = input_msg!('m,
        ImuMeasurement,           // Use imu measurement
        IcpMeasurement,           // Kiss ICP pose
        FromPicoV3,               // Ignored for now
        Vec<AprilTagMeasurement>, // AprilTag detections - global reference
        EncodableIsometry         // T265 pose - for velocity extraction
    );

    type Output<'m> = output_msg!(FromLunabot);

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        // Initial state: all zeros
        let initial_state = StateVec::zeros();

        // Initial covariance: high uncertainty
        let initial_covariance = CovMat::from_diagonal_element(INITIAL_COVARIANCE);

        // Create EKF with our step function
        let ekf = EKF::new_ekf_with_input(step_function, initial_state, initial_covariance);

        // Create pose measurement model
        let pose_h = pose_observation_matrix();
        let pose_r = SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::identity(); // Will be updated per measurement
        let pose_measurement = PoseMeasurement::new(pose_h, pose_r, Vector6::zeros());

        // Create velocity measurement model with T265 noise
        let vel_h = velocity_observation_matrix();
        let mut vel_r = SMatrix::<f64, VEL_MEAS_DIM, VEL_MEAS_DIM>::identity();
        for i in 0..3 {
            vel_r[(i, i)] = T265_VELOCITY_VARIANCE;
        }
        for i in 3..6 {
            vel_r[(i, i)] = T265_ANGULAR_VELOCITY_VARIANCE;
        }
        let t265_velocity_measurement = VelocityMeasurement::new(vel_h, vel_r, Vector6::zeros());

        let imu_vel_h = velocity_observation_matrix();
        let mut imu_vel_r = SMatrix::<f64, VEL_MEAS_DIM, VEL_MEAS_DIM>::identity();
        for i in 0..3 {
            imu_vel_r[(i, i)] = IMU_LINEAR_ACCELERATION_VARIANCE;
        }
        for i in 3..6 {
            imu_vel_r[(i, i)] = IMU_ANGULAR_VELOCITY_VARIANCE;
        }

        if let Some(robot_state) = ROBOT_STATE.get() {
            Ok(Self {
                robot_state: robot_state.clone(),
                last_rerun_log: std::time::Instant::now(),
                ekf,
                pose_measurement,
                t265_velocity_measurement,
                imu_velocity_measurement: VelocityMeasurement::new(
                    imu_vel_h,
                    imu_vel_r,
                    Vector6::zeros(),
                ),
                most_recent_update: CuTime::default(),
                icp_to_global: None,
                t265_to_icp: Isometry3::identity(),
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
            let _ = logger.recorder.log_static("localizer/estimated", &axes);
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

        // Step 1: Predict forward in time
        let dt = ((now - self.most_recent_update).as_nanos() as f64) / 1e9;
        self.most_recent_update = now;

        if dt > 0.0 && dt < 1.0 {
            let input_vec = InputVec::new(dt);
            if let Err(e) = self.ekf.predict(input_vec) {
                eprintln!("EKF predict failed: {:?}", e);
            }
        }

        // Step 2: Process AprilTags FIRST - they provide global reference
        if let Some(tags) = input.3.payload() {
            for tag in tags {
                if let Some(global_pose) = tag.estimated_isometry.to_na() {
                    // First AprilTag establishes global reference
                    if !self.has_global_reference {
                        self.has_global_reference = true;
                        println!("First AprilTag seen - global reference established");
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
                        println!("ICP-to-global offset updated from AprilTag");
                    }

                    // Apply AprilTag as high-confidence pose measurement
                    let pose_vec = isometry_to_pose_vec(&global_pose);

                    let pose_vec =
                        adjust_pose_for_orientation_continuity(&pose_vec, self.ekf.state());

                    // Extract covariance from AprilTag measurement
                    let mut pose_r =
                        SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&tag.variance);

                    // Ensure covariance is positive definite (add small diagonal if needed)
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

        // Step 3: Process ICP - apply as pose measurement (with offset if available)
        if let Some(icp_msg) = input.1.payload() {
            if let Some(raw_icp_pose) = icp_msg.pose.to_na() {
                // Store raw pose for computing offset when AprilTag is seen
                self.last_raw_icp_pose = Some(raw_icp_pose);

                // Visualize raw ICP
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
                    // Compute transformation from T265 frame to ICP frame
                    self.t265_to_icp = compute_frame_offset(
                        &most_recent_t265,
                        &(&self.icp_to_global.unwrap_or(Isometry3::identity()) * raw_icp_pose),
                    );
                }
                let offset = self.icp_to_global.unwrap_or(Isometry3::identity());

                // Only apply ICP measurement if we have an offset (from AprilTag)
                let global_icp_pose = offset * raw_icp_pose;

                // Visualize corrected ICP
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

                // Apply as pose measurement
                let pose_vec = adjust_pose_for_orientation_continuity(
                    &isometry_to_pose_vec(&global_icp_pose),
                    self.ekf.state(),
                );

                // Extract covariance from ICP measurement
                let mut pose_r =
                    SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&icp_msg.variance);

                // Ensure positive definite
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
            }
        }

        // Step 4: Process T265 - extract velocities from pose deltas IN GLOBAL FRAME
        // Step 4: Process T265 - extract velocities from pose deltas IN GLOBAL FRAME
        if let Some(t265_msg) = input.4.payload() {
            if let Some(current_t265_raw) = t265_msg.to_na() {
                // Visualize raw T265
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

                let t265_offset = self.t265_to_icp;

                // Transform current T265 to global frame
                let current_t265_global = t265_offset * current_t265_raw;

                // If we have a previous pose, compute velocity
                if let (Some(prev_t265), Some(prev_time)) =
                    (&self.prev_t265_pose, self.prev_t265_time)
                {
                    let prev_t265_global = t265_offset * prev_t265;
                    let delta_time = ((now - prev_time).as_nanos() as f64) / 1e9;

                    if delta_time > 1e-6 && delta_time < 0.5 {
                        // Linear velocity in global coordinates
                        let linear_vel = (current_t265_global.translation.vector
                            - prev_t265_global.translation.vector)
                            / delta_time;

                        // Angular velocity in global coordinates
                        // Use quaternion multiplication to get the rotation delta
                        // delta_rotation represents: prev * delta = current, so delta = prev.inv * current
                        let delta_rotation =
                            prev_t265_global.rotation.inverse() * current_t265_global.rotation;

                        // scaled_axis() returns axis-angle with magnitude in [0, π]
                        // This is the shortest-path rotation, avoiding the discontinuity
                        let delta_axis_angle = delta_rotation.scaled_axis();
                        let angular_vel = delta_axis_angle / delta_time;

                        // Sanity check: reject impossibly high angular velocities
                        // These can occur from:
                        // - T265 tracking loss/reset
                        // - Frame synchronization issues
                        // - Numerical issues near ±π boundary
                        // 10 rad/s ≈ 573°/s, which is extremely fast for a ground robot
                        let angular_speed = angular_vel.magnitude();
                        let linear_speed = linear_vel.magnitude();

                        // Also sanity check linear velocity (e.g., 10 m/s is very fast for our robot)
                        const MAX_ANGULAR_SPEED: f64 = 10.0; // rad/s
                        const MAX_LINEAR_SPEED: f64 = 10.0; // m/s

                        if angular_speed < MAX_ANGULAR_SPEED && linear_speed < MAX_LINEAR_SPEED {
                            // Create velocity measurement in GLOBAL frame
                            let vel_vec = Vector6::new(
                                linear_vel.x,
                                linear_vel.y,
                                linear_vel.z,
                                angular_vel.x,
                                angular_vel.y,
                                angular_vel.z,
                            );

                            self.t265_velocity_measurement.z = vel_vec;

                            if let Err(e) = self.ekf.update(&self.t265_velocity_measurement) {
                                eprintln!("EKF update with T265 velocity failed: {:?}", e);
                            }
                        } else {
                            // Log rejected velocity for debugging
                            eprintln!(
                                "Rejected T265 velocity: linear={:.2} m/s, angular={:.2} rad/s (likely discontinuity or tracking loss)",
                                linear_speed, angular_speed
                            );
                        }
                    }
                }

                // Store current RAW pose for next iteration
                // (we apply t265_offset each time so it can be updated by ICP)
                self.prev_t265_pose = Some(current_t265_raw);
                self.prev_t265_time = Some(now);
            }
        }

        // Step 5: Log IMU for debugging (not used for estimation)
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
        }

        // Step 6: Output at 60 Hz
        if self.last_rerun_log.elapsed().as_millis() > 1000 / 60 {
            self.last_rerun_log = std::time::Instant::now();

            let state = self.ekf.state();
            let current_iso = state_to_isometry(state);

            // Update shared robot state
            self.robot_state.kinematic_root.set_isometry(current_iso);

            // Update shared kalman state/covariance (convert 12-state to 15-state format for compatibility)
            {
                let mut global_variance_lock = self.robot_state.kalman_variances.write().unwrap();
                if let Some(global_variance) = global_variance_lock.as_mut() {
                    // Clear and fill with our 12-state covariance in the appropriate slots
                    global_variance.fill(0.0);
                    let cov = self.ekf.covariance();
                    // Position (0:3)
                    for i in 0..3 {
                        for j in 0..3 {
                            global_variance[(i, j)] = cov[(i, j)];
                        }
                    }
                    // Velocity (3:6)
                    for i in 0..3 {
                        for j in 0..3 {
                            global_variance[(3 + i, 3 + j)] = cov[(3 + i, 3 + j)];
                        }
                    }
                    // Orientation (9:12 in old format, 6:9 in our format)
                    for i in 0..3 {
                        for j in 0..3 {
                            global_variance[(9 + i, 9 + j)] = cov[(6 + i, 6 + j)];
                        }
                    }
                    // Angular velocity (12:15 in old format, 9:12 in our format)
                    for i in 0..3 {
                        for j in 0..3 {
                            global_variance[(12 + i, 12 + j)] = cov[(9 + i, 9 + j)];
                        }
                    }
                }
            }

            {
                let mut global_state_lock = self.robot_state.kalman_state.write().unwrap();
                if let Some(global_state) = global_state_lock.as_mut() {
                    // Convert 12-state to 15-state format
                    global_state.fill(0.0);
                    // Position (0:3)
                    for i in 0..3 {
                        global_state[(i, 0)] = state[i];
                    }
                    // Velocity (3:6)
                    for i in 0..3 {
                        global_state[(3 + i, 0)] = state[3 + i];
                    }
                    // Acceleration (6:9) - we don't track this, leave as zero
                    // Orientation (9:12 in old format, 6:9 in ours)
                    for i in 0..3 {
                        global_state[(9 + i, 0)] = state[6 + i];
                    }
                    // Angular velocity (12:15 in old format, 9:12 in ours)
                    for i in 0..3 {
                        global_state[(12 + i, 0)] = state[9 + i];
                    }
                }
            }

            // Log to rerun
            if let Some(logger) = RECORDER.get() {
                let _ = logger.recorder.log(
                    "kalman_state/covariance",
                    &rerun::Tensor::new(self.ekf.covariance().data.as_slice()),
                );

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
                    "kalman_state/orientation",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        state[6] as f32,
                        state[7] as f32,
                        state[8] as f32,
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

                let _ = logger.recorder.log(
                    "localizer/estimated",
                    &rerun::Transform3D::from_translation_rotation(
                        current_iso.translation.vector.cast::<f32>().data.0[0],
                        rerun::Quaternion::from_xyzw(
                            current_iso.rotation.as_vector().cast::<f32>().data.0[0],
                        ),
                    ),
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
