//! Localizer module for multi-sensor fusion using Multiplicative EKF (MEKF).
//!
//! This module fuses pose estimates from multiple sensors:
//! - **Kiss ICP**: ~5 Hz, accurate but relative to start frame
//! - **T265**: High frequency velocity estimates (derived from pose deltas)
//! - **AprilTags**: Sparse but globally-referenced position fixes
//!
//! # Approach
//!
//! We use a **Multiplicative Extended Kalman Filter (MEKF)** with a 12-state error vector:
//! - Position [x, y, z]
//! - Linear velocity [vx, vy, vz]  
//! - Orientation ERROR [δθx, δθy, δθz] — small rotation from reference quaternion
//! - Angular velocity [wx, wy, wz]
//!
//! The **reference quaternion** is stored separately and updated after each filter cycle.
//! This avoids the singularity at ±π that occurs with axis-angle representations.
//!
//! ## MEKF Operation
//!
//! 1. **Predict**: Propagate reference quaternion with angular velocity, error state gets process noise
//! 2. **Update**: Apply measurements as errors relative to reference quaternion
//! 3. **Reset**: Compose error onto reference quaternion, reset error state to zero
//!
//! This keeps the orientation error small (near zero), completely avoiding the π singularity.
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
use std::sync::OnceLock;

use crate::rerun_viz;
use crate::rerun_viz::RECORDER;
use crate::tasks::AprilTagMeasurement;
use crate::tasks::IcpMeasurement;
use crate::tasks::ImuMeasurement;
use crate::tasks::T265Msg;
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
use kfilter::system::System;
use nalgebra::UnitQuaternion;
use nalgebra::{Isometry3, SMatrix, SVector, Vector3, Vector6};
use rerun::Scalars;

use crate::ROBOT_STATE;

// ============================================================================
// Constants
// ============================================================================

// Higher values = Less trust in the motion model, more responsive to measurements
// Lower values = More trust in the motion model, smoother but slower to correct
/// State dimension: position(3) + velocity(3) + orientation_error(3) + angular_velocity(3)
/// Note: orientation_error is a small rotation vector relative to reference_quaternion
const STATE_DIM: usize = 12;

/// Input dimension: just dt
const INPUT_DIM: usize = 1;

/// Position/orientation measurement dimension
const POSE_MEAS_DIM: usize = 6;

/// Velocity measurement dimension  
const VEL_MEAS_DIM: usize = 6;

/// Process noise for position (grows with time)
static PROCESS_NOISE_POSITION: OnceLock<f64> = OnceLock::new();

/// Process noise for velocity
static PROCESS_NOISE_VELOCITY: OnceLock<f64> = OnceLock::new();

/// Process noise for orientation
static PROCESS_NOISE_ORIENTATION: OnceLock<f64> = OnceLock::new();

/// Process noise for angular velocity
static PROCESS_NOISE_ANGULAR_VEL: OnceLock<f64> = OnceLock::new();

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

/// MEKF step function: constant velocity motion model for ERROR state
///
/// State: [x, y, z, vx, vy, vz, δθx, δθy, δθz, wx, wy, wz]
/// Input: [dt]
///
/// In MEKF, the orientation error state (δθ) represents the small rotation
/// from the reference quaternion to the true orientation. During prediction:
/// - The reference quaternion is propagated externally with angular velocity
/// - The error state stays at zero (it's reset after each update)
/// - Only process noise is added to the error covariance
///
/// Transition:
/// - position += velocity * dt
/// - velocity stays constant
/// - orientation_error stays constant (near zero, propagated via reference quaternion)
/// - angular_velocity stays constant

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

    // Orientation error: In MEKF, the error state is reset to zero after each update,
    // and the reference quaternion handles the actual rotation propagation.
    // Here we just pass through the current error (which should be near zero).
    new_state
        .fixed_rows_mut::<3>(6)
        .copy_from(&orientation_error);

    // Angular velocity stays constant
    new_state
        .fixed_rows_mut::<3>(9)
        .copy_from(&angular_velocity);

    // Jacobian for error-state formulation
    // The error dynamics are: α̇ = -[ω̂×]α (plus noise)
    // where [ω×] is the skew-symmetric cross-product matrix
    let mut jacobian = SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();

    // Position depends on velocity
    jacobian[(0, 3)] = dt;
    jacobian[(1, 4)] = dt;
    jacobian[(2, 5)] = dt;

    // Orientation error dynamics: α̇ = -[ω̂×]α
    // This is the skew-symmetric matrix of angular velocity
    // [ω×] = [  0  -ωz   ωy ]
    //        [ ωz   0   -ωx ]
    //        [-ωy  ωx    0  ]
    // So ∂α̇/∂α = -[ω×], and discretized: I + (-[ω×]) * dt
    let wx = angular_velocity[0];
    let wy = angular_velocity[1];
    let wz = angular_velocity[2];

    // Row 6 (δθx): affected by -(-wz*δθy + wy*δθz) = wz*δθy - wy*δθz
    jacobian[(6, 7)] = wz * dt; // ∂(δθx)/∂(δθy)
    jacobian[(6, 8)] = -wy * dt; // ∂(δθx)/∂(δθz)

    // Row 7 (δθy): affected by -( wz*δθx - wx*δθz) = -wz*δθx + wx*δθz
    jacobian[(7, 6)] = -wz * dt; // ∂(δθy)/∂(δθx)
    jacobian[(7, 8)] = wx * dt; // ∂(δθy)/∂(δθz)

    // Row 8 (δθz): affected by -(-wy*δθx + wx*δθy) = wy*δθx - wx*δθy
    jacobian[(8, 6)] = wy * dt; // ∂(δθz)/∂(δθx)
    jacobian[(8, 7)] = -wx * dt; // ∂(δθz)/∂(δθy)

    // Process covariance
    let mut covariance = SMatrix::<f64, STATE_DIM, STATE_DIM>::zeros();
    for i in 0..3 {
        covariance[(i, i)] = *PROCESS_NOISE_POSITION.get().unwrap() * dt * dt;
    }
    for i in 3..6 {
        covariance[(i, i)] = *PROCESS_NOISE_VELOCITY.get().unwrap() * dt;
    }
    for i in 6..9 {
        // Orientation error grows with angular velocity uncertainty
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
/// This is the core of MEKF: we always compute errors relative to the reference,
/// keeping the error small and avoiding the π singularity.
fn quaternion_error(
    reference: &UnitQuaternion<f64>,
    measured: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let delta = reference.inverse() * measured;
    // scaled_axis() returns the axis-angle vector with magnitude in [0, π]
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

impl Localizer {
    /// Reset the orientation error state by composing it onto the reference quaternion.
    /// This is called after each filter update to keep the error small.
    fn reset_orientation_error(&mut self) {
        let state = self.ekf.state();
        let error = Vector3::new(state[6], state[7], state[8]);

        // Compose error onto reference quaternion
        self.reference_quaternion = compose_error(&self.reference_quaternion, &error);

        // Reset error state to zero
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

    /// Create a pose measurement vector for MEKF.
    /// Position is absolute, orientation is the ERROR relative to reference quaternion.
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

    /// Log the orientation error magnitude BEFORE reset (for debugging MEKF).
    /// This shows the actual correction being applied.
    fn log_error_before_reset(&self, source: &str) {
        if let Some(logger) = RECORDER.get() {
            let state = self.ekf.state();
            let error = Vector3::new(state[6], state[7], state[8]);
            let error_mag = error.magnitude();
            let error_deg = error_mag * 180.0 / PI;

            let _ = logger.recorder.log(
                "debug/error_before_reset",
                &Scalars::new([error_mag as f32]),
            );
            let _ = logger.recorder.log(
                "debug/error_before_reset_deg",
                &Scalars::new([error_deg as f32]),
            );
            let _ = logger.recorder.log(
                "debug/error_source",
                &rerun::TextLog::new(format!(
                    "{}: {:.4} rad ({:.2}°)",
                    source, error_mag, error_deg
                )),
            );
        }
    }
}

impl Freezable for Localizer {}

impl CuTask for Localizer {
    type Input<'m> = input_msg!('m,
        ImuMeasurement,           // Use imu measurement
        IcpMeasurement,           // Kiss ICP pose
        FromPicoV3,               // Ignored for now
        Vec<AprilTagMeasurement>, // AprilTag detections - global reference
        T265Msg         // T265 pose - for velocity extraction
    );

    type Output<'m> = output_msg!(FromLunabot);

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        // load up values from config
        let process_noise_position = config
            .unwrap()
            .get::<f64>("process_noise_position")
            .expect("please supply process noise position");

        let process_noise_velocity = config
            .unwrap()
            .get::<f64>("process_noise_velocity")
            .expect("please supply process noise velocity");
        let process_noise_orientation = config
            .unwrap()
            .get::<f64>("process_noise_orientation")
            .expect("please supply process noise orientation");

        let process_noise_angular_vel = config
            .unwrap()
            .get::<f64>("process_noise_angular_velocity")
            .expect("please supply process noise angular velocity");

        let initial_covariance = config
            .unwrap()
            .get::<f64>("initial_covariance")
            .expect("please supply initial covariance");
        PROCESS_NOISE_POSITION
            .set(process_noise_position)
            .map_err(|e| CuError::new_with_cause("process noise position already set", e))?;
        PROCESS_NOISE_VELOCITY
            .set(process_noise_velocity)
            .map_err(|e| CuError::new_with_cause("process noise velocity already set", e))?;
        PROCESS_NOISE_ORIENTATION
            .set(process_noise_orientation)
            .map_err(|e| CuError::new_with_cause("process noise orientation already set", e))?;
        PROCESS_NOISE_ANGULAR_VEL
            .set(process_noise_angular_vel)
            .map_err(|e| {
                CuError::new_with_cause("process noise angular velocity already set", e)
            })?;

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
            // MEKF: Propagate reference quaternion with angular velocity
            self.propagate_reference_quaternion(dt);

            // Predict error state (error stays near zero, covariance grows)
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

                    // MEKF: Compute pose measurement with orientation error relative to reference
                    let pose_vec = self.create_pose_measurement(&global_pose);

                    // Extract variance from AprilTag measurement
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

                    // Log error BEFORE reset (for debugging)
                    self.log_error_before_reset("AprilTag");

                    // MEKF: Reset orientation error after update
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

                // MEKF: Compute pose measurement with orientation error relative to reference
                let pose_vec = self.create_pose_measurement(&global_icp_pose);

                // Extract variance from ICP measurement
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

                // Log error BEFORE reset (for debugging)
                self.log_error_before_reset("ICP");

                // MEKF: Reset orientation error after update
                self.reset_orientation_error();
            }
        }

        // Step 4: Process T265 - extract velocities from pose deltas IN GLOBAL FRAME
        // Step 4: Process T265 - extract velocities from pose deltas IN GLOBAL FRAME
        if let Some(t265_msg) = input.4.payload() {
            if let Some(current_t265_raw) = t265_msg.pose.to_na() {
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

                            // extract variance from T265 message
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

                            // Log error BEFORE reset (for debugging)
                            self.log_error_before_reset("T265");

                            // MEKF: Reset orientation error after update
                            self.reset_orientation_error();
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
            // MEKF: Get full isometry using reference quaternion + error
            let current_iso = state_to_isometry(state, &self.reference_quaternion);

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
                    // Orientation (9:12 in old format) - use FULL orientation from reference quaternion
                    let full_orient = self.reference_quaternion.scaled_axis();
                    for i in 0..3 {
                        global_state[(9 + i, 0)] = full_orient[i];
                    }
                    // Angular velocity (12:15 in old format, 9:12 in ours)
                    for i in 0..3 {
                        global_state[(12 + i, 0)] = state[9 + i];
                    }
                }
            }

            // Log to rerun
            if let Some(logger) = RECORDER.get() {
                let state = self.ekf.state();
                let cov = self.ekf.covariance();

                // MEKF: Log orientation ERROR magnitude (should stay small, near zero!)
                let error_mag = Vector3::new(state[6], state[7], state[8]).magnitude();
                let _ = logger.recorder.log(
                    "debug/orientation_error_magnitude",
                    &Scalars::new([error_mag as f32]),
                );

                let _ = logger.recorder.log(
                    "debug/orientation_covariance",
                    &Scalars::new([cov[(6, 6)] as f32]),
                );

                // Log FULL orientation as Euler angles (from reference quaternion)
                let euler = self.reference_quaternion.euler_angles();
                let _ = logger.recorder.log(
                    "debug/euler_roll",
                    &Scalars::new([euler.0 as f32 * 180.0 / PI as f32]),
                );
                let _ = logger.recorder.log(
                    "debug/euler_pitch",
                    &Scalars::new([euler.1 as f32 * 180.0 / PI as f32]),
                );
                let _ = logger.recorder.log(
                    "debug/euler_yaw",
                    &Scalars::new([euler.2 as f32 * 180.0 / PI as f32]),
                );
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

                // Log FULL orientation (from reference quaternion, not error state)
                let full_orient = self.reference_quaternion.scaled_axis();
                let _ = logger.recorder.log(
                    "kalman_state/orientation",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        full_orient[0] as f32,
                        full_orient[1] as f32,
                        full_orient[2] as f32,
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
