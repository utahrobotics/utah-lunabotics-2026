use std::sync::OnceLock;

use crate::ROBOT_STATE;
use crate::kalman_filtering::{
    GlobalMeasurementFilter, MEKF, RelativeMeasurementFilter, STATE_DIM_GLOBAL, STATE_DIM_LOCAL,
    quaternion_error,
};
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
use kfilter::measurement::LinearMeasurement;
use kfilter::system::StepReturn;
use nalgebra::{Isometry3, SMatrix, SVector, UnitQuaternion, Vector3, Vector6};

const INPUT_DIM: usize = 1;
const POSE_MEAS_DIM: usize = 6;

// lower = filter trusts prediction more, higher = filter trusts measurements more
static PROCESS_NOISE_POSITION: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_VELOCITY: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_ORIENTATION: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_ANGULAR_VEL: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_ACCELERATION: OnceLock<f64> = OnceLock::new();

type InputVec = SVector<f64, INPUT_DIM>;
type StateVecLocal = SVector<f64, STATE_DIM_LOCAL>;
type CovMatLocal = SMatrix<f64, STATE_DIM_LOCAL, STATE_DIM_LOCAL>;
type StateVecGlobal = SVector<f64, STATE_DIM_GLOBAL>;
type CovMatGlobal = SMatrix<f64, STATE_DIM_GLOBAL, STATE_DIM_GLOBAL>;
type PoseMeasurementLocal = LinearMeasurement<f64, STATE_DIM_LOCAL, POSE_MEAS_DIM>;
type PoseMeasurementGlobal = LinearMeasurement<f64, STATE_DIM_GLOBAL, POSE_MEAS_DIM>;

/// Global filter step function. Constant-velocity model (no acceleration state).
fn step_fn_global(state: StateVecGlobal, input: InputVec) -> StepReturn<f64, STATE_DIM_GLOBAL> {
    if PROCESS_NOISE_POSITION.get().is_none() {
        eprintln!("Process noise constants not initialized");
        return StepReturn {
            state,
            jacobian: SMatrix::<f64, STATE_DIM_GLOBAL, STATE_DIM_GLOBAL>::identity(),
            covariance: SMatrix::<f64, STATE_DIM_GLOBAL, STATE_DIM_GLOBAL>::zeros(),
        };
    }

    let dt = input[0];
    let position = state.fixed_rows::<3>(0);
    let velocity = state.fixed_rows::<3>(3);
    let orientation_error = state.fixed_rows::<3>(6);
    let angular_velocity = state.fixed_rows::<3>(9);

    let mut new_state = StateVecGlobal::zeros();
    new_state
        .fixed_rows_mut::<3>(0)
        .copy_from(&(position + velocity * dt));
    new_state.fixed_rows_mut::<3>(3).copy_from(&velocity);
    new_state
        .fixed_rows_mut::<3>(6)
        .copy_from(&orientation_error);
    new_state
        .fixed_rows_mut::<3>(9)
        .copy_from(&angular_velocity);

    let mut jacobian = SMatrix::<f64, STATE_DIM_GLOBAL, STATE_DIM_GLOBAL>::identity();
    // ∂position/∂velocity = dt
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

    let mut covariance = SMatrix::<f64, STATE_DIM_GLOBAL, STATE_DIM_GLOBAL>::zeros();
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

/// Local filter step function. Constant-acceleration model.
fn step_function_local(state: StateVecLocal, input: InputVec) -> StepReturn<f64, STATE_DIM_LOCAL> {
    if PROCESS_NOISE_POSITION.get().is_none() {
        eprintln!("Process noise constants not initialized");
        return StepReturn {
            state,
            jacobian: SMatrix::<f64, STATE_DIM_LOCAL, STATE_DIM_LOCAL>::identity(),
            covariance: SMatrix::<f64, STATE_DIM_LOCAL, STATE_DIM_LOCAL>::zeros(),
        };
    }
    let dt = input[0];

    let position = state.fixed_rows::<3>(0);
    let velocity = state.fixed_rows::<3>(3);
    let orientation_error = state.fixed_rows::<3>(6);
    let angular_velocity = state.fixed_rows::<3>(9);
    let acceleration = state.fixed_rows::<3>(12);

    let mut new_state = StateVecLocal::zeros();

    new_state
        .fixed_rows_mut::<3>(0)
        .copy_from(&(position + velocity * dt + 0.5 * acceleration * dt * dt));
    new_state
        .fixed_rows_mut::<3>(3)
        .copy_from(&(velocity + acceleration * dt));
    new_state
        .fixed_rows_mut::<3>(6)
        .copy_from(&orientation_error);
    new_state
        .fixed_rows_mut::<3>(9)
        .copy_from(&angular_velocity);
    new_state.fixed_rows_mut::<3>(12).copy_from(&acceleration);

    let mut jacobian = SMatrix::<f64, STATE_DIM_LOCAL, STATE_DIM_LOCAL>::identity();

    jacobian[(0, 3)] = dt;
    jacobian[(1, 4)] = dt;
    jacobian[(2, 5)] = dt;

    // ∂position/∂acceleration = 0.5 * dt²
    jacobian[(0, 12)] = 0.5 * dt * dt;
    jacobian[(1, 13)] = 0.5 * dt * dt;
    jacobian[(2, 14)] = 0.5 * dt * dt;

    // ∂velocity/∂acceleration = dt
    jacobian[(3, 12)] = dt;
    jacobian[(4, 13)] = dt;
    jacobian[(5, 14)] = dt;

    let wx = angular_velocity[0];
    let wy = angular_velocity[1];
    let wz = angular_velocity[2];

    jacobian[(6, 7)] = wz * dt;
    jacobian[(6, 8)] = -wy * dt;
    jacobian[(7, 6)] = -wz * dt;
    jacobian[(7, 8)] = wx * dt;
    jacobian[(8, 6)] = wy * dt;
    jacobian[(8, 7)] = -wx * dt;

    let mut covariance = SMatrix::<f64, STATE_DIM_LOCAL, STATE_DIM_LOCAL>::zeros();
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
    for i in 12..15 {
        covariance[(i, i)] = *PROCESS_NOISE_ACCELERATION.get().unwrap() * dt;
    }

    StepReturn {
        state: new_state,
        jacobian,
        covariance,
    }
}

/// Observation matrix: maps local state [pos(3), vel(3), orient_err(3), ...] to pose measurement [pos(3), orient_err(3)]
fn pose_observation_matrix_local() -> SMatrix<f64, POSE_MEAS_DIM, STATE_DIM_LOCAL> {
    let mut h = SMatrix::<f64, POSE_MEAS_DIM, STATE_DIM_LOCAL>::zeros();
    h[(0, 0)] = 1.0;
    h[(1, 1)] = 1.0;
    h[(2, 2)] = 1.0;
    h[(3, 6)] = 1.0;
    h[(4, 7)] = 1.0;
    h[(5, 8)] = 1.0;
    h
}

/// Observation matrix: maps global state [pos(3), vel(3), orient_err(3), ...] to pose measurement [pos(3), orient_err(3)]
fn pose_observation_matrix_global() -> SMatrix<f64, POSE_MEAS_DIM, STATE_DIM_GLOBAL> {
    let mut h = SMatrix::<f64, POSE_MEAS_DIM, STATE_DIM_GLOBAL>::zeros();
    h[(0, 0)] = 1.0;
    h[(1, 1)] = 1.0;
    h[(2, 2)] = 1.0;
    h[(3, 6)] = 1.0;
    h[(4, 7)] = 1.0;
    h[(5, 8)] = 1.0;
    h
}

/// Create a 6D pose measurement vector (position + orientation error) for an MEKF update.
fn create_pose_measurement_vec(
    reference_quat: &UnitQuaternion<f64>,
    measured_pose: &Isometry3<f64>,
) -> Vector6<f64> {
    let orient_error = quaternion_error(reference_quat, &measured_pose.rotation);
    Vector6::new(
        measured_pose.translation.x,
        measured_pose.translation.y,
        measured_pose.translation.z,
        orient_error.x,
        orient_error.y,
        orient_error.z,
    )
}

pub struct Localizer {
    robot_state: RobotState,
    last_rerun_log: std::time::Instant,

    local_filter: RelativeMeasurementFilter,
    global_filter: GlobalMeasurementFilter,

    pose_measurement_local: PoseMeasurementLocal,
    pose_measurement_global: PoseMeasurementGlobal,

    most_recent_update: CuTime,

    /// Transform from local filter frame to global (world) frame.
    /// Updated when AprilTags are seen.
    local_to_global_offset: Option<Isometry3<f64>>,
}

impl Freezable for Localizer {}

impl CuTask for Localizer {
    type Input<'m> = input_msg!('m,
        ImuMeasurement,
        IcpMeasurement,
        FromPicoV3,
        Vec<AprilTagMeasurement>,
        T265Msg
    );
    type Resources<'r> = ();
    type Output<'m> = output_msg!(FromLunabot);

    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let process_noise_position = config
            .unwrap()
            .get::<f64>("process_noise_position")
            .expect("failed to deserialize")
            .expect("please supply process noise position");

        let process_noise_velocity = config
            .unwrap()
            .get::<f64>("process_noise_velocity")
            .expect("failed to deserialize")
            .expect("please supply process noise velocity");

        let process_noise_orientation = config
            .unwrap()
            .get::<f64>("process_noise_orientation")
            .expect("failed to deserialize")
            .expect("please supply process noise orientation");

        let process_noise_angular_vel = config
            .unwrap()
            .get::<f64>("process_noise_angular_velocity")
            .expect("failed to deserialize")
            .expect("please supply process noise angular velocity");

        let process_noise_acceleration = config
            .unwrap()
            .get::<f64>("process_noise_acceleration")
            .expect("failed to deserialize")
            .expect("please supply process noise acceleration");

        let initial_covariance = config
            .unwrap()
            .get::<f64>("initial_covariance")
            .expect("failed to deserialize")
            .expect("please supply initial covariance");

        PROCESS_NOISE_POSITION
            .set(process_noise_position)
            .expect("failed to set oncelock");
        PROCESS_NOISE_VELOCITY
            .set(process_noise_velocity)
            .expect("failed to set oncelock");
        PROCESS_NOISE_ORIENTATION
            .set(process_noise_orientation)
            .expect("failed to set oncelock");
        PROCESS_NOISE_ANGULAR_VEL
            .set(process_noise_angular_vel)
            .expect("failed to set oncelock");
        PROCESS_NOISE_ACCELERATION
            .set(process_noise_acceleration)
            .expect("failed to set oncelock");

        let local_filter = RelativeMeasurementFilter::new(
            step_function_local,
            StateVecLocal::zeros(),
            CovMatLocal::from_diagonal_element(initial_covariance),
        );

        let global_filter = GlobalMeasurementFilter::new(
            step_fn_global,
            StateVecGlobal::zeros(),
            CovMatGlobal::from_diagonal_element(initial_covariance),
        );

        let pose_h_local = pose_observation_matrix_local();
        let pose_r_local = SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::identity();
        let pose_measurement_local =
            PoseMeasurementLocal::new(pose_h_local, pose_r_local, Vector6::zeros());

        let pose_h_global = pose_observation_matrix_global();
        let pose_r_global = SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::identity();
        let pose_measurement_global =
            PoseMeasurementGlobal::new(pose_h_global, pose_r_global, Vector6::zeros());

        if let Some(robot_state) = ROBOT_STATE.get() {
            Ok(Self {
                robot_state: robot_state.clone(),
                last_rerun_log: std::time::Instant::now(),
                local_filter,
                global_filter,
                pose_measurement_local,
                pose_measurement_global,
                most_recent_update: CuTime::default(),
                local_to_global_offset: None,
            })
        } else {
            Err(CuError::new_with_cause(
                "no root node found",
                std::io::Error::other("no root node found"),
            ))
        }
    }

    fn start(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
        if let Some(logger) = RECORDER.get() {
            let axes =
                rerun::Arrows3D::from_vectors([[0.5, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.5]])
                    .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                    .with_labels(vec!["x", "y", "z"]);

            let _ = logger
                .recorder
                .log_static("localizer/robot_base_seen_by_t265", &axes);
            let _ = logger.recorder.log_static("localizer/icp_raw", &axes);
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

        let dt = ((now - self.most_recent_update).as_nanos() as f64) / 1e9;
        self.most_recent_update = now;

        // Predict both filters
        if dt > 0.0 && dt < 1.0 {
            let input_vec = InputVec::new(dt);
            if let Err(e) = self.local_filter.predict(input_vec) {
                eprintln!("Local filter predict failed: {:?}", e);
            }
            if let Err(e) = self.global_filter.predict(input_vec) {
                eprintln!("Global filter predict failed: {:?}", e);
            }
        }

        // Process T265 -> local filter (direct pose measurement, transformed to base frame)
        if let Some(t265_msg) = input.4.payload() {
            if let Some(robot_base_seen_by_t265) = t265_msg.pose.to_na() {
                let node_name = &t265_msg.node_name;

                let pose_vec = create_pose_measurement_vec(
                    self.local_filter.reference_quaternion(),
                    &robot_base_seen_by_t265,
                );

                let mut pose_r = SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::zeros();
                for i in 0..3 {
                    pose_r[(i, i)] = t265_msg.pose_variance;
                }
                for i in 3..6 {
                    pose_r[(i, i)] = t265_msg.pose_variance;
                }

                self.pose_measurement_local.R = pose_r;
                self.pose_measurement_local.z = pose_vec;

                if let Err(e) = self.local_filter.update(&self.pose_measurement_local) {
                    eprintln!(
                        "Local filter update with T265 ({}) failed: {:?}",
                        node_name, e
                    );
                }
            }
        }

        // Process ICP -> local filter (direct pose measurement in local frame)
        if let Some(icp_msg) = input.1.payload() {
            if let Some(raw_icp_pose) = icp_msg.pose.to_na() {
                let pose_vec = create_pose_measurement_vec(
                    self.local_filter.reference_quaternion(),
                    &raw_icp_pose,
                );

                let mut pose_r =
                    SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&icp_msg.variance);
                for i in 0..POSE_MEAS_DIM {
                    if pose_r[(i, i)] < 1e-6 {
                        pose_r[(i, i)] = 1e-6;
                    }
                }

                self.pose_measurement_local.R = pose_r;
                self.pose_measurement_local.z = pose_vec;

                if let Err(e) = self.local_filter.update(&self.pose_measurement_local) {
                    eprintln!("Local filter update with ICP failed: {:?}", e);
                }
            }
        }

        // Process AprilTags -> global filter, then compute local-to-global offset
        if let Some(tags) = input.3.payload() {
            for tag in tags {
                if let Some(global_pose) = tag.estimated_isometry.to_na() {
                    let pose_vec = create_pose_measurement_vec(
                        self.global_filter.reference_quaternion(),
                        &global_pose,
                    );

                    let mut pose_r =
                        SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&tag.variance);
                    for i in 0..POSE_MEAS_DIM {
                        if pose_r[(i, i)] < 1e-6 {
                            pose_r[(i, i)] = 1e-6;
                        }
                    }

                    self.pose_measurement_global.R = pose_r;
                    self.pose_measurement_global.z = pose_vec;

                    if let Err(e) = self.global_filter.update(&self.pose_measurement_global) {
                        eprintln!("Global filter update with AprilTag failed: {:?}", e);
                    }

                    // Compute offset: local_to_global = global_pose * inv(local_pose)
                    let global_iso = self.global_filter.state_to_isometry();
                    let local_iso = self.local_filter.state_to_isometry();
                    self.local_to_global_offset = Some(global_iso * local_iso.inverse());
                }
            }
        }

        // Output at 60 hz
        if self.last_rerun_log.elapsed().as_millis() > 1000 / 60 {
            self.last_rerun_log = std::time::Instant::now();

            let local_iso = self.local_filter.state_to_isometry();
            let current_iso = match &self.local_to_global_offset {
                Some(offset) => offset * local_iso,
                None => local_iso,
            };

            self.robot_state.kinematic_root.set_isometry(current_iso);

            let local_state = self.local_filter.state();
            self.robot_state.kalman_state.store(Some(*local_state));
            self.robot_state
                .kalman_variances
                .store(Some(*self.local_filter.covariance()));

            if let Some(logger) = RECORDER.get() {
                let _ = logger.recorder.log(
                    "localizer/velocity",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        local_state[3] as f32,
                        local_state[4] as f32,
                        local_state[5] as f32,
                    )]),
                );

                let _ = logger.recorder.log(
                    "localizer/angular_velocity",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        local_state[9] as f32,
                        local_state[10] as f32,
                        local_state[11] as f32,
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
