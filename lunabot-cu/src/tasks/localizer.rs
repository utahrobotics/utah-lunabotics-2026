use std::collections::HashMap;
use std::sync::OnceLock;

use crate::rerun_viz;
use crate::rerun_viz::RECORDER;
use crate::robot_state::RobotState;
use crate::tasks::AprilTagMeasurement;
use crate::tasks::IcpMeasurement;
use crate::tasks::ImuMeasurement;
use crate::tasks::T265Msg;
use crate::utils::transform_sensor_velocity_to_base;
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
const INPUT_DIM: usize = 1;
const POSE_MEAS_DIM: usize = 6;
const VEL_MEAS_DIM: usize = 6;
// lower = filter trusts prediction more, higher = filter trusts measurements more
static PROCESS_NOISE_POSITION: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_VELOCITY: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_ORIENTATION: OnceLock<f64> = OnceLock::new();
static PROCESS_NOISE_ANGULAR_VEL: OnceLock<f64> = OnceLock::new();

type StateVec = SVector<f64, STATE_DIM>;
type InputVec = SVector<f64, INPUT_DIM>;
type CovMat = SMatrix<f64, STATE_DIM, STATE_DIM>;
type PoseMeasurement = LinearMeasurement<f64, STATE_DIM, POSE_MEAS_DIM>;
type VelocityMeasurement = LinearMeasurement<f64, STATE_DIM, VEL_MEAS_DIM>;

/// Per-sensor state for each T265 tracker
#[derive(Clone)]
struct T265SensorState {
    /// Previous pose in raw sensor frame
    prev_pose: Isometry3<f64>,
    /// Previous timestamp
    prev_time: CuTime,
}

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

    let mut jacobian = SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();

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

fn quaternion_error(
    reference: &UnitQuaternion<f64>,
    measured: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let delta = reference.inverse() * measured;
    delta.scaled_axis()
}

fn compose_error(reference: &UnitQuaternion<f64>, error: &Vector3<f64>) -> UnitQuaternion<f64> {
    let error_quat = UnitQuaternion::from_scaled_axis(*error);
    reference * error_quat
}

fn pose_observation_matrix() -> SMatrix<f64, POSE_MEAS_DIM, STATE_DIM> {
    let mut h = SMatrix::<f64, POSE_MEAS_DIM, STATE_DIM>::zeros();
    h[(0, 0)] = 1.0;
    h[(1, 1)] = 1.0;
    h[(2, 2)] = 1.0;
    h[(3, 6)] = 1.0;
    h[(4, 7)] = 1.0;
    h[(5, 8)] = 1.0;
    h
}

fn velocity_observation_matrix() -> SMatrix<f64, VEL_MEAS_DIM, STATE_DIM> {
    let mut h = SMatrix::<f64, VEL_MEAS_DIM, STATE_DIM>::zeros();
    h[(0, 3)] = 1.0;
    h[(1, 4)] = 1.0;
    h[(2, 5)] = 1.0;
    h[(3, 9)] = 1.0;
    h[(4, 10)] = 1.0;
    h[(5, 11)] = 1.0;
    h
}

fn state_to_isometry(state: &StateVec, reference_quat: &UnitQuaternion<f64>) -> Isometry3<f64> {
    let translation = Vector3::new(state[0], state[1], state[2]);
    let error = Vector3::new(state[6], state[7], state[8]);
    let full_rotation = compose_error(reference_quat, &error);
    Isometry3::from_parts(translation.into(), full_rotation)
}

fn compute_frame_offset(from: &Isometry3<f64>, to: &Isometry3<f64>) -> Isometry3<f64> {
    to * from.inverse()
}

pub struct Localizer {
    robot_state: RobotState,
    last_rerun_log: std::time::Instant,

    ekf: EKF<f64, STATE_DIM, INPUT_DIM>,
    reference_quaternion: UnitQuaternion<f64>,
    pose_measurement: PoseMeasurement,
    t265_velocity_measurement: VelocityMeasurement,
    most_recent_update: CuTime,

    /// ICP-to-global transform
    icp_to_global: Option<Isometry3<f64>>,

    /// Most recent ICP pose in raw ICP frame
    last_raw_icp_pose: Option<Isometry3<f64>>,
    /// Timestamp of the most recent ICP pose
    last_raw_icp_time: CuTime,

    /// Per-sensor state for each T265, keyed by node name
    t265_sensors: HashMap<String, T265SensorState>,

    /// Whether we've received our first global reference (AprilTag)
    has_global_reference: bool,
    /// Max age (seconds) of ICP reading for apriltag offset computation
    icp_apriltag_max_age_s: f64,
}

impl Localizer {
    fn reset_orientation_error(&mut self) {
        let state = self.ekf.state();
        let error = Vector3::new(state[6], state[7], state[8]);
        self.reference_quaternion = compose_error(&self.reference_quaternion, &error);
        let state_mut = self.ekf.system.state_mut();
        state_mut[6] = 0.0;
        state_mut[7] = 0.0;
        state_mut[8] = 0.0;
    }

    fn propagate_reference_quaternion(&mut self, dt: f64) {
        let state = self.ekf.state();
        let angular_vel = Vector3::new(state[9], state[10], state[11]);
        let delta_rotation = UnitQuaternion::from_scaled_axis(angular_vel * dt);
        self.reference_quaternion = self.reference_quaternion * delta_rotation;
    }

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

        let initial_covariance = config
            .unwrap()
            .get::<f64>("initial_covariance")
            .expect("failed to deserialize")
            .expect("please supply initial covariance");

        let icp_apriltag_max_age_ms: f64 = config
            .unwrap()
            .get::<f64>("icp_apriltag_max_age_ms")
            .expect("failed to deserialize")
            .expect("please supply icp_apriltag_max_age_ms");
        let icp_apriltag_max_age_s = icp_apriltag_max_age_ms / 1000.0;

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

        let initial_state = StateVec::zeros();
        let initial_covariance = CovMat::from_diagonal_element(initial_covariance);
        let ekf = EKF::new_ekf_with_input(step_function, initial_state, initial_covariance);

        let pose_h = pose_observation_matrix();
        let pose_r = SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::identity();
        let pose_measurement = PoseMeasurement::new(pose_h, pose_r, Vector6::zeros());

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
                last_raw_icp_pose: None,
                last_raw_icp_time: CuTime::default(),
                t265_sensors: HashMap::new(),
                has_global_reference: false,
                icp_apriltag_max_age_s,
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

        let dt = ((now - self.most_recent_update).as_nanos() as f64) / 1e9;
        self.most_recent_update = now;

        if dt > 0.0 && dt < 1.0 {
            self.propagate_reference_quaternion(dt);
            let input_vec = InputVec::new(dt);
            if let Err(e) = self.ekf.predict(input_vec) {
                eprintln!("EKF predict failed: {:?}", e);
            }
        }

        // Process AprilTags
        if let Some(tags) = input.3.payload() {
            for tag in tags {
                if let Some(global_pose) = tag.estimated_isometry.to_na() {
                    if !self.has_global_reference {
                        self.has_global_reference = true;
                        println!("First AprilTag seen - global reference established");
                    }

                    let pose_vec = self.create_pose_measurement(&global_pose);
                    let mut pose_r =
                        SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&tag.variance);

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

                    // Update ICP-to-global offset using filtered EKF state (not raw apriltag)
                    // Only update when the ICP reading is temporally close (within 100ms)
                    let icp_age_s = ((now - self.last_raw_icp_time).as_nanos() as f64) / 1e9;
                    if let Some(raw_icp) = &self.last_raw_icp_pose {
                        if icp_age_s < self.icp_apriltag_max_age_s {
                            let filtered_state = self.ekf.state();
                            let filtered_pose =
                                state_to_isometry(filtered_state, &self.reference_quaternion);
                            self.icp_to_global =
                                Some(compute_frame_offset(raw_icp, &filtered_pose));
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

        // Process ICP
        if let Some(icp_msg) = input.1.payload() {
            if let Some(raw_icp_pose) = icp_msg.pose.to_na() {
                self.last_raw_icp_pose = Some(raw_icp_pose);
                self.last_raw_icp_time = now;

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
                let mut pose_r =
                    SMatrix::<f64, POSE_MEAS_DIM, POSE_MEAS_DIM>::from_row_slice(&icp_msg.variance);

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

        // if true, the orientation will be reset in the kalman state
        let mut should_reset_orientation = false;
        if let Some(t265_msg) = input.4.payload() {
            if let Some(current_t265_raw) = t265_msg.pose.to_na() {
                let node_name = &t265_msg.node_name;

                if let Some(logger) = RECORDER.get() {
                    let _ = logger.recorder.log(
                        format!("localizer/t265_raw/{}", node_name),
                        &rerun::Transform3D::from_translation_rotation(
                            current_t265_raw.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                current_t265_raw.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    );
                }

                // Get or create sensor state
                let sensor_state =
                    self.t265_sensors
                        .entry(node_name.clone())
                        .or_insert_with(|| T265SensorState {
                            prev_pose: current_t265_raw,
                            prev_time: now,
                        });

                let delta_time = ((now - sensor_state.prev_time).as_nanos() as f64) / 1e9;
                if delta_time > 1e-6 && delta_time < 0.5 {
                    // Compute velocity from RAW T265 sensor readings
                    let sensor_delta = sensor_state.prev_pose.inverse() * current_t265_raw;
                    let sensor_linear_vel = sensor_delta.translation.vector / delta_time;
                    let sensor_angular_vel = sensor_delta.rotation.scaled_axis() / delta_time;

                    // Get the sensor node and transform velocity to base frame
                    if let Some(cam_node) = self
                        .robot_state
                        .kinematic_root
                        .get_node_with_name(node_name)
                    {
                        let base_to_t265 = cam_node.get_isometry_from_base();
                        let (base_linear_vel, base_angular_vel) = transform_sensor_velocity_to_base(
                            sensor_linear_vel,
                            sensor_angular_vel,
                            &base_to_t265,
                        );
                        let state = self.ekf.state();
                        let orientation_error = Vector3::new(state[6], state[7], state[8]);
                        let current_orientation =
                            compose_error(&self.reference_quaternion, &orientation_error);

                        let global_linear_vel = current_orientation * base_linear_vel;
                        let global_angular_vel = current_orientation * base_angular_vel;

                        let angular_speed = global_angular_vel.magnitude();
                        let linear_speed = global_linear_vel.magnitude();

                        const MAX_ANGULAR_SPEED: f64 = 10.0;
                        const MAX_LINEAR_SPEED: f64 = 10.0;

                        if angular_speed < MAX_ANGULAR_SPEED && linear_speed < MAX_LINEAR_SPEED {
                            let vel_vec = Vector6::new(
                                global_linear_vel.x,
                                global_linear_vel.y,
                                global_linear_vel.z,
                                global_angular_vel.x,
                                global_angular_vel.y,
                                global_angular_vel.z,
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
                                eprintln!(
                                    "EKF update with T265 velocity ({}) failed: {:?}",
                                    node_name, e
                                );
                            }
                            should_reset_orientation = true;
                        } else {
                            eprintln!(
                                "Rejected T265 ({}) velocity: linear={:.2} m/s, angular={:.2} rad/s",
                                node_name, linear_speed, angular_speed
                            );
                        }
                    } else {
                        eprintln!(
                            "Warning: T265 node '{}' not found in robot kinematic chain",
                            node_name
                        );
                    }
                }

                // Update sensor state
                sensor_state.prev_pose = current_t265_raw;
                sensor_state.prev_time = now;
            }
        }
        if should_reset_orientation {
            self.reset_orientation_error();
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
        }

        // Output at 60 hz
        if self.last_rerun_log.elapsed().as_millis() > 1000 / 60 {
            self.last_rerun_log = std::time::Instant::now();

            let state = self.ekf.state();
            let current_iso = state_to_isometry(state, &self.reference_quaternion);

            self.robot_state.kinematic_root.set_isometry(current_iso);
            self.robot_state.kalman_state.store(Some(*state));
            self.robot_state
                .kalman_variances
                .store(Some(*self.ekf.covariance()));

            if let Some(logger) = RECORDER.get() {
                let state = self.ekf.state();

                let _ = logger.recorder.log(
                    "localizer/velocity",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(
                        state[3] as f32,
                        state[4] as f32,
                        state[5] as f32,
                    )]),
                );

                let _ = logger.recorder.log(
                    "localizer/angular_velocity",
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
