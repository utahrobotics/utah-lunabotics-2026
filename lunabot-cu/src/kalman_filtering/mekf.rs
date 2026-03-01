use kfilter::kalman::{EKF, KalmanFilter, KalmanPredictInput, KalmanUpdate, KfError};
use kfilter::measurement::LinearMeasurement;
use kfilter::system::{StepReturn, System};
use nalgebra::{Isometry3, SMatrix, SVector, UnitQuaternion, Vector3};

/// Compose a reference quaternion with an orientation error (rotation vector).
pub fn compose_error(reference: &UnitQuaternion<f64>, error: &Vector3<f64>) -> UnitQuaternion<f64> {
    let error_quat = UnitQuaternion::from_scaled_axis(*error);
    reference * error_quat
}

/// Compute the orientation error between a reference and measured quaternion as a rotation vector.
pub fn quaternion_error(
    reference: &UnitQuaternion<f64>,
    measured: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let delta = reference.inverse() * measured;
    delta.scaled_axis()
}

pub const STATE_DIM_LOCAL: usize = 15;
pub const STATE_DIM_GLOBAL: usize = 12;

/// Trait for Multiplicative Extended Kalman Filters.
///
/// Tracks orientation using a reference quaternion plus a small rotation-vector
/// error in the state. After each update, the error is folded into the reference
/// quaternion and zeroed in the state vector.
pub trait MEKF<const STATE_DIM: usize, const INPUT_DIM: usize> {
    const ANGULAR_VEL_INDEX: usize;
    const ORIENTATION_ERR_INDEX: usize;

    fn ekf(&self) -> &EKF<f64, STATE_DIM, INPUT_DIM>;
    fn ekf_mut(&mut self) -> &mut EKF<f64, STATE_DIM, INPUT_DIM>;
    fn reference_quaternion(&self) -> &UnitQuaternion<f64>;
    fn reference_quaternion_mut(&mut self) -> &mut UnitQuaternion<f64>;

    /// Absorb the orientation error into the reference quaternion and zero
    /// the error states.
    fn reset_orientation_error(&mut self) {
        let i = Self::ORIENTATION_ERR_INDEX;
        let error = {
            let state = self.ekf().state();
            Vector3::new(state[i], state[i + 1], state[i + 2])
        };
        let new_quat = compose_error(self.reference_quaternion(), &error);
        *self.reference_quaternion_mut() = new_quat;
        let state_mut = self.ekf_mut().system.state_mut();
        state_mut[i] = 0.0;
        state_mut[i + 1] = 0.0;
        state_mut[i + 2] = 0.0;
    }

    /// Propagate the reference quaternion forward using current angular velocity.
    fn propagate_reference_quaternion(&mut self, dt: f64) {
        let i = Self::ANGULAR_VEL_INDEX;
        let angular_vel = {
            let state = self.ekf().state();
            Vector3::new(state[i], state[i + 1], state[i + 2])
        };
        let delta_rotation = UnitQuaternion::from_scaled_axis(angular_vel * dt);
        let new_quat = *self.reference_quaternion() * delta_rotation;
        *self.reference_quaternion_mut() = new_quat;
    }

    /// EKF predict step. Propagates reference quaternion, then runs EKF prediction.
    /// Input vector's first element is assumed to be dt.
    fn predict(&mut self, input: SVector<f64, INPUT_DIM>) -> Result<(), KfError> {
        let dt = input[0];
        if dt > 0.0 {
            self.propagate_reference_quaternion(dt);
        }
        self.ekf_mut().predict(input)?;
        Ok(())
    }

    /// EKF update step. Runs the measurement update, then resets orientation error.
    fn update<const M: usize>(
        &mut self,
        measurement: &LinearMeasurement<f64, STATE_DIM, M>,
    ) -> Result<(), KfError> {
        self.ekf_mut().update(measurement)?;
        self.reset_orientation_error();
        Ok(())
    }

    /// Current state vector.
    fn state(&self) -> &SVector<f64, STATE_DIM> {
        self.ekf().state()
    }

    /// Mutable access to the state vector.
    fn state_mut(&mut self) -> &mut SVector<f64, STATE_DIM> {
        self.ekf_mut().system.state_mut()
    }

    /// Current covariance matrix.
    fn covariance(&self) -> &SMatrix<f64, STATE_DIM, STATE_DIM> {
        self.ekf().covariance()
    }

    /// Compute the full orientation by composing reference quaternion with current error.
    fn full_orientation(&self) -> UnitQuaternion<f64> {
        let i = Self::ORIENTATION_ERR_INDEX;
        let state = self.ekf().state();
        let error = Vector3::new(state[i], state[i + 1], state[i + 2]);
        compose_error(self.reference_quaternion(), &error)
    }

    /// Convert current state to an Isometry3 (position + full orientation).
    fn state_to_isometry(&self) -> Isometry3<f64> {
        let state = self.ekf().state();
        let translation = Vector3::new(state[0], state[1], state[2]);
        Isometry3::from_parts(translation.into(), self.full_orientation())
    }
}

/// Kalman filter for measurements in the local/body frame.
/// Used for: T265 (direct pose), ICP (direct pose), IMU (acceleration).
///
/// State vector layout (15 dimensions):
///   [0..3]:   Position (x, y, z)
///   [3..6]:   Velocity (vx, vy, vz)
///   [6..9]:   Orientation error (rotation vector)
///   [9..12]:  Angular velocity (wx, wy, wz)
///   [12..15]: Acceleration (ax, ay, az)
pub struct RelativeMeasurementFilter {
    ekf: EKF<f64, STATE_DIM_LOCAL, 1>,
    reference_quaternion: UnitQuaternion<f64>,
}

impl RelativeMeasurementFilter {
    pub fn new(
        step_fn: fn(SVector<f64, STATE_DIM_LOCAL>, SVector<f64, 1>) -> StepReturn<f64, STATE_DIM_LOCAL>,
        initial_state: SVector<f64, STATE_DIM_LOCAL>,
        initial_covariance: SMatrix<f64, STATE_DIM_LOCAL, STATE_DIM_LOCAL>,
    ) -> Self {
        Self {
            ekf: EKF::new_ekf_with_input(step_fn, initial_state, initial_covariance),
            reference_quaternion: UnitQuaternion::identity(),
        }
    }
}

impl MEKF<STATE_DIM_LOCAL, 1> for RelativeMeasurementFilter {
    const ANGULAR_VEL_INDEX: usize = 9;
    const ORIENTATION_ERR_INDEX: usize = 6;

    fn ekf(&self) -> &EKF<f64, STATE_DIM_LOCAL, 1> {
        &self.ekf
    }

    fn ekf_mut(&mut self) -> &mut EKF<f64, STATE_DIM_LOCAL, 1> {
        &mut self.ekf
    }

    fn reference_quaternion(&self) -> &UnitQuaternion<f64> {
        &self.reference_quaternion
    }

    fn reference_quaternion_mut(&mut self) -> &mut UnitQuaternion<f64> {
        &mut self.reference_quaternion
    }
}

/// Kalman filter for measurements in the global frame.
/// Used for: AprilTags, Vive trackers.
///
/// State vector layout (12 dimensions):
///   [0..3]:  Position (x, y, z)
///   [3..6]:  Velocity (vx, vy, vz)
///   [6..9]:  Orientation error (rotation vector)
///   [9..12]: Angular velocity (wx, wy, wz)
pub struct GlobalMeasurementFilter {
    ekf: EKF<f64, STATE_DIM_GLOBAL, 1>,
    reference_quaternion: UnitQuaternion<f64>,
}

impl GlobalMeasurementFilter {
    pub fn new(
        step_fn: fn(SVector<f64, STATE_DIM_GLOBAL>, SVector<f64, 1>) -> StepReturn<f64, STATE_DIM_GLOBAL>,
        initial_state: SVector<f64, STATE_DIM_GLOBAL>,
        initial_covariance: SMatrix<f64, STATE_DIM_GLOBAL, STATE_DIM_GLOBAL>,
    ) -> Self {
        Self {
            ekf: EKF::new_ekf_with_input(step_fn, initial_state, initial_covariance),
            reference_quaternion: UnitQuaternion::identity(),
        }
    }
}

impl MEKF<STATE_DIM_GLOBAL, 1> for GlobalMeasurementFilter {
    const ANGULAR_VEL_INDEX: usize = 9;
    const ORIENTATION_ERR_INDEX: usize = 6;

    fn ekf(&self) -> &EKF<f64, STATE_DIM_GLOBAL, 1> {
        &self.ekf
    }

    fn ekf_mut(&mut self) -> &mut EKF<f64, STATE_DIM_GLOBAL, 1> {
        &mut self.ekf
    }

    fn reference_quaternion(&self) -> &UnitQuaternion<f64> {
        &self.reference_quaternion
    }

    fn reference_quaternion_mut(&mut self) -> &mut UnitQuaternion<f64> {
        &mut self.reference_quaternion
    }
}
