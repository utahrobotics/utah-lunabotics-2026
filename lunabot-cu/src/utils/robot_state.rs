use std::sync::{Arc, RwLock};

use kalman_filter::{SimpleSquareMatrix, SimpleVector};
use simple_motion::StaticNode;



/// Stores all data about the robot's overall state, for use by any task.
#[derive(Clone, Debug)]
pub struct RobotState {
	/// Position of each part of the robot, determined by the simple_motion
	/// kinematics library
	pub kinematic_root: StaticNode,
	pub kalman_state: Arc<RwLock<SimpleVector<15>>>,
	pub kalman_variances: Arc<RwLock<SimpleSquareMatrix<15>>>,
}

impl RobotState {
	// Kalman states
	pub fn get_position(&self) -> SimpleVector<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 1>(0, 0).clone_owned()
	}

	pub fn get_velocity(&self) -> SimpleVector<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 1>(3, 0).clone_owned()
	}

	pub fn get_acceleration(&self) -> SimpleVector<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 1>(6, 0).clone_owned()
	}

	pub fn get_orientation(&self) -> SimpleVector<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 1>(9, 0).clone_owned()
	}

	pub fn get_angular_velocity(&self) -> SimpleVector<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 1>(12, 0).clone_owned()
	}

	// Kalman internal variances
	pub fn get_position_variance(&self) -> SimpleSquareMatrix<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 3>(0, 0).clone_owned()
	}

	pub fn get_velocity_variance(&self) -> SimpleSquareMatrix<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 3>(3, 3).clone_owned()
	}

	pub fn get_acceleration_variance(&self) -> SimpleSquareMatrix<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 3>(6, 6).clone_owned()
	}

	pub fn get_orientation_variance(&self) -> SimpleSquareMatrix<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 3>(9, 9).clone_owned()
	}

	pub fn get_angular_velocity_variance(&self) -> SimpleSquareMatrix<3> {
		self.kalman_state.read().unwrap().fixed_view::<3, 3>(12,12).clone_owned()
	}
}