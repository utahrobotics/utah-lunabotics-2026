use std::time::Duration;

use bonsai_bt::Status::{Failure, Success};
use common::Steering;
use nalgebra::{Rotation2, Vector2};
use tasker::tokio::{self, sync::mpsc};

use crate::{ROBOT_STATE, tasks::ai::jobs::Job};

/// Used to rotate the robot to start facing the direction of travel when a path comes in
/// https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html
/// TODO: test in sim
pub fn rotation_shim(path: &Vec<Vector2<f32>>, tolerance: f64) -> Job<Steering> {
    let (output_tx, output_rx) = mpsc::channel(5);
    // first calculate the direction the robot should be pointing based on the first few points in the path
    let target_yaw = direction_from_path(path);
    // no need for full pid because it would just end up being fully proportional mattering anyways
    let body = async move {
        let Some(target_yaw) = target_yaw else {
            return Failure;
        };
        let target_rot = Rotation2::new(target_yaw as f64);
        loop {
            // idc about unwrapping here because we are in an async task, and if the robot state is uninit,
            // then we might as well blow up anyways
            let current_yaw: f64 = ROBOT_STATE.get().unwrap().kinematic_root.get_root().get_global_isometry().rotation.euler_angles().2;
            let current_rot = Rotation2::new(current_yaw);
            let angle_diff = (target_rot * current_rot.inverse()).angle();
            if angle_diff.abs() < tolerance {
                let _ = output_tx.send(Steering::new_ik(0.0, 0.0, Steering::DEFAULT_WEIGHT)).await;
                break;
            }
            output_tx.send(Steering::new_ik(0.0,angle_diff, Steering::DEFAULT_WEIGHT)).await.expect("failed to send");
            tokio::time::sleep(Duration::from_millis(30)).await;     
        }
        Success
    };
    
    Job::spawn(body, output_rx)
}

/// outputs angle as yaw
fn direction_from_path(path: &Vec<Vector2<f32>>) -> Option<f32> {
    if path.len() < 2 {
        return None;
    }
    let delta = path[1] - path[0];
    Some(delta.y.atan2(delta.x))
}