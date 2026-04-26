use crate::{ROBOT_STATE, tasks::ai::jobs::Job};
use bonsai_bt::Status::Success;
use common::Steering;
use nalgebra::{Rotation2, Vector2};
use std::time::{Duration, Instant};
use tasker::tokio::{self, sync::mpsc};

/// rotates to target yaw within tolerance radians
/// pass in target yaw as degrees
pub fn rotation_shim(
    target_yaw: f32,
    tolerance: f64,
    kp: impl Into<Option<f64>>,
    ki: impl Into<Option<f64>>,
) -> Job<Steering, ()> {
    let (output_tx, output_rx) = mpsc::channel(5);
    let kp = kp.into().unwrap_or(1.0);
    let ki = ki.into().unwrap_or(0.5);

    let body = async move {
        let target_rot = Rotation2::new(target_yaw.to_radians() as f64);

        let max_integral = 1.0;

        let mut integral = 0.0;
        let mut last_time = Instant::now();

        loop {
            let now = Instant::now();
            let dt = now.duration_since(last_time).as_secs_f64();
            last_time = now;

            let current_yaw: f64 = ROBOT_STATE
                .get()
                .unwrap()
                .kinematic_root
                .get_root()
                .get_global_isometry()
                .rotation
                .euler_angles()
                .2;
            let current_rot = Rotation2::new(current_yaw);
            let error = (target_rot * current_rot.inverse()).angle();

            if error.abs() < tolerance {
                let _ = output_tx
                    .send(Steering::new_ik(0.0, 0.0, Steering::DEFAULT_WEIGHT))
                    .await;
                break;
            }

            // accumulate integral with anti-windup
            integral = (integral + error * dt).clamp(-max_integral, max_integral);

            let output = kp * error + ki * integral;

            output_tx
                .send(Steering::new_ik(0.0, output, Steering::DEFAULT_WEIGHT))
                .await
                .expect("failed to send");

            tokio::time::sleep(Duration::from_millis(30)).await;
        }
        Success
    };

    Job::spawn(body, output_rx, None)
}
/// finds initial direction of travel based on first two nodes in a path
pub fn direction_from_path(path: &Vec<Vector2<f32>>) -> Option<f32> {
    if path.len() < 2 {
        return None;
    }
    let delta = path[1] - path[0];
    Some(delta.y.atan2(delta.x))
}
