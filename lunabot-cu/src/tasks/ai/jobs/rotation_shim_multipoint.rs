use std::time::Duration;

use bonsai_bt::{Behavior, Status::Success};
use common::Steering;
use nalgebra::Rotation2;
use tasker::tokio::{self, sync::mpsc, time::{Instant, timeout}};

use crate::{ROBOT_STATE, tasks::ai::{action::LunabotAction, jobs::Job}};

/// attempts to reach target angle within tolerance, the error is divided by agression_factor, and that is used to turn in steering new_ik
pub fn multipoint_rotation_shim(target_angle_degrees: f32, tolerance_degrees: f32, agression_factor: f64, max_points: usize) -> Job<Steering, ()> {
    let target_rot = Rotation2::new(target_angle_degrees.to_radians() as f64);
    let (output_tx, output_rx) = mpsc::channel(5);
    let body = async move {
        for _ in 0..max_points {
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
            if error.abs() < tolerance_degrees.to_radians() as f64 {
                let _ = output_tx
                    .send(Steering::new(0.0,0.0,0.0))
                    .await;
                break;
            }

            // first the backing up
            let _ = timeout(Duration::from_secs(2), async {
                loop {
                    let _ = output_tx.send(Steering::new_ik(-1.0, error/agression_factor, 1200.0)).await;
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }).await;
            // then the going forward
            let _ = timeout(Duration::from_secs(2), async {
                loop {
                    let _ = output_tx.send(Steering::new_ik(1.0, error/agression_factor, 1200.0)).await;
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }).await;
        }
        Success
    };

    Job::spawn(body, output_rx, None)
}
