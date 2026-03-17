use crate::{pathfinding::field_dstar::NavigationPolicy, rerun_viz::RECORDER};
use common::Steering;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use std::{
    f32::{self, consts::FRAC_PI_2},
    time::Duration,
};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

/// Distance between wheels, in m. Used for calculating the turning circle.
const WHEEL_BASE_SIZE: f32 = 0.6; // TODO Find real numbers
/// Default value for adjustment to sharpness of turns. Higher values result in
/// sharper turns, deviating from the theoretical circle to the target point.
const DEFAULT_TURNING_RATIO_ADJUSTMENT: f32 = 15.0;
/// How fast the robot should move by default, by percent.
const DEFAULT_FOLLOW_SPEED_FACTOR: f32 = 0.25;
/// The power number to send to steering
const STEERING_POWER: f64 = 2000.;
/// If the robot is closer to the goal than this distance, it will zero steering
/// and end the job by default.
const DEFAULT_COMPLETION_DISTANCE: f32 = 0.25;
/// If the robot moves slower than this, it is considered stuck and the job
/// will fail if it stays like this for too long by default.
const DEFAULT_STUCK_SPEED: f32 = 0.15;
/// How long the robot must be moving slower than a given threshold to be considered
/// stuck and fail the job by default.
const DEFAULT_STUCK_TIMEOUT: f32 = 5.0;
/// How long each loop of the controller should be.
const DT: f32 = 0.05;

/// ## Summary
/// Follows the policy given directly.
///
/// **Succeeds when:**
/// * The robot reaches a certain distance from the end point of the path.
///
/// **Fails if:**
/// * Robot fails to move significantly in stuck_timeout_secs.
/// * Job is canceled.
///
/// ## Parameters
/// * `chain` - The kinematic chain which will be used to get robot position.
/// * `policy` - The struct containing what actions to take for any given pose.
/// * `turning_ratio_adjustment` - An adjustment factor on turning speed, for
///     when sharper turning is desired.
/// * `follow_speed_factor` - How fast the robot should move, in percent of maximum.
/// * `completion_distance` - If the robot is closer to the endpoint than this,
///     it is considered done and the job succeeds. (in m)
/// * `stuck_speed` - If the robot is moving slower than this, it is considered
///     stuck, and will fail out of the job if it stays like that for too long. (in m/s).
/// * `stuck_timeout_secs` - If the robot stays at too slow of a speed for this long
///     (continuously), it will fail out of the job. (in s).
///
/// ## Details
/// TODO / N/A
pub fn follow_path_job(
    chain: StaticNode,
    mut policy: NavigationPolicy,
    turning_ratio_adjustment: impl Into<Option<f32>>,
    follow_speed_factor: impl Into<Option<f32>>,
    completion_distance: impl Into<Option<f32>>,
    stuck_speed: impl Into<Option<f32>>,
    stuck_timeout_secs: impl Into<Option<f32>>,
) -> Job<Steering, NavigationPolicy> {
    // Unwrap all the parameters to allow defaults
    let turning_ratio_adjustment = turning_ratio_adjustment
        .into()
        .unwrap_or(DEFAULT_TURNING_RATIO_ADJUSTMENT);
    let follow_speed_factor = follow_speed_factor
        .into()
        .unwrap_or(DEFAULT_FOLLOW_SPEED_FACTOR);
    let completion_distance = completion_distance
        .into()
        .unwrap_or(DEFAULT_COMPLETION_DISTANCE);
    let stuck_speed = stuck_speed.into().unwrap_or(DEFAULT_STUCK_SPEED);
    let stuck_timeout_secs = stuck_timeout_secs.into().unwrap_or(DEFAULT_STUCK_TIMEOUT);

    let (output_tx, output_rx) = mpsc::channel(5);
    let (input_tx, mut input_rx) = mpsc::channel(5);
    Job::spawn(
        async move {
            // Prepare loop interval
            let mut interval = tokio::time::interval(Duration::from_secs_f32(DT));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

            let mut previous_robot_pos: Vector2<f32> =
                chain.get_global_isometry().translation.vector.xy().cast();
            let mut stuck_timer: f32 = 0.0;
            loop {
                // drain pending paths
                while let Ok(new_path) = input_rx.try_recv() {
                    policy = new_path;
                    stuck_timer = 0.0;
                }

                interval.tick().await;

                let _robot_isometry = chain.get_global_isometry();
                // Flatten and set up
                let robot_pos = _robot_isometry.translation.vector.xy().cast();
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;

                // Check if stuck
                if (robot_pos - previous_robot_pos).norm_squared() / (DT * DT)
                    < stuck_speed * stuck_speed
                {
                    stuck_timer += DT;
                    // DEBUG
                    //println!("Follower stuck! Count at {:.3}", stuck_timer);
                    if stuck_timer > stuck_timeout_secs {
                        break bonsai_bt::Status::Failure;
                    }
                } else {
                    stuck_timer = 0.0;
                }


                // MAIN LOGIC
                let steering = if let Some(action) = policy.action_closest_to_pose(&(robot_pos.x, robot_pos.y, robot_angle)) {
                    let turning_percent = action.turn_percent * WHEEL_BASE_SIZE * 0.5 * turning_ratio_adjustment;
                    let velocity = follow_speed_factor * (1.0 - turning_percent) * if action.forward {1.0} else {-1.0};
                    let turning = follow_speed_factor * turning_percent;
                    
                    Steering::new_ik(velocity as f64, turning as f64, STEERING_POWER) // Includes normalization
                } else {
                    Steering::new(0.0, 0.0, STEERING_POWER)
                };
                


                log_to_rerun(robot_pos, robot_angle, steering.get_left_and_right());
                // Prepare for next cycle
                previous_robot_pos = robot_pos;

                if policy.full_goal_dist((robot_pos.x, robot_pos.y, robot_angle)) < completion_distance * completion_distance
                {
                    let _ = output_tx.send(steering).await;
                    break bonsai_bt::Status::Success;
                } else {
                    let _ = output_tx.send(steering).await;
                }
            }
        },
        output_rx,
        input_tx,
    )
}

/// Logs all relevant information to rerun for display. Consider adding steering output.
fn log_to_rerun(
    robot_pos: Vector2<f32>,
    robot_angle: f32,
    steering_left_and_right: (f64, f64),
) {
    // Visualization / logging
    if let Some(rec) = RECORDER.get() {
        // Plots a unit vector for the robot
        let _ = rec.recorder.log(
            "ai/path_follower/robot",
            &rerun::Arrows2D::from_vectors([[robot_angle.cos(), robot_angle.sin()]])
                .with_origins([[robot_pos.x, robot_pos.y]])
                .with_colors([rerun::Color::from_rgb(0, 255, 255)])
                .with_draw_order(40.0),
        );

        let robot_right_side = robot_pos
            + 0.2
                * Vector2::new(
                    (robot_angle - FRAC_PI_2).cos(),
                    (robot_angle - FRAC_PI_2).sin(),
                );
        let robot_left_side = robot_pos
            - 0.2
                * Vector2::new(
                    (robot_angle - FRAC_PI_2).cos(),
                    (robot_angle - FRAC_PI_2).sin(),
                );

        // Plots vectors for applied output to each side of the robot
        let _ = rec.recorder.log(
            "ai/path_follower/steering",
            &rerun::Arrows2D::from_vectors([
                [
                    steering_left_and_right.0 as f32 * robot_angle.cos(),
                    steering_left_and_right.0 as f32 * robot_angle.sin(),
                ],
                [
                    steering_left_and_right.1 as f32 * robot_angle.cos(),
                    steering_left_and_right.1 as f32 * robot_angle.sin(),
                ],
            ])
            .with_origins([
                [robot_left_side.x, robot_left_side.y],
                [robot_right_side.x, robot_right_side.y],
            ])
            .with_colors([rerun::Color::from_rgb(16, 32, 32)])
            .with_draw_order(40.0),
        );
    }
}
