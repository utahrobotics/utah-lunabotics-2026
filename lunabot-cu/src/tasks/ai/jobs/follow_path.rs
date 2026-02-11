use std::{f32::{self, consts::FRAC_PI_2}, time::Duration};

use crate::rerun_viz;
use crate::rerun_viz::RECORDER;
use common::Steering;
use nalgebra::Vector2;
use rerun::Archetype;
use simple_motion::StaticNode;
use tasker::tokio::{
    self,
    sync::{mpsc, watch},
};

use crate::tasks::ai::jobs::Job;

/// Distance between wheels, in m. Used for calculating the turning circle.
const WHEEL_BASE_SIZE: f32 = 0.6; // TODO Find real numbers
/// Adjustment to sharpness of turns. Higher values result in sharper turns,
/// deviating from the theoretical circle to the target point.
const TURNING_RATIO_ADJUSTMENT: f32 = 15.0;
/// How fast the robot should move when following the dot. Error in position
/// is also considered.
const FOLLOW_SPEED_FACTOR: f32 = 0.25;
/// If the robot is closer to the dot than this distance, it will not move 
/// towards the dot, to avoid wild spinning from the tiny distances involved.
const MIN_FOLLOW_DISTANCE: f32 = 0.1;
/// The maximum speed at which the dot will move along the given path. Provides
/// a cap for the inverse square law used to determine dot speed. In m/s.
const MAX_DOT_SPEED: f32 = 0.75;
/// How fast the dot will move when the robot is 1 meter away. In m/s. The dot
/// follows an inverse square law with robot distance.
const DOT_SPEED_FACTOR: f32 = 0.35;
/// If the robot is closer to the goal than this distance, it will zero steering
/// and end the job.
const COMPLETION_DISTANCE: f32 = 0.15;
/// If the robot moves slower than this, it is considered stuck and the job
/// will fail if it stays like this for too long.
const STUCK_SPEED: f32 = 0.1;
/// How long each loop of the controller should be.
const DT: f32 = 0.05;

/// ## Summary
/// Uses a pursuit controller to follow a given path in 2D. Directly communicates
/// with the drivetrain to follow the path.
/// 
/// **Succeeds when:**
/// * The robot reaches a certain distance from the end point of the path.
/// 
/// **Fails if:**
/// * Robot fails to move significantly in stuck_timeout_secs.
/// * Job is canceled.
/// 
/// ## Details
/// A traditional pursuit controller has been adapted in two ways:
/// * Adapted to non-holonomic control.
/// * Adapted to have a variable pursuit speed
/// 
/// The controller works by moving a target, (known here as a "dot", although
/// this is not the technical term) along the path, and directing the robot to
/// drive straight towards it.
pub fn follow_path_job(
    stuck_timeout_secs: f32,
    chain: StaticNode,
    path: Vec<Vector2<f32>>,
) -> Job<Steering> {
    let (status_tx, status_rx) = watch::channel(bonsai_bt::Status::Running);
    let (output_tx, output_rx) = mpsc::channel(5);
    Job::spawn(
        async move {
            // Parameterize path
            let mut path_parameter_boundaries: Vec<f32> = Vec::new();
            let mut previous_totals: f32 = 0.0;
            for (i, point) in path.iter().enumerate() {
                if i != 0 {
                    previous_totals += (point - path[i-1]).norm();
                }
                path_parameter_boundaries.push(previous_totals);
            }

            // Prepare loop interval
            let mut interval = tokio::time::interval(Duration::from_secs_f32(DT));

            // The current target on the path being followed. Will move along the path continuously.
            let mut dot: Vector2<f32> = path[0]; // Start at start of path.
            let mut dot_distance: f32 = 0.0;
            let mut previous_robot_pos: Vector2<f32> = chain.get_global_isometry().translation.vector.xy().cast();
            let mut stuck_timer: f32 = 0.0;
            loop {
                let _robot_isometry = chain.get_global_isometry();
                // Flatten and set up
                let robot_pos = _robot_isometry.translation.vector.xy().cast();
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;
                let error = dot - robot_pos;

                // Check if stuck
                if (robot_pos - previous_robot_pos).norm_squared() / DT < STUCK_SPEED*STUCK_SPEED {
                    stuck_timer += DT;
                    if stuck_timer > stuck_timeout_secs {
                        let _ = status_tx.send(bonsai_bt::Status::Failure);
                        break bonsai_bt::Status::Failure
                    }
                } else {
                    stuck_timer = 0.0;
                }

                // In m/s. Moves faster when robot is closer, by inverse square law.
                let dot_speed = (DOT_SPEED_FACTOR / error.norm_squared()).min(MAX_DOT_SPEED);

                // Update dot location
                dot_distance += DT * dot_speed;
                dot = parameter_along_path(dot_distance, &path, &path_parameter_boundaries);

                let error = dot - robot_pos; // Update error again

                // Calculate steering from dot and robot position
                //   Some relevant math:
                //     https://www.desmos.com/calculator/pbk1kdxg5z
                //     https://www.desmos.com/geometry/emcgvxxrg5
                //   Determine radius of turn, so that turn intersects dot, and
                //   aligns with current robot angle.
                let target_distance = error.norm();
                let error_angle = (error.y).atan2(error.x);
                let angle_difference = error_angle - robot_angle;
                let radius = target_distance / (2.0 * angle_difference.sin()); // Check if this has a sign error

                // Only move if the dot is far enough away
                let steering = if error.norm_squared() > MIN_FOLLOW_DISTANCE * MIN_FOLLOW_DISTANCE {
                    //   Radius is proportional to the ratio of velocity to angular velocity:
                    //   https://www.desmos.com/calculator/f7grn652s4
                    // TODO Fix problems with dot being behind bot
                    let velocity = FOLLOW_SPEED_FACTOR * target_distance; // will be fixed by normalization
                    let turning = velocity * WHEEL_BASE_SIZE * TURNING_RATIO_ADJUSTMENT * 0.5 / radius;

                    Steering::new_ik(velocity as f64, turning as f64, 5000.0)
                } else {
                    Steering::new(0.0, 0.0, 5000.0)
                };

                log_to_rerun(robot_pos, robot_angle, dot, steering.get_left_and_right());

                // Prepare for next cycle
                previous_robot_pos = robot_pos;

                if dot_distance > path_parameter_boundaries[path.len() - 1] && error.norm_squared() < COMPLETION_DISTANCE * COMPLETION_DISTANCE {
                    let _ = output_tx.send(steering).await;
                    let _ = status_tx.send(bonsai_bt::Status::Success);
                    break bonsai_bt::Status::Success
                } else {
                    let _ = output_tx.send(steering).await;
                    interval.tick().await;
                    let _ = status_tx.send(bonsai_bt::Status::Running);
                }
            }
        },
        status_rx,
        output_rx,
    )
}

/// Turns a path of points and associated distances into a mapping
/// from distance along the path to points along the path.
fn parameter_along_path(t: f32, path: &Vec<Vector2<f32>>, path_parameter_boundaries: &Vec<f32>) -> Vector2<f32> {
    // Before start
    if t <= 0.0 {
        return path[0];
    }

    // Linear search. If this takes too long, use a binary search or a hash map
    for i in 1..(path.len()) {
        if path_parameter_boundaries[i] > t {
            let relevant_section = path[i] - path[i-1];
            return
                path[i-1] +
                relevant_section * (
                    (t - path_parameter_boundaries[i-1]) /
                    (path_parameter_boundaries[i] - path_parameter_boundaries[i-1])
                );
        }
    }

    // After end
    return path[path.len() - 1];
}

/// Logs all relevant information to rerun for display. Consider adding steering output.
fn log_to_rerun(robot_pos: Vector2<f32>, robot_angle: f32, dot: Vector2<f32>, steering_left_and_right: (f64, f64)) {
    // Visualization / logging
    if let Some(rec) = RECORDER.get() {
        // Plots a unit vector for the robot
        let _ = rec.recorder.log(
            "ai/path_follower/robot",
            &rerun::Arrows2D::from_vectors([[
                    robot_pos.x + robot_angle.cos(), 
                    robot_pos.y + robot_angle.sin()]])
                .with_origins([[robot_pos.x, robot_pos.y]])
                .with_colors([rerun::Color::from_rgb(0, 255, 128)]),
        );

        // Plots a point for the dot
        let _ = rec.recorder.log(
            "ai/path_follower/dot",
            &rerun::Points2D::new([[dot.x, dot.y]])
                .with_colors([rerun::Color::from_rgb(255, 0, 32)]),
        );

        // Plots error vector
        let _ = rec.recorder.log(
            "ai/path_follower/error",
            &rerun::Arrows2D::from_vectors([[dot.x, dot.y]])
                .with_origins([[robot_pos.x, robot_pos.y]])
                .with_colors([rerun::Color::from_rgb(192, 128, 16)]),
        );

        let robot_right_side = robot_pos + 0.2 * Vector2::new((robot_angle - FRAC_PI_2).cos(), (robot_angle - FRAC_PI_2).sin());
        let robot_left_side =  robot_pos - 0.2 * Vector2::new((robot_angle - FRAC_PI_2).cos(), (robot_angle - FRAC_PI_2).sin());

        // Plots vectors for applied output to each side of the robot
        let _ = rec.recorder.log(
            "ai/path_follower/steering",
            &rerun::Arrows2D::from_vectors([
                [
                    robot_left_side.x + steering_left_and_right.0 as f32 * robot_angle.cos(), 
                    robot_left_side.y + steering_left_and_right.0 as f32 * robot_angle.sin()
                ],
                [
                    robot_right_side.x + steering_left_and_right.1 as f32 * robot_angle.cos(), 
                    robot_right_side.y + steering_left_and_right.1 as f32 * robot_angle.sin()
                ]
            ])
                .with_origins([
                    [robot_left_side.x, robot_left_side.y],
                    [robot_right_side.x, robot_right_side.y]
                ])
                .with_colors([rerun::Color::from_rgb(72, 192, 64)]),
        );
    }
}