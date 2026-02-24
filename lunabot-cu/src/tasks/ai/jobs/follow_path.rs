use crate::rerun_viz::RECORDER;
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
/// How fast the robot should move when following the dot by default. Error in
/// position is also considered.
const DEFAULT_FOLLOW_SPEED_FACTOR: f32 = 0.25;
/// If the robot is closer to the dot than this distance, it will not move
/// towards the dot, to avoid wild spinning from the tiny distances involved.
const MIN_FOLLOW_DISTANCE: f32 = 0.1;
/// The maximum speed at which the dot will move along the given path, as a
/// factor of the follow speed factor. Provides a cap for the inverse square
/// law used to determine dot speed. Dimensionless, but operates on m/s.
const MAX_DOT_SPEED_FACTOR: f32 = 2.5;
/// How fast the dot will move when the robot is 1 meter away by default. In m/s.
/// The dot follows an inverse square law with robot distance.
const DEFAULT_DOT_SPEED_FACTOR: f32 = 0.35;
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
/// ## Parameters
/// * `chain` - The kinematic chain which will be used to get robot position.
/// * `path` - The list of 2D points used to define the path to follow.
/// * `turning_ratio_adjustment` - An adjustment factor on turning speed, for
///     when sharper turning is desired.
/// * `follow_speed_factor` - How fast the robot should try to move when the
///     dot is one meter away. Robot speed follows an inverse law with distance
///     to dot. (in m/s).
/// * `dot_speed_factor` - How fast the dot should try to move when the
///     robot is one meter away. Dot speed follows an inverse square law with
///     distance to robot. (in m/s).
/// * `completion_distance` - If the robot is closer to the endpoint than this,
///     it is considered done and the job succeeds. (in m)
/// * `stuck_speed` - If the robot is moving slower than this, it is considered
///     stuck, and will fail out of the job if it stays like that for too long. (in m/s).
/// * `stuck_timeout_secs` - If the robot stays at too slow of a speed for this long
///     (continuously), it will fail out of the job. (in s).
///
/// ## Details
/// A traditional pursuit controller has been adapted in two ways:
/// * Adapted to non-holonomic control.
/// * Adapted to have a variable pursuit speed
///
/// The controller works by moving a target, (known here as a "dot", although
/// this is not the technical term) along the path, and directing the robot to
/// drive straight towards it. Because the robot is non-holonomic, this instead
/// draws the circular path with constant forward speed and turning rate that the
/// robot could take to reach the dot, and applies that speed and turning rate.
/// Additionally, the dot moves faster when the robot is closer, according to an
/// inverse square law.
pub fn follow_path_job(
    chain: StaticNode,
    mut path: Vec<Vector2<f32>>,
    turning_ratio_adjustment: impl Into<Option<f32>>,
    follow_speed_factor: impl Into<Option<f32>>,
    dot_speed_factor: impl Into<Option<f32>>,
    completion_distance: impl Into<Option<f32>>,
    stuck_speed: impl Into<Option<f32>>,
    stuck_timeout_secs: impl Into<Option<f32>>,
) -> Job<Steering, Vec<Vector2<f32>>> {
    // Unwrap all the parameters to allow defaults
    let turning_ratio_adjustment = turning_ratio_adjustment
        .into()
        .unwrap_or(DEFAULT_TURNING_RATIO_ADJUSTMENT);
    let follow_speed_factor = follow_speed_factor
        .into()
        .unwrap_or(DEFAULT_FOLLOW_SPEED_FACTOR);
    let dot_speed_factor = dot_speed_factor.into().unwrap_or(DEFAULT_DOT_SPEED_FACTOR);
    let completion_distance = completion_distance
        .into()
        .unwrap_or(DEFAULT_COMPLETION_DISTANCE);
    let stuck_speed = stuck_speed.into().unwrap_or(DEFAULT_STUCK_SPEED);
    let stuck_timeout_secs = stuck_timeout_secs.into().unwrap_or(DEFAULT_STUCK_TIMEOUT);

    let (output_tx, output_rx) = mpsc::channel(5);
    let (input_tx, mut input_rx) = mpsc::channel(5);
    Job::spawn(
        async move {
            // Parameterize path
            let mut path_parameter_boundaries: Vec<f32> = parameterize_path(&path);

            // Prepare loop interval
            let mut interval = tokio::time::interval(Duration::from_secs_f32(DT));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

            // The current target on the path being followed. Will move along the path continuously.
            let mut dot: Vector2<f32> = path[0]; // Start at start of path.
            let mut dot_distance: f32 = 0.0;
            let mut previous_robot_pos: Vector2<f32> =
                chain.get_global_isometry().translation.vector.xy().cast();
            let mut stuck_timer: f32 = 0.0;
            loop {
                // drain pending paths
                let mut got_new_path = false;
                while let Ok(new_path) = input_rx.try_recv() {
                    path = new_path;
                    got_new_path = true;
                }
                if got_new_path {
                    path_parameter_boundaries = parameterize_path(&path);

                    let robot_pos_now: Vector2<f32> =
                        chain.get_global_isometry().translation.vector.xy().cast();

                    // this should make it so the robot doesnt slow all the way down every time a new path is calculated
                    let current_lead = (dot - robot_pos_now).norm();

                    let closest_param =
                        closest_parameter_on_path(robot_pos_now, &path, &path_parameter_boundaries);

                    dot_distance = closest_param + current_lead;
                    dot = parameter_along_path(dot_distance, &path, &path_parameter_boundaries);

                    stuck_timer = 0.0;
                }

                interval.tick().await;

                let _robot_isometry = chain.get_global_isometry();
                // Flatten and set up
                let robot_pos = _robot_isometry.translation.vector.xy().cast();
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;
                let error = dot - robot_pos;

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

                // In m/s. Moves faster when robot is closer, by inverse square law.
                let dot_speed = (dot_speed_factor / error.norm_squared())
                    .min(MAX_DOT_SPEED_FACTOR * follow_speed_factor);

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
                    let velocity = follow_speed_factor * target_distance; // will be fixed by normalization
                    let turning =
                        velocity * WHEEL_BASE_SIZE * turning_ratio_adjustment * 0.5 / radius;

                    Steering::new_ik(velocity as f64, turning as f64, 2000.)
                } else {
                    Steering::new(0.0, 0.0, 2000.0)
                };

                log_to_rerun(robot_pos, robot_angle, dot, steering.get_left_and_right());

                // Prepare for next cycle
                previous_robot_pos = robot_pos;

                if dot_distance > path_parameter_boundaries[path.len() - 1]
                    && error.norm_squared() < completion_distance * completion_distance
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

/// calculates cumulative arc-length boundaries for each point in the path.
fn parameterize_path(path: &[Vector2<f32>]) -> Vec<f32> {
    let mut boundaries: Vec<f32> = Vec::with_capacity(path.len());
    let mut total: f32 = 0.0;
    for (i, point) in path.iter().enumerate() {
        if i != 0 {
            total += (point - path[i - 1]).norm();
        }
        boundaries.push(total);
    }
    boundaries
}

/// finds the arc-length parameter on the path that is closest to the given
/// position. only used when switching to a new path so the dot starts near the robot.
fn closest_parameter_on_path(pos: Vector2<f32>, path: &[Vector2<f32>], boundaries: &[f32]) -> f32 {
    let mut best_dist_sq = f32::MAX;
    let mut best_param = 0.0_f32;
    for i in 1..path.len() {
        let seg = path[i] - path[i - 1];
        let seg_len = seg.norm();
        if seg_len < 1e-9 {
            continue;
        }
        let t_local = ((pos - path[i - 1]).dot(&seg) / (seg_len * seg_len)).clamp(0.0, 1.0);
        let proj = path[i - 1] + seg * t_local;
        let d_sq = (pos - proj).norm_squared();
        if d_sq < best_dist_sq {
            best_dist_sq = d_sq;
            best_param = boundaries[i - 1] + t_local * seg_len;
        }
    }
    best_param
}

/// Turns a path of points and associated distances into a mapping
/// from distance along the path to points along the path.
fn parameter_along_path(
    t: f32,
    path: &Vec<Vector2<f32>>,
    path_parameter_boundaries: &Vec<f32>,
) -> Vector2<f32> {
    // Before start
    if t <= 0.0 {
        return path[0];
    }

    // Linear search. If this takes too long, use a binary search or a hash map
    for i in 1..(path.len()) {
        if path_parameter_boundaries[i] > t {
            let relevant_section = path[i] - path[i - 1];
            return path[i - 1]
                + relevant_section
                    * ((t - path_parameter_boundaries[i - 1])
                        / (path_parameter_boundaries[i] - path_parameter_boundaries[i - 1]));
        }
    }

    // After end
    return path[path.len() - 1];
}

/// Logs all relevant information to rerun for display. Consider adding steering output.
fn log_to_rerun(
    robot_pos: Vector2<f32>,
    robot_angle: f32,
    dot: Vector2<f32>,
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

        // Plots a point for the dot
        let _ = rec.recorder.log(
            "ai/path_follower/dot",
            &rerun::Points2D::new([[dot.x, dot.y]])
                .with_colors([rerun::Color::from_rgb(255, 128, 0)])
                .with_draw_order(60.0),
        );

        // Plots error vector
        let _ = rec.recorder.log(
            "ai/path_follower/error",
            &rerun::Arrows2D::from_vectors([[(dot - robot_pos).x, (dot - robot_pos).y]])
                .with_origins([[robot_pos.x, robot_pos.y]])
                .with_colors([rerun::Color::from_rgb(192, 128, 16)])
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
