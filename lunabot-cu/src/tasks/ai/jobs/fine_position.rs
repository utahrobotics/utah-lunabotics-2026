use std::{f32::{self, consts::FRAC_PI_2}, time::Duration};
use crate::rerun_viz::RECORDER;
use common::Steering;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use tasker::tokio::{
    self,
    sync::mpsc,
};

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
/// 
/// ## Parameters
/// 
/// ## Details
///
pub fn fine_position_job(
    chain:           StaticNode,
    target_position: Vector2<f32>
    target_angle:    f32
) -> Job<Steering> {

    let (output_tx, output_rx) = mpsc::channel(5);
    let mut state: State = State::InitialTurning;
    Job::spawn(
        async move {

            // Prepare loop interval
            let mut interval = tokio::time::interval(Duration::from_secs_f32(DT));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

            loop {
                interval.tick().await;

                let _robot_isometry = chain.get_global_isometry();
                // Flatten and set up
                let robot_pos = _robot_isometry.translation.vector.xy().cast();
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;

                // Only move if the dot is far enough away
                let steering = match state {
                    InitialTurning {
                        if (target_angle - robot_angle)*(target_angle - robot_angle) < 0.05*0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(0.0, (target_angle - robot_angle) * 0.5, 2000.)
                    },
                    InitialTraversal {
                        // Probably shouldn't be a circle follower
                        let error = target_position - robot_pos;
                        let error_angle = (error.y).atan2(error.x);

                        let target_distance = error.norm();
                        let angle_difference = error_angle - robot_angle;
                        let radius = target_distance / (2.0 * angle_difference.sin());
                        let velocity = follow_speed_factor * target_distance; // will be fixed by normalization
                        let turning = velocity * WHEEL_BASE_SIZE * turning_ratio_adjustment * 0.5 / radius;

                        if error.norm_squared() < 0.05*0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },
                    AdjustmentTargetAlignment {
                        let error = target_position - robot_pos;

                        if (error_angle - robot_angle)*(error_angle - robot_angle) < 0.05*0.05 {
                            if error.norm_squared() < 0.05*0.05 {
                                let _ = output_tx.send(Steering::new(0.0, 0.0, 2000.0)).await;
                                break bonsai_bt::Status::Success
                            }
                            state = state.next();
                        }
                        
                        Steering::new_ik(0.0, (error_angle - robot_angle) * 0.5, 2000.)
                    },
                    AdjustmentSidestepAlignment {
                        // TODO PROJECTION
                        if (target_angle - robot_angle)*(target_angle - robot_angle) < 0.05*0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(0.0, (target_angle - robot_angle) * 0.5, 2000.)
                    },
                    AdjustmentSidestepTraversal {
                        // TODO PROJECTION
                        // Probably shouldn't be a circle follower
                        let error = target_position - robot_pos;
                        let error_angle = (error.y).atan2(error.x);

                        let target_distance = error.norm();
                        let angle_difference = error_angle - robot_angle;
                        let radius = target_distance / (2.0 * angle_difference.sin());
                        let velocity = follow_speed_factor * target_distance; // will be fixed by normalization
                        let turning = velocity * WHEEL_BASE_SIZE * turning_ratio_adjustment * 0.5 / radius;

                        if error.norm_squared() < 0.05*0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },
                    AdjustmentLineupAlignment {
                        let error = target_position - robot_pos;
                        let error_angle = (error.y).atan2(error.x);

                        if (error_angle - robot_angle)*(target_angle - robot_angle) < 0.05*0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(0.0, (error_angle - robot_angle) * 0.5, 2000.)
                    },
                    AdjustmentLineupTraversal {
                        // Probably shouldn't be a circle follower
                        let error = target_position - robot_pos;
                        let error_angle = (error.y).atan2(error.x);

                        let target_distance = error.norm();
                        let angle_difference = error_angle - robot_angle;
                        let radius = target_distance / (2.0 * angle_difference.sin());
                        let velocity = follow_speed_factor * target_distance; // will be fixed by normalization
                        let turning = velocity * WHEEL_BASE_SIZE * turning_ratio_adjustment * 0.5 / radius;

                        if error.norm_squared() < 0.05*0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },
                };

                log_to_rerun(robot_pos, robot_angle, dot, steering.get_left_and_right());

                // Prepare for next cycle
                previous_robot_pos = robot_pos;

                if dot_distance > path_parameter_boundaries[path.len() - 1] && error.norm_squared() < completion_distance*completion_distance {
                    let _ = output_tx.send(steering).await;
                    break bonsai_bt::Status::Success
                } else {
                    let _ = output_tx.send(steering).await;
                }
            }
        },
        output_rx,
    )
}

enum State {
    InitialTurning,
    InitialTraversal,
    AdjustmentTargetAlignment,
    AdjustmentSidestepAlignment,
    AdjustmentSidestepTraversal,
    AdjustmentLineupAlignment,
    AdjustmentLineupTraversal,
}

impl State {
    fn next(self) -> State {
        match self {
            State::InitialTurning              => InitialTraversal,
            State::InitialTraversal            => AdjustmentTargetAlignment,
            State::AdjustmentTargetAlignment   => AdjustmentSidestepAlignment,
            State::AdjustmentSidestepAlignment => AdjustmentSidestepTraversal,
            State::AdjustmentSidestepTraversal => AdjustmentLineupAlignment,
            State::AdjustmentLineupAlignment   => AdjustmentLineupTraversal,
            State::AdjustmentLineupTraversal   => AdjustmentTargetAlignment,
        }
    }
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
                    robot_angle.cos(), 
                    robot_angle.sin()]])
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
            &rerun::Arrows2D::from_vectors([[(dot-robot_pos).x, (dot-robot_pos).y]])
                .with_origins([[robot_pos.x, robot_pos.y]])
                .with_colors([rerun::Color::from_rgb(192, 128, 16)])
                .with_draw_order(40.0),
        );

        let robot_right_side = robot_pos + 0.2 * Vector2::new((robot_angle - FRAC_PI_2).cos(), (robot_angle - FRAC_PI_2).sin());
        let robot_left_side =  robot_pos - 0.2 * Vector2::new((robot_angle - FRAC_PI_2).cos(), (robot_angle - FRAC_PI_2).sin());

        // Plots vectors for applied output to each side of the robot
        let _ = rec.recorder.log(
            "ai/path_follower/steering",
            &rerun::Arrows2D::from_vectors([
                [
                    steering_left_and_right.0 as f32 * robot_angle.cos(), 
                    steering_left_and_right.0 as f32 * robot_angle.sin()
                ],
                [
                    steering_left_and_right.1 as f32 * robot_angle.cos(), 
                    steering_left_and_right.1 as f32 * robot_angle.sin()
                ]
            ])
                .with_origins([
                    [robot_left_side.x, robot_left_side.y],
                    [robot_right_side.x, robot_right_side.y]
                ])
                .with_colors([rerun::Color::from_rgb(16, 32, 32)])
                .with_draw_order(40.0),
        );
    }
}