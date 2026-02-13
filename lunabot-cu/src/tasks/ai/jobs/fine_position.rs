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
    target_position: Vector2<f32>,
    target_angle:    f32,
) -> Job<Steering> {

    let (output_tx, output_rx) = mpsc::channel(5);
    let mut state: State = State::InitialAlignment;
    Job::spawn(
        async move {

            let mut attempt_count: u32 = 0;

            // Prepare loop interval
            let mut interval = tokio::time::interval(Duration::from_secs_f32(DT));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

            loop {
                interval.tick().await;

                let _robot_isometry = chain.get_global_isometry();
                // Flatten and set up
                let robot_pos = _robot_isometry.translation.vector.xy().cast();
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;

                let error = target_position - robot_pos;
                let error_angle = (error.y).atan2(error.x);

                // Only move if the dot is far enough away
                let steering = match state {
                    State::InitialAlignment => {
                        if (error_angle - robot_angle).abs() < 0.05_f32 {
                            state = state.next();
                        }

                        Steering::new_ik(0.0, 0.5 * (error_angle - robot_angle) as f64, 2000.)
                    },

                    State::InitialTraversal => {
                        let velocity = 1.0; // will be fixed by normalization
                        let turning = (error_angle - robot_angle) * velocity * WHEEL_BASE_SIZE * 0.5;
                        let turning = 0.0; // Temporary override

                        let parallel_dist = error.dot(&Vector2::new(robot_angle.cos(), robot_angle.sin()));
                        if parallel_dist.abs() < 0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },

                    State::AdjustmentTargetAlignment => {
                        if (target_angle - robot_angle).abs() < 0.05_f32 {
                            state = state.next();
                            attempt_count += 1;
                        }
                        
                        Steering::new_ik(0.0, 0.5 * (target_angle - robot_angle) as f64, 2000.)
                    },

                    State::AdjustmentSidestepAlignment => {
                        let sidestep_angle = 
                            target_angle +
                            if (target_position - robot_pos).dot(&Vector2::new(target_angle.cos(), target_angle.sin())) > 0.0 {
                                20.0
                            } else {
                                -20.0
                            }
                        ;

                        if (sidestep_angle - robot_angle).powi(2) < 0.05_f32.powi(2) {
                            state = state.next();
                        }

                        Steering::new_ik(0.0, 0.5 * (sidestep_angle - robot_angle) as f64, 2000.)
                    },

                    State::AdjustmentSidestepTraversal => {
                        let (sidestep_angle, sidestep_speed) = 
                            if (target_position - robot_pos).dot(&Vector2::new(target_angle.cos(), target_angle.sin())) > 0.0 {
                                (target_angle + 20.0, -1.0)
                            } else {
                                (target_angle - 20.0, 1.0)
                            }
                        ;

                        let velocity = sidestep_speed; // will be fixed by normalization
                        let turning = (sidestep_angle - robot_angle) * velocity * WHEEL_BASE_SIZE * 0.5;

                        let perpendicular_distance = cross_vector2(error, Vector2::new(target_angle.cos(), target_angle.sin()));
                        if perpendicular_distance.abs() < 0.05_f32 {
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },

                    State::AdjustmentLineupAlignment => {
                        if (error_angle - robot_angle).abs() < 0.05_f32 {
                            state = state.next();
                        }

                        Steering::new_ik(0.0, 0.5 * (error_angle - robot_angle) as f64, 2000.)
                    },

                    State::AdjustmentLineupTraversal => {
                        let parallel_dist = error.dot(&Vector2::new(target_angle, target_angle.sin().cos()));
                        let velocity = -parallel_dist / parallel_dist.abs(); // will be fixed by normalization
                        let turning = (target_angle - robot_angle) * velocity * WHEEL_BASE_SIZE * 0.5;
                        
                        if parallel_dist.abs() < 0.05 {
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },
                };

                // Prepare for next cycle
                if error.norm_squared() < 0.05_f32.powi(2) && (target_angle - robot_angle).abs() < 0.05_f32 {
                    let _ = output_tx.send(Steering::new(0.0, 0.0, 2000.)).await;
                    break bonsai_bt::Status::Success
                } else if attempt_count > 3 {
                    let _ = output_tx.send(Steering::new(0.0, 0.0, 2000.)).await;
                    break bonsai_bt::Status::Failure
                } else {
                    let _ = output_tx.send(steering).await;
                }
            }
        },
        output_rx,
    )
}

enum State {
    InitialAlignment,
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
            State::InitialAlignment            => State::InitialTraversal,
            State::InitialTraversal            => State::AdjustmentTargetAlignment,
            State::AdjustmentTargetAlignment   => State::AdjustmentSidestepAlignment,
            State::AdjustmentSidestepAlignment => State::AdjustmentSidestepTraversal,
            State::AdjustmentSidestepTraversal => State::AdjustmentLineupAlignment,
            State::AdjustmentLineupAlignment   => State::AdjustmentLineupTraversal,
            State::AdjustmentLineupTraversal   => State::AdjustmentTargetAlignment,
        }
    }
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

fn cross_vector2(a: Vector2<f32>, b: Vector2<f32>) -> f32 {
    a.x * b.y - a.y * b.x
}