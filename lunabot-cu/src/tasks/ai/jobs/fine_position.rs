use std::{f32::{self, consts::{FRAC_PI_2, PI}}, fmt, time::Duration};
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
    target_pos: Vector2<f32>,
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
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;
                let robot_pos = _robot_isometry.translation.vector.xy().cast() + Vector2::new(robot_angle.cos(), robot_angle.sin()) * 0.5; // Patch attempt

                let error = target_pos - robot_pos;
                let error_angle = (error.y).atan2(error.x);

                // Only move if the dot is far enough away
                let steering = match state {
                    State::InitialAlignment => {
                        if (error_angle - robot_angle).abs() < 0.05_f32 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                        }

                        Steering::new_ik(0.0, 1.0 * (error_angle - robot_angle) as f64, 2000.)
                    },

                    State::InitialTraversal => {
                        let velocity = 1.0; // will be fixed by normalization
                        //let turning = (error_angle - robot_angle) * velocity * WHEEL_BASE_SIZE * 0.5;
                        let turning = 0.0; // Temporary override

                        let parallel_dist = error.dot(&Vector2::new(robot_angle.cos(), robot_angle.sin()));
                        if parallel_dist.abs() < 0.05 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },

                    State::AdjustmentTargetAlignment => {
                        if (target_angle - robot_angle).abs() < 0.05_f32 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                            attempt_count += 1;
                        }
                        
                        Steering::new_ik(0.0, 1.0 * (target_angle - robot_angle) as f64, 2000.)
                    },

                    State::AdjustmentSidestepAlignment => {
                        let sidestep_angle = angle_norm(
                            target_angle +
                            if (target_pos - robot_pos).dot(&Vector2::new(target_angle.cos(), target_angle.sin())) > 0.0 {
                                20.0 * PI / 180.0
                            } else {
                                -20.0 * PI / 180.0
                            }
                        );

                        if (sidestep_angle - robot_angle).abs() < 0.05_f32 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                        }

                        Steering::new_ik(0.0, 1.0 * (sidestep_angle - robot_angle) as f64, 2000.)
                    },

                    State::AdjustmentSidestepTraversal => {
                        let (sidestep_angle, sidestep_speed) = 
                            if (target_pos - robot_pos).dot(&Vector2::new(target_angle.cos(), target_angle.sin())) > 0.0 {
                                (target_angle + 20.0, -1.0)
                            } else {
                                (target_angle - 20.0, 1.0)
                            }
                        ;

                        let velocity = sidestep_speed; // will be fixed by normalization
                        let turning = 0.0;//(sidestep_angle - robot_angle) * velocity * WHEEL_BASE_SIZE * 0.5;

                        let perpendicular_distance = cross_vector2(error, Vector2::new(target_angle.cos(), target_angle.sin()));
                        if perpendicular_distance.abs() < 0.05_f32 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },

                    State::AdjustmentLineupAlignment => {
                        if (error_angle - robot_angle).abs() < 0.05_f32 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                        }

                        Steering::new_ik(0.0, 1.0 * (error_angle - robot_angle) as f64, 2000.)
                    },

                    State::AdjustmentLineupTraversal => {
                        let parallel_dist = error.dot(&Vector2::new(target_angle, target_angle.sin().cos()));
                        let velocity = -parallel_dist / parallel_dist.abs(); // will be fixed by normalization
                        let turning = (target_angle - robot_angle) * velocity * WHEEL_BASE_SIZE * 0.5;
                        
                        if parallel_dist.abs() < 0.05 {
                            on_phase_change(state, attempt_count, 3, robot_pos, robot_angle);
                            state = state.next();
                        }

                        Steering::new_ik(velocity as f64, turning as f64, 2000.)
                    },
                };

                log_to_rerun(robot_pos, robot_angle, target_pos, target_angle, steering.get_left_and_right());
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

#[derive(Clone, Copy)]
enum State {
    InitialAlignment,
    InitialTraversal,
    AdjustmentTargetAlignment,
    AdjustmentSidestepAlignment,
    AdjustmentSidestepTraversal,
    AdjustmentLineupAlignment,
    AdjustmentLineupTraversal,
}

impl fmt::Debug for State {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::InitialAlignment =>            write!(f, "InitialAlignment            | ) * |"),
            Self::InitialTraversal =>            write!(f, "InitialTraversal            | ->* |"),
            Self::AdjustmentTargetAlignment =>   write!(f, "AdjustmentTargetAlignment   | *)  |"),
            Self::AdjustmentSidestepAlignment => write!(f, "AdjustmentSidestepAlignment | ( * |"),
            Self::AdjustmentSidestepTraversal => write!(f, "AdjustmentSidestepTraversal | <-* |"),
            Self::AdjustmentLineupAlignment =>   write!(f, "AdjustmentLineupAlignment   | ) * |"),
            Self::AdjustmentLineupTraversal =>   write!(f, "AdjustmentLineupTraversal   | ->* |"),
        }
        
    }
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
fn log_to_rerun(robot_pos: Vector2<f32>, robot_angle: f32, target_pos: Vector2<f32>, target_angle: f32, steering_left_and_right: (f64, f64)) {
    // Visualization / logging
    if let Some(rec) = RECORDER.get() {
        // Plots a unit vector for the robot
        let _ = rec.recorder.log(
            "ai/fine_positioner/robot",
            &rerun::Arrows2D::from_vectors([[
                    robot_angle.cos(), 
                    robot_angle.sin()]])
                .with_origins([[robot_pos.x, robot_pos.y]])
                .with_colors([rerun::Color::from_rgb(0, 255, 255)])
                .with_draw_order(40.0),
        );

        // Plots a unit vector for the target
        let _ = rec.recorder.log(
            "ai/fine_positioner/target",
            &rerun::Arrows2D::from_vectors([[
                    target_angle.cos(), 
                    target_angle.sin()]])
                .with_origins([[target_pos.x, target_pos.y]])
                .with_colors([rerun::Color::from_rgb(255, 128, 0)])
                .with_draw_order(40.0),
        );

        let robot_right_side = robot_pos + 0.2 * Vector2::new((robot_angle - FRAC_PI_2).cos(), (robot_angle - FRAC_PI_2).sin());
        let robot_left_side =  robot_pos - 0.2 * Vector2::new((robot_angle - FRAC_PI_2).cos(), (robot_angle - FRAC_PI_2).sin());

        // Plots vectors for applied output to each side of the robot
        let _ = rec.recorder.log(
            "ai/fine_positioner/steering",
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

fn on_phase_change(prev_state: State, attempt: u32, attempt_limit: u32, position: Vector2<f32>, angle: f32) {
    println!(
        "Fine positioner completed {:?} and is now performing {:?}\n\tAttempt: {}/{}\n\tPose: ({:3}, {:3}) {:3}°",
        prev_state, prev_state.next(),
        attempt + 1, attempt_limit,
        position.x, position.y, angle * 180.0 / PI
    );
}

fn cross_vector2(a: Vector2<f32>, b: Vector2<f32>) -> f32 {
    a.x * b.y - a.y * b.x
}

fn angle_norm(x: f32) -> f32 {
    let mut result = x;
    result %= PI * 2.0;
    if result > PI {
        result = PI - result;
    }
    if result < -PI {
        result = -PI - result;
    }
    return result;
}