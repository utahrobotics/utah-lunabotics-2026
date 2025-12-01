use bonsai_bt::Status::{self, *};
use common::{LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::{Actuator, ActuatorCommand};
use rerun::Vec3D;

use crate::{
    ROBOT_STATE,
    pathfinding::rrt::find_path,
    rerun_viz::RECORDER,
    tasks::ai::{blackboard::LunabotBlackboard, jobs::follow_path_job},
};
static PATHFINDING_GOAL: [f32; 2] = [3.0, 0.0];

#[derive(Clone, Debug, Copy)]
pub enum LunabotAction {
    Yield,
    SetSteering(Steering),
    SetLastSteering,
    SetLastLift,
    SetLastBucket,
    SetLift(i8),
    SetBucket(i8),
    // actions for checking the lunabot stage (dig, dump, manual, soft stop, navigate)
    IsSoftStop,
    IsAutonomy,
    IsManual,
    None,

    // autonomy related things
    IsObstacleMapReady,

    /// if the robot is in an occupied cell, we should first pathfind to the nearest free cell (if a free cell is within some range)
    IsInOccupiedCell,
    IsInFreeCell,
    /// Success if the robot has reached destination, Running if the robot is currently navigating, Failiure if the robot got stuck
    CheckNavigation,
    /// if the robot is in a cell of unknown status, we should first pathfind to the nearest free cell (if a free cell is within some range)
    IsInUnknownCell,
    /// calculates path from the robots position to x,y
    CalculatePath,
    FollowPath,
    SetStage(LunabotStage),
    GetUnstuck,
}

impl LunabotAction {
    /// returns (Status, remaining time)
    pub fn handle(&self, blackboard: &mut LunabotBlackboard) -> (Status, f64) {
        let status = match self {
            LunabotAction::SetSteering(steering) => {
                blackboard.outgoing_steering_msg = Some(*steering);
                Success
            }
            LunabotAction::SetLastSteering => {
                if let Some(steering) = blackboard.last_steering {
                    blackboard.outgoing_steering_msg = Some(steering);
                }
                Success
            }
            LunabotAction::SetLastBucket => {
                if let Some(value) = blackboard.last_bucket.take() {
                    let commands = actuator_commands_from_i8(value, Actuator::Bucket);
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(commands[0]);
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(commands[1]);
                }
                Success
            }
            LunabotAction::SetLastLift => {
                if let Some(value) = blackboard.last_lift.take() {
                    let commands = actuator_commands_from_i8(value, Actuator::Lift);
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(commands[0]);
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(commands[1]);
                }
                Success
            }
            LunabotAction::SetBucket(value) => {
                let [direction, speed] = actuator_commands_from_i8(*value, Actuator::Bucket);
                blackboard.last_bucket = None;
                blackboard.outgoing_actuator_msg_queue.push_back(direction);
                blackboard.outgoing_actuator_msg_queue.push_back(speed);
                Success
            }
            LunabotAction::SetLift(value) => {
                let [direction, speed] = actuator_commands_from_i8(*value, Actuator::Lift);
                blackboard.last_lift = None;
                blackboard.outgoing_actuator_msg_queue.push_back(direction);
                blackboard.outgoing_actuator_msg_queue.push_back(speed);
                Success
            }
            LunabotAction::IsSoftStop => match blackboard.current_mission {
                LunabotStage::SoftStop => Running,
                _ => Success,
            },
            LunabotAction::IsAutonomy => match blackboard.current_mission {
                LunabotStage::Autonomy => Running,
                _ => Success,
            },
            LunabotAction::IsManual => match blackboard.current_mission {
                LunabotStage::Manual => Running,
                _ => Success,
            },
            LunabotAction::None => Success,
            LunabotAction::IsObstacleMapReady => {
                // if blackboard.latest_obstacle_map.is_some() {
                //     Success
                // } else {
                //     Failure
                // }
                Success
            }
            LunabotAction::IsInOccupiedCell => todo!(),
            LunabotAction::IsInFreeCell => todo!(),
            LunabotAction::IsInUnknownCell => todo!(),
            LunabotAction::CalculatePath => {
                if let Some(ref map) = blackboard.latest_obstacle_map {
                    let translation = blackboard.kinematic_root.get_global_isometry().translation;
                    if let Some(path) = find_path(
                        map,
                        [translation.x as f32, translation.y as f32],
                        PATHFINDING_GOAL,
                        0.3,
                        map.cell_size * 2.,
                        // FIXME: tune this parameter
                        100,
                    ) && let Some(rec) = RECORDER.get()
                    {
                        let _ = rec.recorder.log(
                            "ai/calculated_path",
                            &rerun::LineStrips3D::new(&[path
                                .iter()
                                .map(|p| Vec3D::new(p.0 as f32, p.1 as f32, 1.0))
                                .collect::<Vec<_>>()])
                            .with_colors(vec![rerun::Color::from_rgb(0, 200, 0)]),
                        );
                        Success
                    } else {
                        Failure
                    }
                } else {
                    Failure
                }
            }
            LunabotAction::FollowPath => {
                if ROBOT_STATE.get().is_none() {
                    eprintln!(
                        "Cannot start follow path job because ROBOT_STATE is not initialized"
                    );
                    Failure
                } else if let Some(ref mut path_follower) = blackboard.path_follower {
                    blackboard.outgoing_steering_msg = path_follower.get_output();
                    let status = path_follower.get_status();
                    if status == Success || status == Failure {
                        println!("Follow path job completed with status: {:?}", status);
                        // ensure the task is no longer running just in case
                        path_follower.cancel();
                        blackboard.path_follower = None;
                    }
                    status
                } else {
                    println!("Starting new follow path job");
                    let mut follower_job =
                        follow_path_job(5.0, ROBOT_STATE.get().unwrap().kinematic_root, vec![]);
                    let job_initial_status = follower_job.get_status();
                    blackboard.path_follower = Some(follower_job);
                    println!(
                        "Follow path job started with initial status: {:?}",
                        job_initial_status
                    );
                    job_initial_status
                }
            }
            LunabotAction::CheckNavigation => Running,
            LunabotAction::GetUnstuck => todo!(),
            LunabotAction::Yield => {
                if !blackboard.yielded {
                    blackboard.yielded = true;
                    Running
                } else {
                    blackboard.yielded = false;
                    Success
                }
            }
            LunabotAction::SetStage(stage) => {
                println!("Setting stage to {:?}", stage);
                blackboard.last_mission = LunabotStage::Manual;
                blackboard.current_mission = *stage;
                blackboard.path_follower = None;
                LUNABOT_STAGE.store(*stage);
                Success
            }
        };
        (status, 0.0)
    }
}

fn actuator_commands_from_i8(value: i8, actuator: Actuator) -> [ActuatorCommand; 2] {
    if value < 0 {
        [
            ActuatorCommand::backward(actuator),
            ActuatorCommand::set_speed(value as f64 / i8::MIN as f64, actuator),
        ]
    } else {
        [
            ActuatorCommand::forward(actuator),
            ActuatorCommand::set_speed(value as f64 / i8::MAX as f64, actuator),
        ]
    }
}
