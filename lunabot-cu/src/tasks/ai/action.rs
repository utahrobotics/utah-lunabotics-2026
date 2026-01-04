use bonsai_bt::Status::{self, *};
use common::{LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::{Actuator, ActuatorCommand};
use nalgebra::Vector2;
use rerun::Vec3D;

use crate::{
    ROBOT_STATE,
    tasks::ai::{
        blackboard::LunabotBlackboard,
        jobs::{find_path_job, follow_path_job},
    },
};
static PATHFINDING_GOAL: [f32; 2] = [1.490662524, 1.22606992];

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
                if let Some(ref local_map) = blackboard.latest_local_map {
                    // if the kinematic root is not initialized, we might as well just blow up because nothing will work anyways
                    let current_translation = ROBOT_STATE
                        .get()
                        .unwrap()
                        .kinematic_root
                        .get_global_isometry()
                        .translation;

                    let start =
                        Vector2::new(current_translation.x as f32, current_translation.y as f32);

                    // Get destination from blackboard, or use default PATHFINDING_GOAL
                    let end = if let Some((x, y)) = blackboard.navigate_destination {
                        Vector2::new(x, y)
                    } else {
                        Vector2::new(PATHFINDING_GOAL[0], PATHFINDING_GOAL[1])
                    };

                    // Check if we already have a path finder job running
                    if let Some(ref mut path_finder) = blackboard.path_finder {
                        let status = path_finder.get_status();
                        if status == Success {
                            // Job completed successfully, get the path
                            if let Some(path) = path_finder.get_output() {
                                println!(
                                    "Path calculation completed with {} waypoints",
                                    path.len()
                                );
                                blackboard.calculated_path = Some(path);
                                blackboard.path_finder = None;
                                Success
                            } else {
                                eprintln!("Path finder job succeeded but produced no output");
                                blackboard.path_finder = None;
                                Failure
                            }
                        } else if status == Failure {
                            eprintln!("Path finder job failed");
                            blackboard.path_finder = None;
                            Failure
                        } else {
                            // Still running
                            Running
                        }
                    } else {
                        // Start a new path finder job
                        println!("Starting path finder job from {:?} to {:?}", start, end);
                        let mut job = find_path_job(local_map.clone(), start, end);
                        let initial_status = job.get_status();
                        blackboard.path_finder = Some(job);
                        println!(
                            "Path finder job started with initial status: {:?}",
                            initial_status
                        );
                        initial_status
                    }
                } else {
                    eprintln!("Cannot calculate path: no local map available");
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
                    // Use the calculated path from CalculatePath action
                    if let Some(path) = blackboard.calculated_path.take() {
                        println!("Starting new follow path job with {} waypoints", path.len());
                        let mut follower_job =
                            follow_path_job(5.0, ROBOT_STATE.get().unwrap().kinematic_root, path);
                        let job_initial_status = follower_job.get_status();
                        blackboard.path_follower = Some(follower_job);
                        println!(
                            "Follow path job started with initial status: {:?}",
                            job_initial_status
                        );
                        job_initial_status
                    } else {
                        eprintln!("Cannot follow path: no calculated path available");
                        Failure
                    }
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
