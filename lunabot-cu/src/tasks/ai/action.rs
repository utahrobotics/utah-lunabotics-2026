use std::time::Duration;

use bonsai_bt::Status::{self, *};
use common::{LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::{Actuator, ActuatorCommand, Direction};
use nalgebra::Vector2;
use rerun::Boxes2D;

use crate::{
    ROBOT_STATE,
    rerun_viz::RECORDER,
    tasks::ai::{
        behaviors::autonomy::navigate::NavigationGoal,
        blackboard::LunabotBlackboard,
        jobs::{
            dig_job, direction_from_path, dump_job, find_path_job, follow_path_job, rotation_shim,
        },
    },
    utils::{rwlock_read_unpoison, rwlock_write_unpoison},
};

static _PATHFINDING_GOAL: [f32; 2] = [5.843524, 1.4796992];

#[derive(Clone, Debug)]
pub enum LunabotAction {
    /// Sets a status message that is sent over to the lunabase.
    /// Dont be repeatedly calling this a million times per second because it will use bandwidth.
    SetBTStatusMsg(String),

    ResetAllObstacles,
    ResetLocalObstacles,

    /// returns Running if reset_map or reset_local_map are set to true in the shared blackboard,
    /// otherwise returns Success
    ObstacleResetRequested,

    /// returns Running if the latest local map is None, this is a way to basically wait until an occupancy grid map is published
    /// returns success if latest local map is Some
    LatestLocalMapReady,

    Yield,
    SetSteering(Steering),

    /// sets steering to last known value rx'd from lunabase
    SetLastSteering,

    /// sets lift to last known value rx'd from lunabase
    SetLastLift,

    /// sets bucket to last known value rx'd from lunabase
    SetLastBucket,

    /// sets dumper to last known value rx'd from lunabase
    SetLastDumper,

    SetLift(i8),
    SetBucket(i8),
    SetDumper(i8),

    /// Set lift arm to a target angle (radians)
    SetLiftAngle(f32),
    /// Set bucket to a target angle (radians)
    SetBucketAngle(f32),
    /// Set dumper to a target angle (radians)
    SetDumperAngle(f32),

    // actions for checking the lunabot stage (dig, dump, manual, soft stop, navigate)
    IsSoftStop,
    IsAutonomy,
    IsManual,
    IsTestMotors,

    // These are used if we manually trigger a dig or dump, they dont involve traversal
    IsDig,
    IsDump,

    None,

    /// if the robot is in an occupied cell, we should first pathfind to the nearest free cell (if a free cell is within some range)
    IsInOccupiedCell,
    IsInFreeCell,

    /// Success if the robot has reached destination, Running if the robot is currently navigating, Failiure if the robot got stuck
    CheckNavigation,

    /// if the robot is in a cell of unknown status, we should first pathfind to the nearest free cell (if a free cell is within some range)
    IsInUnknownCell,

    /// calculates path from the robots position to x,y
    CalculatePath(NavigationGoal),
    FollowPath,
    SetStage(LunabotStage),
    GetUnstuck,

    // dig up some moon dirt
    Dig,
    Dump,
    /// Rotates to reach a target yaw. (in degrees)
    RotateTo(f32),

    /// Cancels long running jobs like pathfinding and path following
    CancelJobs,
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
                if let Some(last_steer_pack_time) = blackboard.last_non_zero_steering_pack {
                    let elapsed = last_steer_pack_time.elapsed();
                    if elapsed > Duration::from_millis(200) {
                        blackboard.last_non_zero_steering_pack = None;
                        eprintln!("[!!!MOTOR_ERROR!!!] DROPPED STEERING PACKET, SETTING TO ZERO");
                        // currently commented out because it was working commented out and further down the line in motors.rs is the timeout safety feature so I just dont want to touch this
                        // blackboard.outgoing_steering_msg = Some(Steering::new(0.0, 0.0, 0.0));
                        return (Success, 0.0);
                    }
                }
                if let Some(steering) = blackboard.last_steering.take() {
                    blackboard.outgoing_steering_msg = Some(steering);
                }

                Success
            }
            LunabotAction::SetLastBucket => {
                if let Some(last_bucket_pack_time) = blackboard.last_non_zero_bucket_pack {
                    if last_bucket_pack_time.elapsed() > Duration::from_millis(200) {
                        eprintln!("safety: lunabase silent, forcing bucket to stop");
                        blackboard.last_non_zero_bucket_pack = None;
                        blackboard.last_bucket = None;
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(actuator_command_from_i8(0, Actuator::Bucket));
                        return (Success, 0.0);
                    }
                }

                if let Some(value) = blackboard.last_bucket.take() {
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(actuator_command_from_i8(value, Actuator::Bucket));
                }

                Success
            }
            LunabotAction::SetLastDumper => {
                if let Some(last_dumper_pack_time) = blackboard.last_non_zero_dumper_pack {
                    if last_dumper_pack_time.elapsed() > Duration::from_millis(200) {
                        eprintln!("safety: lunabase silent, forcing lift to stop");
                        blackboard.last_non_zero_dumper_pack = None;
                        blackboard.last_dumper = None;
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(actuator_command_from_i8(0, Actuator::Dumper));
                        return (Success, 0.0);
                    }
                }

                if let Some(value) = blackboard.last_dumper.take() {
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(actuator_command_from_i8(value, Actuator::Dumper));
                }

                Success
            }

            LunabotAction::SetLastLift => {
                if let Some(last_lift_pack_time) = blackboard.last_non_zero_lift_pack {
                    if last_lift_pack_time.elapsed() > Duration::from_millis(200) {
                        println!("safety: lunabase silent, forcing lift to stop");
                        blackboard.last_non_zero_lift_pack = None;
                        blackboard.last_lift = None;
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(actuator_command_from_i8(0, Actuator::Lift));
                        return (Success, 0.0);
                    }
                }

                if let Some(value) = blackboard.last_lift.take() {
                    blackboard
                        .outgoing_actuator_msg_queue
                        .push_back(actuator_command_from_i8(value, Actuator::Lift));
                }

                Success
            }
            LunabotAction::SetBucket(value) => {
                blackboard.last_bucket = None;
                blackboard
                    .outgoing_actuator_msg_queue
                    .push_back(actuator_command_from_i8(*value, Actuator::Bucket));
                Success
            }
            LunabotAction::SetLift(value) => {
                blackboard.last_lift = None;
                blackboard
                    .outgoing_actuator_msg_queue
                    .push_back(actuator_command_from_i8(*value, Actuator::Lift));
                Success
            }
            LunabotAction::SetDumper(value) => {
                blackboard.last_dumper = None;
                blackboard
                    .outgoing_actuator_msg_queue
                    .push_back(actuator_command_from_i8(*value, Actuator::Dumper));
                Success
            }
            LunabotAction::SetLiftAngle(angle) => {
                blackboard
                    .outgoing_actuator_msg_queue
                    .push_back(ActuatorCommand::set_angle(Actuator::Lift, *angle));
                Success
            }
            LunabotAction::SetBucketAngle(angle) => {
                blackboard
                    .outgoing_actuator_msg_queue
                    .push_back(ActuatorCommand::set_angle(Actuator::Bucket, *angle));
                Success
            }
            LunabotAction::SetDumperAngle(angle) => {
                blackboard
                    .outgoing_actuator_msg_queue
                    .push_back(ActuatorCommand::set_angle(Actuator::Dumper, *angle));
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
            LunabotAction::IsTestMotors => match blackboard.current_mission {
                LunabotStage::TestMotors => Running,
                _ => Success,
            },
            LunabotAction::IsDig => match blackboard.current_mission {
                LunabotStage::Dig => Running,
                _ => Success,
            },
            LunabotAction::IsDump => match blackboard.current_mission {
                LunabotStage::Dump => Running,
                _ => Success,
            },
            LunabotAction::None => Success,
            LunabotAction::IsInOccupiedCell => todo!(),
            LunabotAction::IsInFreeCell => todo!(),
            LunabotAction::IsInUnknownCell => todo!(),
            LunabotAction::CalculatePath(navigation_goal) => {
                let (center, hw, hh) = navigation_goal.to_center_and_halfsizes();
                if let Some(rec) = RECORDER.get() {
                    let _ = rec.recorder.log(
                        "ai/goal",
                        &Boxes2D::from_centers_and_half_sizes(
                            vec![(center.x, center.y)],
                            vec![(hw, hh)],
                        ),
                    );
                }
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
                    // PATHFINDING_GOAL is just for testing for now.

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
                                blackboard.calculated_path = Some(path.clone());
                                blackboard.path_finder = None;
                                if let Some(ref follower) = blackboard.path_follower {
                                    if let Err(e) = follower.send_to_job(path) {
                                        eprintln!(
                                            "Failed to send new path to existing follower: {e}"
                                        );
                                    }
                                }
                                Success
                            } else {
                                eprintln!("Path finder job succeeded but produced no output.");
                                blackboard.path_finder = None;
                                Failure
                            }
                        } else if status == Failure {
                            eprintln!("Path finder job failed.");
                            blackboard.path_finder = None;
                            Failure
                        } else {
                            // Still running
                            Running
                        }
                    } else {
                        // Start a new path finder job
                        // println!("Starting path finder job from {:?} to {:?}.", start, end);
                        let mut job = find_path_job(
                            local_map.clone(),
                            start,
                            blackboard.obstacle_gradient_threshold_expander,
                            blackboard.obstacle_gradient_threshold_pathfinder,
                            blackboard.robot_radius,
                            *navigation_goal,
                        );
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
                        let mut follower_job = follow_path_job(
                            ROBOT_STATE.get().unwrap().kinematic_root,
                            path,
                            None,
                            0.85,
                            0.05,
                            None,
                            None,
                            None,
                        );
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
            LunabotAction::Dig => {
                // Check if we already got a digging job going
                if let Some(ref mut digger) = blackboard.digger {
                    while let Some(command) = digger.get_output() {
                        blackboard.outgoing_actuator_msg_queue.push_back(command);
                    }

                    let status = digger.get_status();
                    if status == Success {
                        // We (hopefully) have a bucket full of moon dirt now
                        Success
                    } else if status == Failure {
                        // Somehow we managed to fuck this one up too
                        eprintln!("Failed Digging job!");
                        blackboard.digger = None;
                        Failure
                    } else {
                        // Still digging
                        Running
                    }
                } else {
                    // Start a new digging job
                    let mut job = dig_job();
                    let initial_status: Status = job.get_status();
                    blackboard.digger = Some(job);
                    println!(
                        "Digging job started with intial status {:?}",
                        initial_status
                    );

                    initial_status
                }
            }
            LunabotAction::Dump => {
                // Check if we already got a dumping job going
                if let Some(ref mut dumper) = blackboard.dumper {
                    while let Some(command) = dumper.get_output() {
                        blackboard.outgoing_actuator_msg_queue.push_back(command);
                    }

                    let status = dumper.get_status();
                    if status == Success {
                        Success
                    } else if status == Failure {
                        //
                        eprintln!("Failed Dumping job!");
                        blackboard.dumper = None;
                        Failure
                    } else {
                        // Still digging
                        Running
                    }
                } else {
                    // Start a new digging job
                    let mut job = dump_job();
                    let initial_status: Status = job.get_status();
                    blackboard.dumper = Some(job);
                    println!(
                        "Dumping job started with intial status {:?}",
                        initial_status
                    );

                    initial_status
                }
            }
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
                blackboard.current_mission = *stage;
                blackboard.path_follower = None;
                LUNABOT_STAGE.store(*stage);
                Success
            }
            LunabotAction::CancelJobs => {
                if let Some(ref mut pathfinder) = blackboard.path_finder {
                    pathfinder.cancel();
                    blackboard.path_finder = None;
                }
                if let Some(ref mut pathfollower) = blackboard.path_follower {
                    pathfollower.cancel();
                    blackboard.path_follower = None;
                }
                Success
            }
            LunabotAction::RotateTo(target_yaw) => {
                if ROBOT_STATE.get().is_none() {
                    eprintln!(
                        "Cannot start rotation shim job because ROBOT_STATE is not initialized"
                    );
                    Failure
                } else if let Some(ref mut rotation_shim) = blackboard.rotation_shim {
                    blackboard.outgoing_steering_msg = rotation_shim.get_output();
                    let status = rotation_shim.get_status();
                    if status == Success || status == Failure {
                        println!(
                            "Rotate to face path job completed with status: {:?}",
                            status
                        );
                        blackboard.rotation_shim = None;
                    }
                    status
                } else {
                    // uncomment to get yaw to face general direction the path wants you to go
                    // let Some(target_yaw) = direction_from_path(path) else {
                    //     eprintln!("Calculated path has < 2 nodes");
                    //     return (Failure, 0.0);
                    // };
                    let mut rotation_shim = rotation_shim(*target_yaw, 0.1, None, None, None);
                    let job_initial_status = rotation_shim.get_status();
                    blackboard.rotation_shim = Some(rotation_shim);
                    println!(
                        "Face path job started with initial status: {:?}",
                        job_initial_status
                    );
                    job_initial_status
                }
            }
            LunabotAction::ResetAllObstacles => {
                blackboard.latest_local_map = None;
                rwlock_write_unpoison(&*blackboard.blackboard_shared).reset_map = true;
                Success
            }
            LunabotAction::ResetLocalObstacles => {
                blackboard.latest_local_map = None;
                rwlock_write_unpoison(&*blackboard.blackboard_shared).reset_local_map = true;
                Success
            }
            LunabotAction::ObstacleResetRequested => {
                let guard = rwlock_read_unpoison(&*blackboard.blackboard_shared);
                if guard.reset_local_map || guard.reset_map {
                    Running
                } else {
                    Success
                }
            }
            LunabotAction::LatestLocalMapReady => {
                if blackboard.latest_local_map.is_some() {
                    Success
                } else {
                    Running
                }
            }
            LunabotAction::SetBTStatusMsg(msg) => {
                blackboard.outgoing_bt_status_msg = Some(msg.to_owned());
                Success
            }
        };
        (status, 0.0)
    }
}

fn actuator_command_from_i8(value: i8, actuator: Actuator) -> ActuatorCommand {
    if value < 0 {
        ActuatorCommand::set_speed(value as f64 / i8::MIN as f64, actuator, Direction::Backward)
    } else {
        ActuatorCommand::set_speed(value as f64 / i8::MAX as f64, actuator, Direction::Forward)
    }
}
