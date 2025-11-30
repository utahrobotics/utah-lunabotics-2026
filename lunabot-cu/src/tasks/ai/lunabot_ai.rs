use bonsai_bt::Status::{Failure, Running, Success};
use bonsai_bt::{BT, Event, UpdateArgs};
use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use cu29::prelude::*;
use cu29::{
    cutask::{CuTask, Freezable},
    input_msg,
};
use embedded_common::{Actuator, ActuatorCommand};

use std::sync::mpsc::Receiver;

use crate::ROBOT_STATE;
use crate::tasks::OccupancyGrid;
use crate::tasks::ai::action::LunabotAction;
use crate::tasks::ai::behaviors::teleop::teleop_behavior;
use crate::tasks::ai::blackboard::LunabotBlackboard;
use crate::tasks::ai::jobs::follow_path_job;
use crate::utils::nanos_to_secs;

pub struct LunabotAi {
    bt: BT<LunabotAction, LunabotBlackboard>,
    last_tick_nanos: u64,
}

impl Freezable for LunabotAi {}

impl CuTask for LunabotAi {
    type Input<'m> = input_msg!('m, Option<FromLunabase>, OccupancyGrid);

    // (Steering, ActuatorCommand)
    type Output<'m> = output_msg!((Option<Steering>, Option<ActuatorCommand>));

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let blackboard = LunabotBlackboard::default();
        let behavior = teleop_behavior();
        let mut bt = BT::new(behavior, blackboard);
        let viz = bt.get_graphviz();
        println!("GRAPHVIZ: {viz}");
        Ok(Self {
            bt: bt,
            last_tick_nanos: 0,
        })
    }

    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        self.last_tick_nanos = clock.now().into();
        Ok(())
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        let dt = nanos_to_secs(clock.now().as_nanos() - self.last_tick_nanos);

        let e: Event = UpdateArgs { dt }.into();
        if let Some(Some(from_lunabase)) = input.0.payload() {
            self.bt.blackboard_mut().update_with_msg(from_lunabase);
        }

        if let Some(_) = input.1.payload() {
            // info!("got occupancy grid in the ai");
        }

        let mut remaining_dt = 0.0;
        self.bt.tick(&e, &mut |args, blackboard| {
            let status = match *args.action {
                LunabotAction::SetSteering(steering) => {
                    blackboard.outgoing_steering_msg = Some(steering);
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
                    let [direction, speed] = actuator_commands_from_i8(value, Actuator::Bucket);
                    blackboard.last_bucket = None;
                    blackboard.outgoing_actuator_msg_queue.push_back(direction);
                    blackboard.outgoing_actuator_msg_queue.push_back(speed);
                    Success
                }
                LunabotAction::SetLift(value) => {
                    let [direction, speed] = actuator_commands_from_i8(value, Actuator::Lift);
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
                LunabotAction::CalculatePath => Success,
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
                    blackboard.current_mission = stage;
                    blackboard.path_follower = None;
                    LUNABOT_STAGE.store(stage);
                    Success
                }
            };
            (status, remaining_dt) //passing 0.0 consumes all the remaining time for a tick
        });
        self.last_tick_nanos = clock.now().into();
        let mut payload = (None, None);
        if let Some(actuator_cmd) = self
            .bt
            .blackboard_mut()
            .outgoing_actuator_msg_queue
            .pop_front()
        {
            payload.1 = Some(actuator_cmd);
        }
        if let Some(steering_cmd) = self.bt.blackboard_mut().outgoing_steering_msg.take() {
            payload.0 = Some(steering_cmd);
        }
        if payload.0.is_some() || payload.1.is_some() {
            output.set_payload(payload);
            return Ok(());
        } else {
            output.clear_payload();
        }

        Ok(())
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
