use bonsai_bt::Behavior::{self, Action};
use bonsai_bt::Status::{Failure, Running, Success};
use bonsai_bt::{BT, Event, Status, UpdateArgs};
use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use cu29::prelude::*;
use cu29::{
    cutask::{CuTask, Freezable},
    input_msg,
};
use embedded_common::{Actuator, ActuatorCommand};

use std::sync::mpsc::Receiver;

use crate::tasks::ai::action::LunabotAction;
use crate::tasks::ai::behaviors::teleop::teleop_behavior;
use crate::tasks::ai::blackboard::LunabotBlackboard;
use crate::utils::nanos_to_secs;

pub struct LunabotAi {
    bt: BT<LunabotAction, LunabotBlackboard>,
    last_tick_nanos: u64,
}

impl Freezable for LunabotAi {}

impl CuTask for LunabotAi {
    type Input<'m> = input_msg!(Option<FromLunabase>);

    // (Steering, ActuatorCommand)
    type Output<'m> = output_msg!((Option<Steering>, Option<[u8; 5]>));

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let mut blackboard = LunabotBlackboard::default();
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
        let e: Event = UpdateArgs {
            dt: nanos_to_secs(clock.now().as_nanos() - self.last_tick_nanos),
        }
        .into();
        if let Some(Some(from_lunabase)) = input.payload() {
            self.bt.blackboard_mut().update_with_msg(from_lunabase);
        }

        self.bt.tick(&e, &mut |args, blackboard| {
            let mut status = Running;
            match *args.action {
                LunabotAction::SetSteering(steering) => {
                    blackboard.outgoing_steering_msg = Some(steering);
                    status = Success;
                }
                LunabotAction::SetLastSteering => {
                    if let Some(steering) = blackboard.last_steering {
                        blackboard.outgoing_steering_msg = Some(steering);
                        blackboard.last_steering = None;
                    }
                }
                LunabotAction::SetLastBucket => {
                    if let Some(value) = blackboard.last_bucket {
                        let commands = actuator_commands_from_i8(value, Actuator::Bucket);
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(commands[0]);
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(commands[1]);
                    }
                }
                LunabotAction::SetLastLift => {
                    if let Some(value) = blackboard.last_lift {
                        let commands = actuator_commands_from_i8(value, Actuator::Lift);
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(commands[0]);
                        blackboard
                            .outgoing_actuator_msg_queue
                            .push_back(commands[1]);
                    }
                }
                LunabotAction::SetBucket(value) => {
                    let [direction, speed] = actuator_commands_from_i8(value, Actuator::Bucket);
                    blackboard.last_bucket = None;
                    blackboard.outgoing_actuator_msg_queue.push_back(direction);
                    blackboard.outgoing_actuator_msg_queue.push_back(speed);
                }
                LunabotAction::SetLift(value) => {
                    let [direction, speed] = actuator_commands_from_i8(value, Actuator::Lift);
                    blackboard.last_lift = None;
                    blackboard.outgoing_actuator_msg_queue.push_back(direction);
                    blackboard.outgoing_actuator_msg_queue.push_back(speed);
                }
                LunabotAction::IsSoftStop => match LUNABOT_STAGE.load() {
                    LunabotStage::SoftStop => status = Running,
                    _ => status = Success,
                },
                LunabotAction::IsAutonomy => match LUNABOT_STAGE.load() {
                    LunabotStage::Autonomy => status = Running,
                    _ => status = Success,
                },
                LunabotAction::IsManual => match LUNABOT_STAGE.load() {
                    LunabotStage::Manual => status = Running,
                    _ => status = Success,
                },
                LunabotAction::None => status = Success,
                LunabotAction::IsObstacleMapReady => {
                    if blackboard.latest_obstacle_map.is_some() {
                        status = Success;
                    } else {
                        status = Failure;
                    }
                }
                LunabotAction::IsInOccupiedCell => todo!(),
                LunabotAction::IsInFreeCell => todo!(),
                LunabotAction::IsInUnknownCell => todo!(),
                LunabotAction::CalculatePath => todo!(),
                LunabotAction::FollowPathFor(meters) => {
                    // this will be a long running task in a different thread
                    // the task will live in the jobs folder, and be pollable to get the Status
                    todo!()
                }
                LunabotAction::CheckNavigation => todo!(),
                LunabotAction::GetUnstuck => todo!(),
            }
            (status, args.dt)
        });

        // TODO: make sure backpressure isnt bad here, maybe we should drain the queue before advancing to the next tick of the blackboard?
        let mut payload = (None, None);
        if let Some(actuator_cmd) = self
            .bt
            .blackboard_mut()
            .outgoing_actuator_msg_queue
            .pop_front()
        {
            payload.1 = Some(actuator_cmd.serialize());
        }
        if let Some(steering_cmd) = self.bt.blackboard_mut().outgoing_steering_msg.take() {
            payload.0 = Some(steering_cmd);
        }
        if payload.0.is_some() || payload.1.is_some() {
            output.set_payload(payload);
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
