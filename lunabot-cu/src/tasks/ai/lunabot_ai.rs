use bonsai_bt::Behavior::{self, Action};
use bonsai_bt::Status::{Running, Success};
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
    type Input<'m> = input_msg!(FromLunabase);

    // (Steering, ActuatorCommand)
    type Output<'m> = output_msg!((Option<Steering>, Option<[u8; 5]>));

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let mut blackboard = LunabotBlackboard::default();
        let behavior = teleop_behavior(&mut blackboard);
        Ok(Self {
            bt: BT::new(behavior, blackboard),
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
        if let Some(from_lunabase) = input.payload() {
            self.bt.blackboard_mut().update_with_msg(from_lunabase);
        }

        self.bt.tick(&e, &mut |args, blackboard| {
            let mut status = Running;
            match *args.action {
                LunabotAction::SetSteering(steering) => {
                    blackboard.outgoing_steering_msg = Some(steering);
                    status = Success;
                }
                LunabotAction::SetBucket(value) => {
                    let [direction, speed] = if value < 0 {
                        [
                            ActuatorCommand::backward(Actuator::Bucket),
                            ActuatorCommand::set_speed(
                                value as f64 / i8::MIN as f64,
                                Actuator::Bucket,
                            ),
                        ]
                    } else {
                        [
                            ActuatorCommand::forward(Actuator::Bucket),
                            ActuatorCommand::set_speed(
                                value as f64 / i8::MAX as f64,
                                Actuator::Bucket,
                            ),
                        ]
                    };
                    blackboard.last_bucket = None;
                    blackboard.outgoing_actuator_msg_queue.push_back(direction);
                    blackboard.outgoing_actuator_msg_queue.push_back(speed);
                }
                LunabotAction::SetLift(value) => {
                    let [direction, speed] = if value < 0 {
                        [
                            ActuatorCommand::backward(Actuator::Lift),
                            ActuatorCommand::set_speed(
                                value as f64 / i8::MIN as f64,
                                Actuator::Lift,
                            ),
                        ]
                    } else {
                        [
                            ActuatorCommand::forward(Actuator::Lift),
                            ActuatorCommand::set_speed(
                                value as f64 / i8::MAX as f64,
                                Actuator::Lift,
                            ),
                        ]
                    };
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
