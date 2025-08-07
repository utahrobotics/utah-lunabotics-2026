use std::collections::VecDeque;

use bincode::{config::standard, decode_from_slice};
use common::{FromAI, LUNABOT_STAGE, Steering};
use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuMsg, CuSrcTask, Freezable},
    output_msg,
};
use embedded_common::ActuatorCommand;
use iceoryx_types::{FROM_AI_MAX_BYTES, FromAIBytes};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;

const FROM_AI_SERVICE: &str = "lunabot/ai_to_host";

pub struct AiSource {
    subscriber: Subscriber<ipc::Service, FromAIBytes, ()>,
    actuator_msg_queue: VecDeque<ActuatorCommand>,
    steering_msg_queue: VecDeque<Steering>,
}

impl Freezable for AiSource {}

impl CuSrcTask for AiSource {
    // (Steering, LiftAct, BucketAct) each Option<FromAI>
    type Output<'m> = output_msg!((Option<Steering>, Option<[u8; 5]>));

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("AiSource: node create", e))?;

        let service = node
            .service_builder(
                &ServiceName::new(FROM_AI_SERVICE)
                    .map_err(|e| CuError::new_with_cause("AiSource: invalid service name", e))?,
            )
            .publish_subscribe::<FromAIBytes>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("AiSource: service", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("AiSource: subscriber", e))?;

        Ok(Self {
            subscriber,
            actuator_msg_queue: VecDeque::new(),
            steering_msg_queue: VecDeque::new(),
        })
    }

    fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        // Drain the subscriber queue so we always act on the most recent message
        loop {
            match self
                .subscriber
                .receive()
                .map_err(|e| CuError::new_with_cause("AiSource: receive", e))?
            {
                Some(sample) => {
                    let payload: &FromAIBytes = &*sample;
                    let len = payload.len.min(FROM_AI_MAX_BYTES as u32) as usize;
                    let bytes = &payload.data[..len];

                    let config = standard();
                    if let Ok((msg, _)) = decode_from_slice::<FromAI, _>(bytes, config) {
                        if let FromAI::SetStage(stage) = msg {}
                        // Track latest actuator & general message separately so we can prioritise actuator commands.
                        match msg {
                            FromAI::SetActuators(actuator_cmd) => {
                                self.actuator_msg_queue.push_back(actuator_cmd);
                            }
                            FromAI::SetSteering(steering_cmd) => {
                                self.steering_msg_queue.push_back(steering_cmd);
                            }
                            FromAI::SetStage(stage) => {
                                LUNABOT_STAGE.store(stage);
                            }
                            _ => {}
                        }
                    }
                    // continue loop to see if there's an even newer message queued
                }
                None => break,
            }
        }
        // steering, actuators
        let mut payload = (None, None);
        if let Some(actuator_cmd) = self.actuator_msg_queue.pop_front() {
            payload.1 = Some(actuator_cmd.serialize());
        }
        if let Some(steering_cmd) = self.steering_msg_queue.pop_front() {
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
