use common::{FromAI, LUNABOT_STAGE};
use bincode::{config::standard, decode_from_slice};
use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use iceoryx_types::{FromAIBytes, FROM_AI_MAX_BYTES};

const FROM_AI_SERVICE: &str = "lunabot/ai_to_host";

pub struct AiSource {
    subscriber: Subscriber<ipc::Service, FromAIBytes, ()>,
}

impl Freezable for AiSource {}

impl<'cl> CuSrcTask<'cl> for AiSource {
    // (Steering, LiftAct, BucketAct) each Option<FromAI>
    type Output = output_msg!('cl, (Option<FromAI>, Option<FromAI>, Option<FromAI>));

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("AiSource: node create", e))?;

        let service = node
            .service_builder(&ServiceName::new(FROM_AI_SERVICE).map_err(|e| CuError::new_with_cause("AiSource: invalid service name", e))?)
            .publish_subscribe::<FromAIBytes>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("AiSource: service", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("AiSource: subscriber", e))?;

        Ok(Self { subscriber })
    }

    fn process(&mut self, clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        let start = clock.now().as_nanos();
        
        // Drain the subscriber queue so we always act on the most recent message
        let mut steering_msg: Option<FromAI> = None;
        let mut lift_msg: Option<FromAI> = None;
        let mut bucket_msg: Option<FromAI> = None;
        loop {
            match self.subscriber.receive().map_err(|e| CuError::new_with_cause("AiSource: receive", e))? {
                Some(sample) => {
                    let payload: &FromAIBytes = &*sample;
                    let len = payload.len.min(FROM_AI_MAX_BYTES as u32) as usize;
                    let bytes = &payload.data[..len];

                    let config = standard();
                    if let Ok((msg, _)) = decode_from_slice::<FromAI, _>(bytes, config) {
                        if let FromAI::SetStage(stage) = msg {
                            // Keep global stage in sync – important for Ping packets.
                            LUNABOT_STAGE.store(stage);
                        }
                        // Track latest actuator & general message separately so we can prioritise actuator commands.
                        match msg {
                            FromAI::SetActuators(cmd) => {
                                use embedded_common::Actuator::{Lift, Bucket};
                                match cmd {
                                    embedded_common::ActuatorCommand::SetSpeed(_, act) => match act {
                                        Lift => lift_msg = Some(FromAI::SetActuators(cmd)),
                                        Bucket => bucket_msg = Some(FromAI::SetActuators(cmd)),
                                    },
                                    embedded_common::ActuatorCommand::SetDirection(_, act) => match act {
                                        Lift => lift_msg = Some(FromAI::SetActuators(cmd)),
                                        Bucket => bucket_msg = Some(FromAI::SetActuators(cmd)),
                                    },
                                    embedded_common::ActuatorCommand::Shake => lift_msg = Some(FromAI::SetActuators(cmd)),
                                    embedded_common::ActuatorCommand::StartPercuss | embedded_common::ActuatorCommand::StopPercuss => bucket_msg = Some(FromAI::SetActuators(cmd)),
                                }
                            },
                            FromAI::SetSteering(_) => steering_msg = Some(msg),
                            _ => {},
                        }
                    }
                    // continue loop to see if there's an even newer message queued
                }
                None => break,
            }
        }

        // Compose tuple (steering, lift, bucket)
        if steering_msg.is_some() || lift_msg.is_some() || bucket_msg.is_some() {
            output.set_payload((steering_msg, lift_msg, bucket_msg));
        } else {
            output.clear_payload();
        }
        
        Ok(())
    }
}