use std::error::Error;

use cu29::{
    CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSinkTask, Freezable},
    prelude::*,
};

use common::{FromAI, Steering};
use embedded_common::ActuatorCommand;

use crate::PICO_TX;

pub struct ActuatorController;

impl Freezable for ActuatorController {}

impl CuSinkTask for ActuatorController {
    // steering, actuators (just ignore the steering for now)
    // actuator command is a slice because the ActuatorCommand doesnt implement Serialize or Decode due to being no_std
    type Input<'m> = input_msg!((Option<Steering>, Option<[u8; 5]>));

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(payload) = input.payload()
            && let Some(cmd_bytes) = &payload.1
            && let Some(pico_tx) = PICO_TX.get()
            && let Ok(actuator_cmd) = ActuatorCommand::deserialize(*cmd_bytes)
        {
            if pico_tx.is_full() {
                return Err(
                    CuError::new_with_cause("pico crossbeam thread channel is full", std::io::Error::other("channel full"))
                );
            }
            if let Err(err) = pico_tx.send(actuator_cmd) {
                error!("Failed to send actuator command: {}", err.to_string());
            }
        }
        Ok(())
    }
}

trait ToStringExt {
    fn to_string(&self) -> String;
}

impl ToStringExt for ActuatorCommand {
    fn to_string(&self) -> String {
        format!("ActuatorCommand: {:?}", self)
    }
}
