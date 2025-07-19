use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSinkTask, Freezable}, prelude::*, CuResult};

use common::FromAI;
use embedded_common::ActuatorCommand;

pub struct ActuatorController;

impl Freezable for ActuatorController {}

impl<'cl> CuSinkTask<'cl> for ActuatorController {
    type Input = input_msg!('cl, (Option<FromAI>, Option<FromAI>, Option<FromAI>));

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, clock: &RobotClock, input: Self::Input) -> CuResult<()> {        
        if let Some(payload) = input.payload() {
            if let Some(FromAI::SetActuators(cmd)) = &payload.1 {
                info!("ActuatorController: lift command {}", cmd.to_string());
            }
            if let Some(FromAI::SetActuators(cmd)) = &payload.2 {
                info!("ActuatorController: bucket command {}", cmd.to_string());
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