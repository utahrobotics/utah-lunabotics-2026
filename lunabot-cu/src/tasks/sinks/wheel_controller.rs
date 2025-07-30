use cu29::{
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSinkTask, Freezable},
    prelude::*,
    CuResult,
};

use common::{FromAI, Steering};

pub struct WheelController;

impl Freezable for WheelController {}

impl CuSinkTask for WheelController {
    type Input<'m> = input_msg!((Option<FromAI>, Option<FromAI>, Option<FromAI>));

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            if let Some(FromAI::SetSteering(steer)) = &payload.0 {
                info!("WheelController: steering {}", steer.get_left_and_right());
            }
        }
        Ok(())
    }
}
