use cu29::{
    CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSinkTask, Freezable},
    prelude::*,
};

use common::Steering;

pub struct WheelController;

impl Freezable for WheelController {}

impl CuSinkTask for WheelController {
    // steering, actuators (just ignore the actuators here for now)
    type Input<'m> = input_msg!((Option<Steering>, Option<[u8; 5]>));

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            if let Some(steering) = &payload.0 {
                info!(
                    "WheelController: steering {}",
                    steering.get_left_and_right()
                );
            }
        }
        Ok(())
    }
}
