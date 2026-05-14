use cu29::{cutask::CuSinkTask, prelude::*};

use crate::tasks::EncodableGetValuesResponse;

pub struct MotorLogger {

}

impl Freezable for MotorLogger {}

impl CuSinkTask for MotorLogger {
    type Input<'m> = input_msg!(EncodableGetValuesResponse);

    type Resources<'r> = ();

    fn new(_config: Option<&cu29::prelude::ComponentConfig>, _resources: Self::Resources<'_>) -> cu29::CuResult<Self>
    where
        Self: Sized {
        Ok(Self {  })
    }

    fn process<'i>(&mut self, _clock: &cu29::prelude::RobotClock, _input: &Self::Input<'i>) -> cu29::CuResult<()> {
        Ok(())
    }
}