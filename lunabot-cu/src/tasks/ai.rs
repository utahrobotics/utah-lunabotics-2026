use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};

use crate::common::{FromAI, FromLunabase};

pub struct LunabotAI {

}

impl Freezable for LunabotAI {}

impl<'cl> CuTask<'cl> for LunabotAI {
    type Input = input_msg!('cl, Option<FromLunabase>);
    type Output = output_msg!('cl, Option<FromAI>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self {})
    }

    fn process(&mut self, clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()> {
        todo!()
    }
}