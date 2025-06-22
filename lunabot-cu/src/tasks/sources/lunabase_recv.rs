use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable, CuMsg}, output_msg, CuResult};

use crate::common::FromLunabase;



pub struct LunabaseReceiver;

impl Freezable for LunabaseReceiver{}

impl<'cl> CuSrcTask<'cl> for LunabaseReceiver {
    type Output = output_msg!('cl, Option<FromLunabase>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized {
        todo!()
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        todo!()
    }
}