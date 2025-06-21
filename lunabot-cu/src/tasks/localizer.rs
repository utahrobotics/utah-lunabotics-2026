use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSinkTask, CuTask, Freezable}, input_msg, output_msg, CuResult};
use cu_spatial_payloads::Transform3D;
use crate::common::IMUReading;

pub struct CuLocalizer {

}


impl Freezable for CuLocalizer {}


impl<'cl> CuSinkTask<'cl> for CuLocalizer {
    /// IMU and estimated observer isometry from apriltags
    type Input = input_msg!('cl, Transform3D<f64>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where Self: Sized {
        Ok(
            CuLocalizer {  }
        )
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
    ) -> CuResult<()> {

        Ok(())
    }
}