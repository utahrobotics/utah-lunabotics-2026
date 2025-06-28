use cu29::{cutask::{CuMsg, CuTask, Freezable}, input_msg, output_msg};
use cu_spatial_payloads::Transform3D;

use crate::tasks::PointCloudPayload;

pub struct KissIcp {
    
}

impl Freezable for KissIcp {}

impl<'cl> CuTask<'cl> for KissIcp {
    type Input = input_msg!('cl, PointCloudPayload);

    type Output = output_msg!('cl, Transform3D<f64>);

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized {
        todo!()
    }

    fn preprocess(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
        todo!()
    }

    fn process(
        &mut self,
        _clock: &cu29::prelude::RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> cu29::CuResult<()> {
        todo!()
    }

    fn postprocess(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
        todo!()
    }
}