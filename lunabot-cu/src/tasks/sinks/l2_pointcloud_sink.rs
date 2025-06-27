use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSinkTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use cu_sensor_payloads::{PointCloud, PointCloudSoa};

use crate::tasks::PointCloudPayload;

pub struct L2PointCloudSink;

impl Freezable for L2PointCloudSink {}

impl<'cl> CuSinkTask<'cl> for L2PointCloudSink {
    type Input = input_msg!('cl, PointCloudPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        if let Some(payload) = new_msg.payload() {
            info!("Received {} points", payload.len());
        } else {
            info!("No points received");
        }
        Ok(())
    }
}