use cu29::cutask::CuMsg;
use cu29::{
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSinkTask, Freezable},
    input_msg,
    prelude::*,
    CuResult,
};
use cu_apriltag::AprilTagDetections;

// Sink that logs AprilTag detections
#[derive(Default)]
pub struct DetectionLogger;

impl Freezable for DetectionLogger {}

impl<'cl> CuSinkTask<'cl> for DetectionLogger {
    // one detections struct per camera
    type Input = (
        input_msg!('cl, AprilTagDetections),
        input_msg!('cl, AprilTagDetections),
    );

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let (input1, input2) = input;

        if let Some(dets) = input1.payload() {
            for (id, pose, _) in dets.filtered_by_decision_margin(50.0) {
                //TODO: poses still are garbage from that one godforsaken bug
                info!(
                    "[{}] Detected tag {} with pose: {}",
                    &dets.camera_id, id, pose
                );
            }
        }
        if let Some(dets) = input2.payload() {
            for (id, pose, _) in dets.filtered_by_decision_margin(50.0) {
                //TODO: poses still are garbage from that one godforsaken bug
                info!(
                    "[{}] Detected tag {} with pose: {}",
                    &dets.camera_id, id, pose
                );
            }
        }

        Ok(())
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
