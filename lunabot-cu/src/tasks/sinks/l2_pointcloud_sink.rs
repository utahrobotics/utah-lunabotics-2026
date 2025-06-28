use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSinkTask, Freezable}, prelude::*, CuResult};
use rerun::{Points3D, Transform3D};

use cu_sensor_payloads::Distance;
use crate::tasks::PointCloudPayload;
use crate::rerun_viz;

pub struct L2PointCloudSink {}

impl Freezable for L2PointCloudSink {}

impl<'cl> CuSinkTask<'cl> for L2PointCloudSink {
    type Input = input_msg!('cl, PointCloudPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        if let Some(payload) = new_msg.payload() {
            info!("Received {} points", payload.len());
            
            // Only log to rerun if we have points and rerun is available
            if !payload.is_empty() {
                if let Some(recorder_data) = rerun_viz::RECORDER.get() {
                    // Extract positions and intensities from the point cloud
                    let mut positions = Vec::with_capacity(payload.len());
                    let mut colors = Vec::with_capacity(payload.len());
                    
                    for i in 0..payload.len() {
                        // Convert Distance to meters (f32)
                        let Distance(x_length) = payload.x[i];
                        let Distance(y_length) = payload.y[i];
                        let Distance(z_length) = payload.z[i];
                        
                        let x = x_length.value;
                        let y = y_length.value;
                        let z = z_length.value;
                        
                        positions.push([x, y, z]);

                        colors.push([0,255,0]);
                    }

                    // Log the point cloud to Rerun
                    if let Err(e) = recorder_data.recorder.log(
                        "lidar/pointcloud",
                        &Points3D::new(positions)
                            .with_colors(colors)
                            .with_radii([0.01f32])
                    ) {
                        warning!("Failed to log point cloud to Rerun: {}", format!("{:?}", e));
                    }
                } else {
                    debug!("Rerun recorder not available");
                }
            }
        } else {
            // info!("No points received");
        }
        Ok(())
    }
}