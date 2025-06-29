use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSinkTask, Freezable}, prelude::*, CuResult};
use rerun::{components::RotationQuat, Points3D, Transform3D};

use cu_sensor_payloads::Distance;
use simple_motion::StaticNode;
use crate::{tasks::PointCloudPayload, ROOT_NODE};
use crate::rerun_viz;

pub struct L2PointCloudSink {
    lidar_node: StaticNode,
}

impl Freezable for L2PointCloudSink {}

impl<'cl> CuSinkTask<'cl> for L2PointCloudSink {
    type Input = input_msg!('cl, PointCloudPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {

        Ok(Self {
            lidar_node: ROOT_NODE.get().unwrap().clone().get_node_with_name("l2_front").unwrap()
        })
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        if let Some(payload) = new_msg.payload() {
            info!("Received {} points", payload.points.len());
            let iso = self.lidar_node.get_global_isometry();
            // Only log to rerun if we have points and rerun is available
            if !payload.points.is_empty() {
                if let Some(recorder_data) = rerun_viz::RECORDER.get() {
                    // Extract positions and intensities from the point cloud
                    let mut positions = Vec::with_capacity(payload.points.len());
                    let mut colors = Vec::with_capacity(payload.points.len());
                    
                    for i in 0..payload.points.len() {
                        // Convert Distance to meters (f32)
                        let Distance(x_length) = payload.points.x[i];
                        let Distance(y_length) = payload.points.y[i];
                        let Distance(z_length) = payload.points.z[i];
                        
                        let x = x_length.value;
                        let y = y_length.value;
                        let z = z_length.value;
                        
                        positions.push([x, y, z]);

                        colors.push([0,255,0]);
                    }

                    if let Err(e) = recorder_data.recorder.log(
                        "lidar/pointcloud",
                        &rerun::Transform3D::from_translation_rotation(
                            iso.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                iso.rotation.as_vector().cast::<f32>().data.0[0]
                            )
                        )
                    ) {
                        warning!("failed to log pointcloud transformation: {}", e.to_string());
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