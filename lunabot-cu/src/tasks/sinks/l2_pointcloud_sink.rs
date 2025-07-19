use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSinkTask, Freezable}, prelude::*, CuResult};
use iceoryx_types::IceoryxPointCloud;
use nalgebra::Isometry3;
use rerun::{components::RotationQuat, Points3D, Transform3D};

use cu_sensor_payloads::Distance;
use simple_motion::StaticNode;
use crate::{ROOT_NODE};
use crate::rerun_viz;

pub struct L2PointCloudSink {
    lidar_node: StaticNode,
}

impl Freezable for L2PointCloudSink {}

impl<'cl> CuSinkTask<'cl> for L2PointCloudSink {
    type Input = input_msg!('cl, IceoryxPointCloud);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {

        Ok(Self {
            lidar_node: ROOT_NODE.get().unwrap().clone().get_node_with_name("l2_front").unwrap()
        })
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        if let Some(payload) = new_msg.payload() {
            info!("Received {} points", payload.publish_count);
            // Only log to rerun if we have points and rerun is available
            if !payload.points.is_empty() {
                if let Some(recorder_data) = rerun_viz::RECORDER.get() {
                    // Extract positions and intensities from the point cloud
                    let mut positions = Vec::with_capacity(payload.publish_count as usize);
                    let mut colors = Vec::with_capacity(payload.publish_count as usize);
                    
                    for i in 0..payload.publish_count as usize {
                        // Convert Distance to meters (f32)
                        let x= payload.points[i].x;
                        let y = payload.points[i].y;
                        let z = payload.points[i].z;

                        
                        positions.push([x, y, z]);

                        colors.push([0,255,0]);
                    }
                    // // Log the point cloud to Rerun
                    // if let Err(e) = recorder_data.recorder.log(
                    //     "lidar/pointcloud",
                    //     &Points3D::new(positions)
                    //         .with_colors(colors)
                    //         .with_radii([0.01f32])
                    // ) {
                    //     warning!("Failed to log point cloud to Rerun: {}", format!("{:?}", e));
                    // }
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
