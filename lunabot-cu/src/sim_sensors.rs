use cu29::prelude::*;
use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use mujoco_rs::prelude::*;

use crate::tasks::{DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT};

pub struct SimSensors {
    accel_sensor_id: i32,
    gyro_sensor_id: i32,
    quat_sensor_id: i32,
}

impl SimSensors {
    pub fn new(model: &MjModel) -> CuResult<Self> {
        let accel_sensor_id = model.sensor("base_accel")
            .ok_or_else(|| CuError::new_with_cause("base_accel sensor not found", ""))?;
        let gyro_sensor_id = model.sensor("base_gyro")
            .ok_or_else(|| CuError::new_with_cause("base_gyro sensor not found", ""))?;
        let quat_sensor_id = model.sensor("base_quat")
            .ok_or_else(|| CuError::new_with_cause("base_quat sensor not found", ""))?;
        
        Ok(Self {
            accel_sensor_id,
            gyro_sensor_id,
            quat_sensor_id,
        })
    }
    
    pub fn read_depth_frame(
        &self, 
        _model: &MjModel, 
        _data: &MjData
    ) -> IceoryxDepthFrame<DEPTH_FRAME_SIZE> {
        // TODO: implement depth rendering
        IceoryxDepthFrame::default()
    }
    
    pub fn read_imu_data(
        &self,
        model: &MjModel,
        data: &MjData
    ) -> ImuMsg {
        let accel_adr = model.sensor_adr(self.accel_sensor_id);
        let gyro_adr = model.sensor_adr(self.gyro_sensor_id);
        let quat_adr = model.sensor_adr(self.quat_sensor_id);
        
        let linear_acceleration = [
            data.sensordata(accel_adr),
            data.sensordata(accel_adr + 1),
            data.sensordata(accel_adr + 2),
        ];
        
        let angular_velocity = [
            data.sensordata(gyro_adr),
            data.sensordata(gyro_adr + 1),
            data.sensordata(gyro_adr + 2),
        ];
        
        let quaternion = [
            data.sensordata(quat_adr),
            data.sensordata(quat_adr + 1),
            data.sensordata(quat_adr + 2),
            data.sensordata(quat_adr + 3),
        ];
        
        ImuMsg {
            quaternion,
            angular_velocity,
            linear_acceleration,
        }
    }
}


    // 1. Set up offscreen rendering context
    // 2. Position camera at depth_camera site
    // 3. Render scene to depth buffer
    // 4. Read depth pixels
    // 5. Convert to IceoryxDepthFrame format
    //    - Handle depth scaling
    //    - Apply noise model (optional)
    //    - Set intrinsics (fx, fy, ppx, ppy)

// TODO:
/*
Convert depth buffer to IceoryxDepthFrame
Convert mujoco units to SI units for IMU data
Read accelerometer, gyroscope, and quaternion data and format in ImuMsg

For lidar: 
simulate it using the outputs from kiss icp 
just pipe the data in and add noise model if necessary
*/