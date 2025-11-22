use cu29::prelude::*;
use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
use mujoco_rs::prelude::*;

use crate::tasks::{DEPTH_FRAME_SIZE, DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT};

pub struct SimSensors {
    accel_sensor_idx: usize,
    gyro_sensor_idx: usize,
    quat_sensor_idx: usize,
}

impl SimSensors {
    pub fn new(model: &MjModel) -> CuResult<Self> {
        let accel_sensor_idx = model
            .sensor("base_accel")
            .ok_or_else(|| CuError::from("base_accel sensor not found"))?
            .id;
        let gyro_sensor_idx = model
            .sensor("base_gyro")
            .ok_or_else(|| CuError::from("base_gyro sensor not found"))?
            .id;
        let quat_sensor_idx = model
            .sensor("base_quat")
            .ok_or_else(|| CuError::from("base_quat sensor not found"))?
            .id;

        Ok(Self {
            accel_sensor_idx,
            gyro_sensor_idx,
            quat_sensor_idx,
        })
    }
    
    pub fn read_depth_frame<'m>(
        &self, 
        _model: &'m MjModel, 
        _data: &MjData<&'m MjModel>
    ) -> IceoryxDepthFrame<DEPTH_FRAME_SIZE> {
        // TODO: implement depth rendering
        IceoryxDepthFrame::default()
    }
    
    pub fn read_imu_data<'m>(
        &self,
        model: &'m MjModel,
        data: &MjData<&'m MjModel>
    ) -> ImuMsg {
        let sensor_adr = model.sensor_adr();
        let sensordata = data.sensordata();

        let accel_adr = sensor_adr[self.accel_sensor_idx] as usize;
        let gyro_adr = sensor_adr[self.gyro_sensor_idx] as usize;
        let quat_adr = sensor_adr[self.quat_sensor_idx] as usize;

        let linear_acceleration = [
            sensordata[accel_adr] as f32,
            sensordata[accel_adr + 1] as f32,
            sensordata[accel_adr + 2] as f32,
        ];

        let angular_velocity = [
            sensordata[gyro_adr] as f32,
            sensordata[gyro_adr + 1] as f32,
            sensordata[gyro_adr + 2] as f32,
        ];

        let quaternion = [
            sensordata[quat_adr] as f32,
            sensordata[quat_adr + 1] as f32,
            sensordata[quat_adr + 2] as f32,
            sensordata[quat_adr + 3] as f32,
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