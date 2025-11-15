use bincode::{Decode, Encode};
use cu29::cutask::CuMsg;
use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSrcTask, Freezable},
    output_msg,
};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use kalman_filter::SimpleVector;
use serde::Serialize;
use crate::rerun_viz::RECORDER;

use crate::ROOT_NODE;
use iceoryx_types::ImuMsg;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use simple_motion::StaticNode;

pub struct ImuIceoryxReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, ImuMsg, ()>>,
    subscriber: Option<Subscriber<ipc::Service, ImuMsg, ()>>,
    lidar_node: StaticNode,
    imu_variance: [f64; 81],
    warmup_samples: Vec<Vector3<f64>>,
    warmup_complete: bool,
    gravity_magnitude: f64,
}

#[derive(Clone, Copy, Debug, Encode, Decode, Serialize, ZeroCopySend)]
#[repr(C)]
pub struct ImuMeasurement {
    pub acceleration: [f64; 3],
    pub orientation: [f64; 3],
    pub angular_velocity: [f64; 3],
    #[serde(serialize_with = "<[_]>::serialize")]
    pub variance: [f64; 81],
}

impl Default for ImuMeasurement {
    fn default() -> Self {
        Self {
            acceleration: Default::default(),
            orientation: Default::default(),
            angular_velocity: Default::default(),
            variance: [0.0; 81],
        }
    }
}

impl Freezable for ImuIceoryxReceiver {}

impl CuSrcTask for ImuIceoryxReceiver {
    type Output<'m> = output_msg!(ImuMeasurement);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let service_str = config
            .and_then(|c| c.get::<String>("service"))
            .unwrap_or_else(|| "unilidar/imu".to_string());

        let service_name = ServiceName::new(&service_str)
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: invalid service name", e))?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: node create", e))?;

        let diagonal = SimpleVector::<9>::from_column_slice(&[
            0.005 as f64, // Acceleration variance
            0.005 as f64,
            0.005 as f64,
            0.05 as f64, // Orientation variance
            0.05 as f64,
            0.05 as f64,
            0.1 as f64, // Angular velocity variance
            0.1 as f64,
            0.1 as f64,
        ]);
        let variance: [f64; 81] = kalman_filter::SimpleSquareMatrix::<9>::from_diagonal(&diagonal)
            .as_slice()
            .try_into()
            .expect("Variance matrix in [l2_imu.rs] was not 9x9");

        Ok(Self {
            service_name,
            node,
            service: None,
            subscriber: None,
            lidar_node: ROOT_NODE
                .get()
                .unwrap()
                .clone()
                .get_node_with_name("l2_front")
                .unwrap(),
            imu_variance: variance,
            warmup_samples: Vec::with_capacity(200), // Collect 200 samples for warmup
            warmup_complete: false,
            gravity_magnitude: 9.8, // default, replaced after warmup sequence
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        set_log_level(LogLevel::Fatal);
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<ImuMsg>()
            .enable_safe_overflow(true)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: service", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: subscriber", e))?;

        self.service = Some(service);
        self.subscriber = Some(subscriber);

        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.clear_payload();

        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("ImuIceoryxReceiver: subscriber missing"))?;

        while let Some(sample) = subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: receive", e))?
        {
            let imu_raw: &ImuMsg = &*sample;

            let base_to_l2: UnitQuaternion<f64> = self.lidar_node.get_isometry_from_base().rotation;

            let imu_linear_acceleration = Vector3::new(
                imu_raw.linear_acceleration[0] as f64,
                imu_raw.linear_acceleration[1] as f64,
                imu_raw.linear_acceleration[2] as f64,
            );
            let imu_angular_velocity = Vector3::new(
                imu_raw.angular_velocity[0] as f64,
                imu_raw.angular_velocity[1] as f64,
                imu_raw.angular_velocity[2] as f64,
            );
            let imu_quaternion_sensor = UnitQuaternion::new_normalize(Quaternion::new(
                imu_raw.quaternion[0] as f64,
                imu_raw.quaternion[1] as f64,
                imu_raw.quaternion[2] as f64,
                imu_raw.quaternion[3] as f64,
            ));
            
            if let Some(logger) = RECORDER.get()
            {
                let _ = logger.recorder.log(
                    "imu_raw",
                    &rerun::Arrows3D::from_vectors([rerun::Vec3D::new(imu_linear_acceleration.x as f32, imu_linear_acceleration.y as f32, imu_linear_acceleration.z as f32)]),
                );
                //self.root_node.set_isometry(pose_msg);
            }
            let imu_quaternion_base = base_to_l2.inverse() * imu_quaternion_sensor;

            // TODO: figure out if these transformations are right
            let acc = imu_quaternion_base * (base_to_l2.inverse() * imu_linear_acceleration);
            let gyr = base_to_l2.inverse() * imu_angular_velocity;

            let orientation_state = imu_quaternion_base
                .axis()
                .and_then(|vec| Some(vec.into_inner()))
                .unwrap_or(SimpleVector::<3>::zeros())
                * imu_quaternion_base.angle();

            let imu_orientation = Vector3::new(
                orientation_state.x as f64,
                orientation_state.y as f64,
                orientation_state.z as f64,
            );

            // figure out the magnitude of gravity
            // sometimes the L2 is has egregious scaling errors.
            // this assumes the robot is relatively stable for the first bit.
            if !self.warmup_complete {
                self.warmup_samples.push(acc);

                if self.warmup_samples.len() >= 200 {
                    let sum: Vector3<f64> = self.warmup_samples.iter().sum();
                    let avg = sum / (self.warmup_samples.len() as f64);
                    self.gravity_magnitude = avg.norm();

                    println!(
                        "Measured gravity magnitude: {:.3} m/s²",
                        self.gravity_magnitude
                    );
                    println!("Gravity vector: [{:.3}, {:.3}, {:.3}]", avg.x, avg.y, avg.z);

                    self.warmup_complete = true;
                    self.warmup_samples.clear();
                }

                // Don't publish messages during warmup
                continue;
            }

            // After warmup, subtract the measured gravity magnitude in the z direction
            let actual_message = ImuMeasurement {
                acceleration: [acc.x, acc.y, acc.z - self.gravity_magnitude],
                angular_velocity: [gyr.x, gyr.y, gyr.z],
                orientation: [imu_orientation.x, imu_orientation.y, imu_orientation.z],
                variance: self.imu_variance,
            };
            new_msg.set_payload(actual_message);
        }
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.service = None;
        self.subscriber = None;
        Ok(())
    }
}
