use cu29::cutask::CuMsg;
use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSrcTask, Freezable},
    output_msg,
};

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::node::NodeBuilder;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::port::subscriber::Subscriber;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::prelude::*;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

use crate::ROOT_NODE;
use iceoryx_types::ImuMsg;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use simple_motion::StaticNode;

pub struct ImuIceoryxReceiver {
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    service_name: ServiceName,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    node: iceoryx2::node::Node<ipc::Service>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    service: Option<PortFactory<ipc::Service, ImuMsg, ()>>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    subscriber: Option<Subscriber<ipc::Service, ImuMsg, ()>>,
    lidar_node: StaticNode,
}

impl Freezable for ImuIceoryxReceiver {}
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuSrcTask for ImuIceoryxReceiver {
    type Output<'m> = output_msg!(ImuMsg);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let service_str = config
            .and_then(|c| c.get::<String>("service"))
            .unwrap_or_else(|| "unilidar/imu".to_string());

        let service_name = ServiceName::new(&service_str)
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: invalid service name", e))?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("ImuIceoryxReceiver: node create", e))?;

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
            let imu_quaternion = UnitQuaternion::new_normalize(Quaternion::new(
                imu_raw.quaternion[0] as f64,
                imu_raw.quaternion[1] as f64,
                imu_raw.quaternion[2] as f64,
                imu_raw.quaternion[3] as f64,
            ));

            let rot_base_sensor: UnitQuaternion<f64> =
                self.lidar_node.get_isometry_from_base().rotation;

            let quat = rot_base_sensor * imu_quaternion;
            let quat = quat.coords.as_slice();
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
            let acc = rot_base_sensor * imu_linear_acceleration;
            let gyr = rot_base_sensor * imu_angular_velocity;
            let actual_message = ImuMsg {
                linear_acceleration: [acc.x as f32, acc.y as f32, acc.z as f32],
                angular_velocity: [gyr.x as f32, gyr.y as f32, gyr.z as f32],
                quaternion: [
                    quat[0] as f32,
                    quat[1] as f32,
                    quat[2] as f32,
                    quat[3] as f32,
                ],
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

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for ImuIceoryxReceiver {
    type Output<'m> = output_msg!(ImuMsg);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self {
            lidar_node: ROOT_NODE
                .get()
                .unwrap()
                .clone()
                .get_node_with_name("l2_front")
                .unwrap(),
        })
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.clear_payload();
        Ok(())
    }
}
