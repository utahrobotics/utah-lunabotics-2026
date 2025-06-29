use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use cu29::cutask::CuMsg;

use crate::common::point_types::ImuMsg;
use crate::ROOT_NODE;
use simple_motion::StaticNode;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3, Quaternion};

pub struct ImuIceoryxReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, ImuMsg, ()>>,
    subscriber: Option<Subscriber<ipc::Service, ImuMsg, ()>>,
    lidar_node: StaticNode,
}

impl Freezable for ImuIceoryxReceiver {}

impl<'cl> CuSrcTask<'cl> for ImuIceoryxReceiver {
    type Output = output_msg!('cl, ImuMsg);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let service_str = config
            .and_then(|c| c.get::<String>("service"))
            .unwrap_or_else(|| "unilidar/imu".to_string());

        let service_name = ServiceName::new(&service_str).map_err(|e| {
            CuError::new_with_cause("ImuIceoryxReceiver: invalid service name", e)
        })?;

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
        set_log_level(LogLevel::Trace);
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<ImuMsg>()
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

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("ImuIceoryxReceiver: subscriber missing"))?;

        if let Some(sample) = subscriber.receive().map_err(|e| {
            CuError::new_with_cause("ImuIceoryxReceiver: receive", e)
        })? {
            let imu_raw: &ImuMsg = &*sample;
            // coordinate system swap?
            // let flip_matrix = Matrix3::new(
            //     0.0, -1.0, 0.0, // X_std = -Y_raw
            //     0.0, 0.0, 1.0,  // Y_std =  Z_raw
            //    -1.0, 0.0, 0.0,  // Z_std = -X_raw
            // );
            // let rot_flip = Rotation3::from_matrix_unchecked(flip_matrix);

            // let rot_base_sensor: UnitQuaternion<f64> = self
            //     .lidar_node
            //     .get_isometry_from_base()
            //     .rotation;

            // let rot_total = rot_base_sensor * UnitQuaternion::from_rotation_matrix(&rot_flip);

            // let q_raw = UnitQuaternion::new_normalize(Quaternion::new(
            //     imu_raw.quaternion[0] as f64,
            //     imu_raw.quaternion[1] as f64,
            //     imu_raw.quaternion[2] as f64,
            //     imu_raw.quaternion[3] as f64,
            // ));
            // let q_robot = rot_total * q_raw;

            // let ang_raw = Vector3::new(
            //     imu_raw.angular_velocity[0] as f64,
            //     imu_raw.angular_velocity[1] as f64,
            //     imu_raw.angular_velocity[2] as f64,
            // );
            // let lin_raw = Vector3::new(
            //     imu_raw.linear_acceleration[0] as f64,
            //     imu_raw.linear_acceleration[1] as f64,
            //     imu_raw.linear_acceleration[2] as f64,
            // );

            // let ang_robot = rot_total.transform_vector(&ang_raw);
            // let lin_robot = rot_total.transform_vector(&lin_raw);

            // let mut imu_out = *imu_raw;

            // imu_out.quaternion = [
            //     q_robot.w as f32,
            //     q_robot.i as f32,
            //     q_robot.j as f32,
            //     q_robot.k as f32,
            // ];
            // imu_out.angular_velocity = [
            //     ang_robot.x as f32,
            //     ang_robot.y as f32,
            //     ang_robot.z as f32,
            // ];
            // imu_out.linear_acceleration = [
            //     lin_robot.x as f32,
            //     lin_robot.y as f32,
            //     lin_robot.z as f32,
            // ];

            // new_msg.set_payload(*imu_raw);
        } else {
            new_msg.clear_payload();
        }

        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.service = None;
        self.subscriber = None;
        Ok(())
    }
} 