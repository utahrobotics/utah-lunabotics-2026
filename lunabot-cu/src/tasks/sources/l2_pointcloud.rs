use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use cu_sensor_payloads::{PointCloud, PointCloudSoa};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use cu29::cutask::CuMsg;
use crate::ROOT_NODE;
use simple_motion::StaticNode;
use crate::common::point_types::{IceoryxPointCloud, PointXYZIR, MAX_POINT_CLOUD_POINTS};
use nalgebra::Point3;

pub type PointCloudPayload = PointCloudSoa<MAX_POINT_CLOUD_POINTS>;

pub struct PointCloudIceoryxReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, IceoryxPointCloud, ()>>,
    subscriber: Option<Subscriber<ipc::Service, IceoryxPointCloud, ()>>,
}

impl Freezable for PointCloudIceoryxReceiver {}

impl<'cl> CuSrcTask<'cl> for PointCloudIceoryxReceiver {
    type Output = output_msg!('cl, PointCloudPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let service_str = config
            .and_then(|c| c.get::<String>("service"))
            .unwrap_or_else(|| "unilidar/cloud_full".to_string());

        let service_name = ServiceName::new(&service_str).map_err(|e| {
            CuError::new_with_cause("PointCloudIceoryxReceiver: invalid service name", e)
        })?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("PointCloudIceoryxReceiver: node create", e))?;


        Ok(Self {
            service_name,
            node,
            service: None,
            subscriber: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        set_log_level(LogLevel::Trace);
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<IceoryxPointCloud>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("PointCloudIceoryxReceiver: service", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("PointCloudIceoryxReceiver: subscriber", e))?;

        self.service = Some(service);
        self.subscriber = Some(subscriber);
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("PointCloudIceoryxReceiver: subscriber missing"))?;

        let mut payload = PointCloudPayload::default();
        while let Some(sample) = subscriber.receive().map_err(|e| {
            CuError::new_with_cause("PointCloudIceoryxReceiver: receive", e)
        })? {
            let cloud: &IceoryxPointCloud = &*sample;
            info!("Received {} points in cloud", cloud.publish_count);

            for idx in 0..cloud.publish_count.min(MAX_POINT_CLOUD_POINTS as u64) {
                let p: PointXYZIR = cloud.points[idx as usize];
                // First convert the L2 coordinate system to the lidar coordinate system
                // used in the rest of the robot code, then transform that point into the
                // robot base frame using the kinematic chain.

                let local_point = Point3::new(-(p.y as f64), p.z as f64, -(p.x as f64));
                payload.push(PointCloud::new(
                    clock.now(),
                    local_point.x as f32,
                    local_point.y as f32,
                    local_point.z as f32,
                    p.intensity,
                    Some(p.ring as u8),
                ));
            }
        }

        if !payload.is_empty() {
            new_msg.set_payload(payload);
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
