use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use cu_sensor_payloads::PointCloud;
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use cu29::cutask::CuMsg;
use crate::ROOT_NODE;
use simple_motion::StaticNode;
use iceoryx_types::{IceoryxPointCloud, PointXYZIR, MAX_POINT_CLOUD_POINTS};
use crate::tasks::PointCloudPayload;
use nalgebra::Point3;

pub struct RealSensePointCloudReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, IceoryxPointCloud, ()>>,
    subscriber: Option<Subscriber<ipc::Service, IceoryxPointCloud, ()>>,
    camera_node: StaticNode,
}

impl Freezable for RealSensePointCloudReceiver {}

impl<'cl> CuSrcTask<'cl> for RealSensePointCloudReceiver {
    type Output = output_msg!('cl, PointCloudPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let service_str = config
            .and_then(|c| c.get::<String>("service"))
            .unwrap_or_else(|| "realsense/309622300683/cloud".to_string());

        let camera_name = config
            .and_then(|c| c.get::<String>("camera_node"))
            .unwrap_or_else(|| "upper_depth_camera".to_string());

        let service_name = ServiceName::new(&service_str).map_err(|e| {
            CuError::new_with_cause("RealSensePointCloudReceiver: invalid service name", e)
        })?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("RealSensePointCloudReceiver: node create", e))?;

        let camera_node = ROOT_NODE.get()
            .ok_or_else(|| CuError::from("RealSensePointCloudReceiver: ROOT_NODE not initialized"))?
            .get_node_with_name(&camera_name)
            .ok_or_else(|| CuError::from(format!("RealSensePointCloudReceiver: camera node '{}' not found", camera_name)))?
            .clone();

        Ok(Self {
            service_name,
            node,
            service: None,
            subscriber: None,
            camera_node,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        set_log_level(LogLevel::Fatal);
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<IceoryxPointCloud>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("RealSensePointCloudReceiver: service", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("RealSensePointCloudReceiver: subscriber", e))?;

        self.service = Some(service);
        self.subscriber = Some(subscriber);
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("RealSensePointCloudReceiver: subscriber missing"))?;

        let mut payload = Box::new(PointCloudPayload::default());
        let iso = self.camera_node.get_isometry_from_base();

        while let Some(sample) = subscriber.receive().map_err(|e| {
            CuError::new_with_cause("RealSensePointCloudReceiver: receive", e)
        })? {
            let cloud: &IceoryxPointCloud = &*sample;
            info!("Received {} points from RealSense", cloud.publish_count);

            for idx in 0..cloud.publish_count.min(MAX_POINT_CLOUD_POINTS as u64) {
                let p: PointXYZIR = cloud.points[idx as usize];
                let point = Point3::new(p.x as f64, p.y as f64, p.z as f64);
                let transformed_point = iso.transform_point(&point);

                // Convert camera coordinate system to global coordinate system
                payload.points.push(PointCloud::new(
                    clock.now(),
                    transformed_point.x as f32,
                    transformed_point.y as f32,
                    transformed_point.z as f32,
                    p.intensity,
                    Some(p.ring as u8),
                ));
                payload.timestamps[idx as usize] = p.time;
            }
        }

        if !payload.points.is_empty() {
            new_msg.set_payload(*payload);
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