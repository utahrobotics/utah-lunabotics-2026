use bincode::{Decode, Encode};
use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use cu_sensor_payloads::{PointCloud, PointCloudSoa};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use cu29::cutask::CuMsg;
use crate::ROOT_NODE;
use simple_motion::StaticNode;
use iceoryx_types::{IceoryxPointCloud, PointXYZIR, MAX_POINT_CLOUD_POINTS};
use nalgebra::Point3;
// timestamps are normalized to [0,1] range for KISS-ICP deskewing

#[derive(Debug, Clone)]
pub struct PointCloudPayload {
    pub points: PointCloudSoa<MAX_POINT_CLOUD_POINTS>,
    pub timestamps: [f32; MAX_POINT_CLOUD_POINTS],
}

impl Default for PointCloudPayload {
    fn default() -> Self {
        Self {
            points: PointCloudSoa::default(),
            timestamps: [0.0; MAX_POINT_CLOUD_POINTS],
        }
    }
}

impl Encode for PointCloudPayload {
    fn encode<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), bincode::error::EncodeError> {
        self.points.encode(encoder)?;
        self.timestamps.encode(encoder)?;
        Ok(())
    }
}

impl Decode<()> for PointCloudPayload {
    fn decode<D: bincode::de::Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, bincode::error::DecodeError> {
        let points = PointCloudSoa::decode(decoder)?;
        let timestamps = <[f32; MAX_POINT_CLOUD_POINTS]>::decode(decoder)?;
        Ok(Self { points, timestamps })
    }
}
pub struct PointCloudIceoryxReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, IceoryxPointCloud, ()>>,
    subscriber: Option<Subscriber<ipc::Service, IceoryxPointCloud, ()>>,
    l2_node: StaticNode,
    last_seen: Instant
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
            l2_node: ROOT_NODE.get().unwrap().get_node_with_name("l2_front").unwrap().clone(),
            last_seen: Instant::now()
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

        // Allocate on the heap to keep the stack small in debug builds
        let mut payload = Box::new(PointCloudPayload::default());

        let iso = self.l2_node.get_global_isometry();
        while let Some(sample) = subscriber.receive().map_err(|e| {
            CuError::new_with_cause("PointCloudIceoryxReceiver: receive", e)
        })? {
            let cloud: &IceoryxPointCloud = &*sample;
            info!("Received {} points in cloud", cloud.publish_count);
            let mut pub_count = cloud.publish_count.min(MAX_POINT_CLOUD_POINTS as u64);
            self.last_seen = Instant::now();
            for idx in 0..(pub_count as usize) {
                let p: PointXYZIR = cloud.points[idx as usize];
                let point = Point3::new(p.x as f64, p.y as f64, p.z as f64);
                let transformed_point = iso.transform_point(&point);
                // if p.intensity < 200. {
                //     pub_count -= 1;
                //     continue;
                // }
                // Convert L2 coordinate system to lidar coordinate system
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
            new_msg.set_payload(*payload); // move value out of the Box
        } else {
            new_msg.clear_payload();
        }

        if self.last_seen.elapsed().as_millis() > 600 {
            return Err(
                CuError::new_with_cause(
                    "No points seen in 600 ms", 
                    std::io::Error::other("No points seen in 600 ms")
                )
            )
        } else {
            return Ok(());
        }
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.service = None;
        self.subscriber = None;
        Ok(())
    }
}
