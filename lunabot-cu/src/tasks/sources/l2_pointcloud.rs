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

use serde::ser::{SerializeStruct, Serializer};

pub struct PointCloudIceoryxReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, IceoryxPointCloud, ()>>,
    subscriber: Option<Subscriber<ipc::Service, IceoryxPointCloud, ()>>,
    l2_node: StaticNode,
    last_seen: u64
}

impl Freezable for PointCloudIceoryxReceiver {}

impl<'cl> CuSrcTask<'cl> for PointCloudIceoryxReceiver {
    type Output = output_msg!('cl, IceoryxPointCloud);

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
            last_seen: 0
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
        new_msg.clear_payload();
        
        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("PointCloudIceoryxReceiver: subscriber missing"))?;

        // Allocate on the heap to keep the stack small in debug builds

        let iso = self.l2_node.get_isometry_from_base();
        while let Some(sample) = subscriber.receive().map_err(|e| {
            CuError::new_with_cause("PointCloudIceoryxReceiver: receive", e)
        })? {
            let payload = sample.payload().clone();
            new_msg.set_payload(payload);
            self.last_seen = clock.now().as_nanos();
        }

        if clock.now().as_nanos() - self.last_seen > 600_000 {
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
