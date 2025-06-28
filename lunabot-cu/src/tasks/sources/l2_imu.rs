use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use cu29::cutask::CuMsg;

use crate::common::point_types::ImuMsg;

pub struct ImuIceoryxReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, ImuMsg, ()>>,
    subscriber: Option<Subscriber<ipc::Service, ImuMsg, ()>>,
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
            let mut imu_msg: &ImuMsg = &*sample;
            info!("Received IMU message");
            new_msg.set_payload(*imu_msg);
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