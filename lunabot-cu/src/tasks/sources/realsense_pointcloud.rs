use crate::ROOT_NODE;
use crate::rerun_viz::RECORDER;
use cu29::cutask::CuMsg;
use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSrcTask, Freezable},
    output_msg,
    prelude::*,
};
use iceoryx_types::{IceoryxPointCloud, PointXYZIR};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use simple_motion::StaticNode;

pub struct RealSensePointCloudReceiver {
    service_name: ServiceName,
    node: iceoryx2::node::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, IceoryxPointCloud, ()>>,
    subscriber: Option<Subscriber<ipc::Service, IceoryxPointCloud, ()>>,
    camera_node: StaticNode,
    last_seen: u64,
}

impl Freezable for RealSensePointCloudReceiver {}

impl CuSrcTask for RealSensePointCloudReceiver {
    type Output<'m> = output_msg!(IceoryxPointCloud);

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

        let camera_node = ROOT_NODE
            .get()
            .ok_or_else(|| CuError::from("RealSensePointCloudReceiver: ROOT_NODE not initialized"))?
            .get_node_with_name(&camera_name)
            .ok_or_else(|| {
                CuError::from(format!(
                    "RealSensePointCloudReceiver: camera node '{}' not found",
                    camera_name
                ))
            })?
            .clone();

        Ok(Self {
            service_name,
            node,
            service: None,
            subscriber: None,
            camera_node,
            last_seen: 0,
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

    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.clear_payload();

        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("PointCloudIceoryxReceiver: subscriber missing"))?;

        while let Some(sample) = subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("PointCloudIceoryxReceiver: receive", e))?
        {
            let mut payload = sample.payload().clone();
            let mut positions = Vec::new();
            let mut colors = Vec::new();

            for point in payload.points.iter_mut() {
                let transformed = point.to_nalgebra();
                positions.push([
                    transformed.x as f32,
                    transformed.y as f32,
                    transformed.z as f32,
                ]);
                colors.push([0, 100, 100]);
                *point =
                    PointXYZIR::from_nalgebra(transformed, point.intensity, point.time, point.ring);
            }
            if let Err(e) = RECORDER.get().unwrap().recorder.log(
                "realsense",
                &rerun::Points3D::new(positions)
                    .with_colors(colors)
                    .with_radii([0.02f32]),
            ) {
                warning!("Failed to log accumulated map to Rerun: {}", e.to_string());
            }
            new_msg.set_payload(payload);
            self.last_seen = clock.now().as_nanos();
        }

        if clock.now().as_nanos() - self.last_seen > 600_000 {
            return Err(CuError::new_with_cause(
                "No points seen in 600 ms",
                std::io::Error::other("No points seen in 600 ms"),
            ));
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
