use crate::ROOT_NODE;
use crate::rerun_viz::{Level, RECORDER};
use cu29::cutask::CuMsg;
use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSrcTask, Freezable},
    output_msg,
    prelude::*,
};
use iceoryx_types::{IceoryxPointCloud, PointCloudAccumulator, PointXYZIR};
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
    accumulator: PointCloudAccumulator,
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
            accumulator: PointCloudAccumulator::new(),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        set_log_level(LogLevel::Fatal);
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<IceoryxPointCloud>()
            .subscriber_max_buffer_size(20)
            .enable_safe_overflow(false)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("RealSensePointCloudReceiver: service", e))?;

        let subscriber = service
            .subscriber_builder()
            .buffer_size(19)
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
            let payload = sample.payload();

            match self.accumulator.add_message(payload) {
                Ok(Some(complete_points)) => {
                    let mut positions = Vec::new();
                    let mut colors = Vec::new();

                    for point in complete_points.iter() {
                        if point.x != 0.0 || point.y != 0.0 || point.z != 0.0 {
                            positions.push([point.x as f32, point.y as f32, point.z as f32]);
                            colors.push([0, 100, 100]);
                        }
                    }

                    if !positions.is_empty() && RECORDER.get().unwrap().level == Level::All {
                        if let Err(e) = RECORDER.get().unwrap().recorder.log(
                            "realsense",
                            &rerun::Points3D::new(positions)
                                .with_colors(colors)
                                .with_radii([0.02f32]),
                        ) {
                            warning!(
                                "Failed to log complete point cloud to Rerun: {}",
                                e.to_string()
                            );
                        }
                    }

                    let _ = self.accumulator.take_completed();
                }
                Ok(None) => {}
                Err(e) => {
                    eprintln!("Point cloud accumulator error: {}", e);
                    self.accumulator.reset();

                    let mut positions = Vec::new();
                    let mut colors = Vec::new();

                    for point in payload.points[..payload.publish_count as usize].iter() {
                        positions.push([point.x as f32, point.y as f32, point.z as f32]);
                        colors.push([0, 100, 100]);
                    }

                    if RECORDER.get().unwrap().level == Level::All {
                        if let Err(e) = RECORDER.get().unwrap().recorder.log(
                            "realsense",
                            &rerun::Points3D::new(positions)
                                .with_colors(colors)
                                .with_radii([0.02f32]),
                        ) {
                            warning!("Failed to log fallback points to Rerun: {}", e.to_string());
                        }
                    }
                }
            }

            new_msg.set_payload(payload.clone());
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
        self.accumulator.reset();
        Ok(())
    }
}
