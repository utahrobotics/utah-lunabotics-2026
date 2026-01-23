use crate::ROBOT_STATE;
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

use iceoryx_types::{IceoryxPointCloud, PointXYZIR};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::node::NodeBuilder;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::port::subscriber::Subscriber;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::prelude::*;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

use simple_motion::StaticNode;

pub struct PointCloudIceoryxReceiver {
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    service_name: ServiceName,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    node: iceoryx2::node::Node<ipc::Service>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    service: Option<PortFactory<ipc::Service, IceoryxPointCloud, ()>>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    subscriber: Option<Subscriber<ipc::Service, IceoryxPointCloud, ()>>,
    l2_node: StaticNode,
    last_seen: u64,
}

impl Freezable for PointCloudIceoryxReceiver {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuSrcTask for PointCloudIceoryxReceiver {
    type Output<'m> = output_msg!(IceoryxPointCloud);
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let service_str = config
            .and_then(|c| c.get::<String>("service").expect("failed to deserialize"))
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
            l2_node: ROBOT_STATE
                .get()
                .unwrap()
                .kinematic_root
                .get_node_with_name("l2_front")
                .unwrap()
                .clone(),
            last_seen: 0,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        set_log_level(LogLevel::Trace);
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<IceoryxPointCloud>()
            .enable_safe_overflow(true)
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

    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.clear_payload();

        let subscriber = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("PointCloudIceoryxReceiver: subscriber missing"))?;

        // Allocate on the heap to keep the stack small in debug builds

        let base_iso = self.l2_node.get_isometry_from_base();
        while let Some(sample) = subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("PointCloudIceoryxReceiver: receive", e))?
        {
            let mut payload = sample.payload().clone();
            let mut positions = Vec::new();
            let mut colors = Vec::new();
            for point in payload.points.iter_mut() {
                let transformed = base_iso.transform_point(&point.to_nalgebra());
                positions.push([
                    transformed.x as f32,
                    transformed.y as f32,
                    transformed.z as f32,
                ]);
                colors.push([0, 255, 0]);
                *point =
                    PointXYZIR::from_nalgebra(transformed, point.intensity, point.time, point.ring);
            }
            if RECORDER.get().is_some() && RECORDER.get().unwrap().level == Level::All {
                if let Err(e) = RECORDER.get().unwrap().recorder.log(
                    format!("kiss_icp/local/cloud"),
                    &rerun::Points3D::new(positions)
                        .with_colors(colors)
                        .with_radii([0.02f32]),
                ) {
                    warning!("Failed to log accumulated map to Rerun: {}", e.to_string());
                }
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

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for PointCloudIceoryxReceiver {
    type Output<'m> = output_msg!(IceoryxPointCloud);
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>,_resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            l2_node: ROBOT_STATE
                .get()
                .unwrap()
                .kinematic_root
                .get_node_with_name("l2_front")
                .unwrap()
                .clone(),
            last_seen: 0,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, _new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
