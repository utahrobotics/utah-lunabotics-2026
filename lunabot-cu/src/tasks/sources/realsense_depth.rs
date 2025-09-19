use cu29::{
    cutask::{CuSrcTask, Freezable},
    prelude::*,
};
use iceoryx_types::{IceoryxDepthFrame, IceoryxPointCloud};
use iceoryx2::{
    node::NodeBuilder,
    port::subscriber::Subscriber,
    prelude::{LogLevel, ServiceName, set_log_level},
    service::ipc,
};

pub static DEPTH_FRAME_SIZE: usize = 640 * 480;
pub static DEPTH_FRAME_WIDTH: u32 = 640;
pub static DEPTH_FRAME_HEIGHT: u32 = 480;

pub struct RealsenseDepth {
    last_seen: u64,
    subscriber: Subscriber<ipc::Service, IceoryxDepthFrame<DEPTH_FRAME_SIZE>, ()>,
}

impl Freezable for RealsenseDepth {}

impl CuSrcTask for RealsenseDepth {
    type Output<'m> = output_msg!(IceoryxDepthFrame<DEPTH_FRAME_SIZE>);

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let service_str = config
            .and_then(|c| c.get::<String>("service"))
            .unwrap_or_else(|| "realsense/depth".to_string());
        let service_name = ServiceName::new(&service_str)
            .map_err(|e| CuError::new_with_cause("invalid service name", e))?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("node create faliure", e))?;

        set_log_level(LogLevel::Trace);
        let service = node
            .service_builder(&service_name)
            .publish_subscribe::<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>()
            .enable_safe_overflow(true)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("service open error", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("subscriber creation error", e))?;
        Ok(Self {
            last_seen: 0,
            subscriber,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        new_msg.clear_payload();

        while let Some(sample) = self
            .subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("subscriber receive failed", e))?
        {
            new_msg.set_payload(sample.payload().clone());
            self.last_seen = clock.now().into();
        }

        if clock.now().as_nanos() - self.last_seen > 500_000_000 {
            return Err(CuError::new_with_cause(
                "no depth frames seen in 500 ms",
                std::io::Error::other("no depth frames seen in 500 ms"),
            ));
        }

        return Ok(());
    }
}
