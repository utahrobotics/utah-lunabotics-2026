use cu29::{
    cutask::{CuSrcTask, Freezable},
    prelude::*,
};
use iceoryx_types::{IceoryxDepthFrame, ImuMsg};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::{
    node::NodeBuilder,
    port::subscriber::Subscriber,
    prelude::{LogLevel, ServiceName, set_log_level},
    service::ipc,
};

pub static DEPTH_FRAME_SIZE: usize = 640 * 480;
pub static DEPTH_FRAME_WIDTH: u32 = 640;
pub static DEPTH_FRAME_HEIGHT: u32 = 480;

pub struct RealsenseSubscriber {
    last_seen: u64,
    /// subscribes to depth frames published by the realsense external task
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    depth_subscriber: Subscriber<ipc::Service, IceoryxDepthFrame<DEPTH_FRAME_SIZE>, ()>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    imu_subscriber: Subscriber<ipc::Service, ImuMsg, ()>,
}

impl Freezable for RealsenseSubscriber {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]

impl CuSrcTask for RealsenseSubscriber {
    type Output<'m> = output_msg!((Option<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>, Option<ImuMsg>));

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let serial_num = config
            .and_then(|c| c.get::<String>("serial_num"))
            .unwrap_or_else(|| "realsense/depth".to_string());
        let depth_service_str = format!("realsense/{serial_num}/depth");
        let imu_service_str = format!("realsense/{serial_num}/imu");

        let depth_service_name = ServiceName::new(&depth_service_str)
            .map_err(|e| CuError::new_with_cause("invalid service name", e))?;
        let imu_service_name = ServiceName::new(&imu_service_str)
            .map_err(|e| CuError::new_with_cause("invalid service name", e))?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("node create faliure", e))?;

        set_log_level(LogLevel::Debug);
        let depth_service = node
            .service_builder(&depth_service_name)
            .publish_subscribe::<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>()
            .enable_safe_overflow(true)
            .subscriber_max_buffer_size(20)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("service open error", e))?;
        let imu_service = node
            .service_builder(&imu_service_name)
            .publish_subscribe::<ImuMsg>()
            .enable_safe_overflow(true)
            .subscriber_max_buffer_size(20)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("service open error", e))?;

        let depth_subscriber = depth_service
            .subscriber_builder()
            .buffer_size(19)
            .create()
            .map_err(|e| CuError::new_with_cause("subscriber creation error", e))?;
        let imu_subscriber = imu_service
            .subscriber_builder()
            .buffer_size(19)
            .create()
            .map_err(|e| CuError::new_with_cause("subscriber creation error", e))?;
        Ok(Self {
            last_seen: 0,
            depth_subscriber,
            imu_subscriber,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        new_msg.clear_payload();

        let mut output = (None, None);

        // gather any pending samples from the realsense publishers
        while let Some(sample) = self
            .depth_subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("subscriber receive failed", e))?
        {
            // this clone feels bad
            output.0 = Some(sample.payload().clone());
            self.last_seen = clock.now().into();
        }
        while let Some(sample) = self
            .imu_subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("subscriber receive failed", e))?
        {
            output.1 = Some(sample.payload().clone());
            self.last_seen = clock.now().into();
        }

        if output.0.is_some() || output.1.is_some() {
            new_msg.set_payload(output);
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

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for RealsenseSubscriber {
    type Output<'m> = output_msg!((Option<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>, Option<ImuMsg>));

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { last_seen: 0 })
    }

    fn process<'o>(
        &mut self,
        _clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        new_msg.clear_payload();
        Ok(())
    }
}
