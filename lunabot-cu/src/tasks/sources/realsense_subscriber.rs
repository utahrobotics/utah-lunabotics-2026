#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::sync::Arc;

use cu_sensor_payloads::CuImage;
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

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::payloads::depth_frame::CuDepthFrame;

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
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    pool: Arc<CuHostMemoryPool<Vec<u16>>>,
}

impl Freezable for RealsenseSubscriber {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]

impl CuSrcTask for RealsenseSubscriber {
    type Output<'m> = output_msg!(CuDepthFrame<Vec<u16>>);
    type Resources<'r> = ();

    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let serial_num = config
            .and_then(|c| {
                c.get::<String>("serial_num")
                    .expect("failed to deserialize")
            })
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
        let pool = CuHostMemoryPool::new("realsense_depth_frames", 4, || {
            vec![016; (DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH) as usize]
        })?;

        Ok(Self {
            last_seen: 0,
            depth_subscriber,
            imu_subscriber,
            pool,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        let t_start = clock.now().as_nanos();

        new_msg.clear_payload();
        let t_after_clear = clock.now().as_nanos();

        while let Some(sample) = self
            .depth_subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("subscriber receive failed", e))?
        {
            use crate::payloads::depth_frame::CuDepthFrameFormat;
            let Some(handle) = self.pool.acquire() else {
                return Err(CuError::from("No handle available for realsense"));
            };
            handle.with_inner_mut(|inner| {
                use std::ops::DerefMut;
                let dst = inner.deref_mut();
                if dst.len() != sample.payload().depths.len() {
                    return Err(CuError::from("handle data len doesnt match image data len"));
                }
                dst.copy_from_slice(&sample.payload().depths);
                Ok(())
            })?;
            new_msg.set_payload(CuDepthFrame::new(
                CuDepthFrameFormat {
                    width: DEPTH_FRAME_WIDTH,
                    height: DEPTH_FRAME_HEIGHT,
                    depth_scale: sample.depth_scale,
                    focal_len: sample.focal_len,
                },
                handle,
            ));
            self.last_seen = clock.now().into();
        }
        let t_after_recv = clock.now().as_nanos();

        let timeout_result = if clock.now().as_nanos() - self.last_seen > 500_000_000 {
            Err(CuError::new_with_cause(
                "no depth frames seen in 500 ms",
                std::io::Error::other("no depth frames seen in 500 ms"),
            ))
        } else {
            Ok(())
        };
        let t_end = clock.now().as_nanos();

        let total_ns = t_end.saturating_sub(t_start);
        if total_ns > 500_000 {
            eprintln!(
                "[DBG realsense] total={}µs  clear={}µs  depth_recv={}µs",
                total_ns / 1000,
                t_after_clear.saturating_sub(t_start) / 1000,
                t_after_recv.saturating_sub(t_after_clear) / 1000,
            );
        }

        timeout_result
    }
}

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for RealsenseSubscriber {
    type Output<'m> = output_msg!((Option<IceoryxDepthFrame<DEPTH_FRAME_SIZE>>, Option<ImuMsg>));
    type Resources<'r> = ();

    fn new(
        _config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
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
