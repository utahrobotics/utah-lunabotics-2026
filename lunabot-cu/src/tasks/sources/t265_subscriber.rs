use cu29::{
    cutask::{CuSrcTask, Freezable},
    prelude::*,
};
use iceoryx_types::PoseMsg;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use iceoryx2::{
    node::NodeBuilder,
    port::subscriber::Subscriber,
    prelude::{LogLevel, ServiceName, set_log_level},
    service::ipc,
};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

pub struct T265Subscriber {
    last_seen: u64,
    /// subscribes to pose frames published by the realsense external task (T265)
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    pose_subscriber: Subscriber<ipc::Service, PoseMsg, ()>,
}

impl Freezable for T265Subscriber {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuSrcTask for T265Subscriber {
    type Output<'m> = output_msg!(PoseMsg);

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let serial_num = config
            .and_then(|c| c.get::<String>("serial_num"))
            .unwrap_or_else(|| "realsense/t265".to_string());
        let pose_service_str = format!("realsense/{serial_num}/pose");

        let pose_service_name = ServiceName::new(&pose_service_str)
            .map_err(|e| CuError::new_with_cause("invalid service name", e))?;

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("node create faliure", e))?;

        set_log_level(LogLevel::Debug);
        let pose_service = node
            .service_builder(&pose_service_name)
            .publish_subscribe::<PoseMsg>()
            .enable_safe_overflow(true)
            .subscriber_max_buffer_size(20)
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("service open error", e))?;

        let pose_subscriber = pose_service
            .subscriber_builder()
            .buffer_size(19)
            .create()
            .map_err(|e| CuError::new_with_cause("subscriber creation error", e))?;
        Ok(Self {
            last_seen: 0,
            pose_subscriber,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        new_msg.clear_payload();

        let mut output = None;

        // gather any pending pose samples from the realsense T265 publisher
        while let Some(sample) = self
            .pose_subscriber
            .receive()
            .map_err(|e| CuError::new_with_cause("subscriber receive failed", e))?
        {
            output = Some(sample.payload().clone());
            self.last_seen = clock.now().into();
        }

        if let Some(PoseMsg {
            mut position,
            mut quaternion,
        }) = output
        {
            let transformed_translation = Vector3::new(-position[2], position[0], position[1]);
            position[0] = transformed_translation.x;
            position[1] = -transformed_translation.y;
            position[2] = transformed_translation.z;

            let (qx, qy, qz, qw) = (quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

            quaternion[0] = -qz;
            quaternion[1] = -qx;
            quaternion[2] = qy;
            quaternion[3] = qw;
            new_msg.set_payload(PoseMsg {
                position,
                quaternion,
            });
        }

        if clock.now().as_nanos() - self.last_seen > 500_000_000 {
            return Err(CuError::new_with_cause(
                "no pose frames seen in 500 ms",
                std::io::Error::other("no pose frames seen in 500 ms"),
            ));
        }

        Ok(())
    }
}

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for T265Subscriber {
    type Output<'m> = output_msg!(PoseMsg);
    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { last_seen: 0 })
    }
    fn process<'o>(
        &mut self,
        _clock: &cu29::prelude::RobotClock,
        _new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        Ok(())
    }
}
