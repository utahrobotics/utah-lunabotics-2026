use cu_bincode::{Decode, Encode};
use cu_spatial_payloads::EncodableIsometry;
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
use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use serde::Deserialize;
use simple_motion::StaticNode;

use crate::ROBOT_STATE;

pub struct T265Subscriber {
    last_seen: u64,
    /// subscribes to pose frames published by the realsense external task (T265)
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    pose_subscriber: Subscriber<ipc::Service, PoseMsg, ()>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    node: StaticNode,
    /// Initial yaw offset captured on first pose to align T265's arbitrary tracking frame with world
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    initial_yaw_offset: Option<f32>,

    /// we only use the deltas between poses in the localizer
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    velocity_variance: f64,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    angular_velocity_variance: f64,
}

#[derive(Encode, Decode, Clone, Copy, Serialize, Debug, Deserialize)]
pub struct T265Msg {
    pub pose: EncodableIsometry,
    pub velocity_variance: f64,
    pub angular_velocity_variance: f64,
}

impl Default for T265Msg {
    fn default() -> Self {
        Self {
            pose: EncodableIsometry::default(),
            velocity_variance: 1.0,
            angular_velocity_variance: 1.0,
        }
    }
}

impl Freezable for T265Subscriber {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuSrcTask for T265Subscriber {
    type Output<'m> = output_msg!(T265Msg);
    type Resources<'r> = ();

    fn new(config: Option<&cu29::prelude::ComponentConfig>, _resources: Self::Resources<'_>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let serial_num = config
            .and_then(|c| c.get::<String>("serial_num").expect("failed to deserialize"))
            .unwrap_or_else(|| "realsense/t265".to_string());
        let pose_service_str = format!("realsense/{serial_num}/pose");

        let node_name: String = config
            .and_then(|c| c.get::<String>("node").expect("failed to deserialize"))
            .expect("must provide node name in chain");

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
        let t265_node = ROBOT_STATE
            .get()
            .expect("root node should be defined")
            .kinematic_root
            .get_node_with_name(&node_name)
            .expect("node not found in chain");

        let velocity_variance = config
            .and_then(|c| c.get::<f64>("t265_velocity_variance").expect("failed to deserialize"))
            .unwrap_or(1.0);

        let angular_velocity_variance = config
            .and_then(|c| c.get::<f64>("t265_angular_velocity_variance").expect("failed to deserialize"))
            .unwrap_or(1.0);

        Ok(Self {
            last_seen: 0,
            pose_subscriber,
            node: t265_node,
            initial_yaw_offset: None,
            velocity_variance,
            angular_velocity_variance,
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
            position,
            quaternion,
            confidence,
        }) = output
        {
            // T265 to robot coordinate frame

            use iceoryx_types::T265Confidence;
            let transformed_translation = Vector3::new(-position[2], position[0], position[1]);
            let t265_translation = Vector3::new(
                transformed_translation.x,
                -transformed_translation.y,
                transformed_translation.z,
            );

            let (qx, qy, qz, qw) = (quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            let t265_rotation =
                UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qw, -qz, -qx, qy));

            let t265_pose = Isometry3::from_parts(t265_translation.into(), t265_rotation);

            let base_to_t265 = self.node.get_isometry_from_base().cast::<f32>();

            let robot_pose = t265_pose * base_to_t265.inverse();

            // t264 starts with a basically arbitrary "twist" error
            if self.initial_yaw_offset.is_none() {
                let initial_yaw = robot_pose.rotation.euler_angles().2;
                self.initial_yaw_offset = Some(initial_yaw);
            }

            // align with world frame (robot starts facing +X)
            let yaw_correction = UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                -self.initial_yaw_offset.unwrap(),
            );

            let corrected_robot_pose =
                yaw_correction * Isometry3::from_parts(robot_pose.translation, robot_pose.rotation);

            let (velocity_variance, angular_velocity_variance) =
                if confidence == T265Confidence::High {
                    (self.velocity_variance, self.angular_velocity_variance)
                } else if confidence == T265Confidence::Medium {
                    (
                        self.velocity_variance * 2.,
                        self.angular_velocity_variance * 2.,
                    )
                } else if confidence == T265Confidence::Low {
                    (
                        self.velocity_variance * 5.,
                        self.angular_velocity_variance * 5.,
                    )
                } else {
                    (
                        self.velocity_variance * 30.,
                        self.angular_velocity_variance * 30.,
                    )
                };

            let payload = T265Msg {
                pose: EncodableIsometry::from_na(&corrected_robot_pose.cast::<f64>()),
                velocity_variance: velocity_variance,
                angular_velocity_variance: angular_velocity_variance,
            };
            new_msg.set_payload(payload);
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
    type Resources<'r> = ();
    fn new(config: Option<&cu29::prelude::ComponentConfig>, _resources: Self::Resources<'_>) -> cu29::CuResult<Self>
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
