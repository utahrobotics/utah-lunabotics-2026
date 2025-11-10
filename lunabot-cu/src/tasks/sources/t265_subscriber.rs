use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    cutask::{CuSrcTask, Freezable},
    prelude::*,
};
use iceoryx_types::PoseMsg;
use iceoryx2::{
    node::NodeBuilder,
    port::subscriber::Subscriber,
    prelude::{LogLevel, ServiceName, set_log_level},
    service::ipc,
};
use nalgebra::{Isometry3, Quaternion, UnitQuaternion, Vector3};
use simple_motion::StaticNode;

use crate::ROOT_NODE;

pub struct T265Subscriber {
    last_seen: u64,
    /// subscribes to pose frames published by the realsense external task (T265)
    pose_subscriber: Subscriber<ipc::Service, PoseMsg, ()>,
    node: StaticNode,
}

impl Freezable for T265Subscriber {}

impl CuSrcTask for T265Subscriber {
    type Output<'m> = output_msg!(EncodableIsometry);

    fn new(config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let serial_num = config
            .and_then(|c| c.get::<String>("serial_num"))
            .unwrap_or_else(|| "realsense/t265".to_string());
        let pose_service_str = format!("realsense/{serial_num}/pose");

        let node_name: String = config
            .and_then(|c| c.get::<String>("node"))
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
        let t265_node = ROOT_NODE
            .get()
            .expect("root node should be defined")
            .get_node_with_name(&node_name)
            .expect("node not found in chain");
        Ok(Self {
            last_seen: 0,
            pose_subscriber,
            node: t265_node,
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
        }) = output
        {
            // Apply coordinate system transform from T265 to robot coordinate frame
            let transformed_translation = Vector3::new(-position[2], position[0], position[1]);
            let t265_translation = Vector3::new(
                transformed_translation.x,
                -transformed_translation.y,
                transformed_translation.z,
            );

            let (qx, qy, qz, qw) = (quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            let t265_rotation =
                UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qw, -qz, -qx, qy));

            // T265 sensor's pose in the world (its tracking frame)
            let t265_pose_in_world = Isometry3::from_parts(t265_translation.into(), t265_rotation);

            // Get the transform from robot base to T265 sensor
            // "T265 is at this position/orientation relative to the robot base"
            let base_to_t265 = self.node.get_isometry_from_base().cast::<f32>();

            // Calculate robot base pose in world
            // robot_pose * base_to_t265 = t265_pose_in_world
            let robot_pose = t265_pose_in_world * base_to_t265.inverse();

            new_msg.set_payload(EncodableIsometry::from_na(&robot_pose.cast::<f64>()));
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
