use std::collections::VecDeque;

use common::{FromLunabase, Steering};
use embedded_common::ActuatorCommand;
use iceoryx_types::IceoryxOccupancyGrid;
use simple_motion::StaticNode;

use crate::ROOT_NODE;

pub struct LunabotBlackboard {
    pub root_node: StaticNode,
    pub latest_obstacle_map: IceoryxOccupancyGrid,
    pub last_lift: Option<i8>,
    pub last_bucket: Option<i8>,
    pub last_steering: Option<Steering>,
    /// Queue of actuator commands to be sent to the actuator and motor control tasks
    pub outgoing_actuator_msg_queue: VecDeque<ActuatorCommand>,
    pub outgoing_steering_msg_queue: VecDeque<Steering>,
}

impl Default for LunabotBlackboard {
    fn default() -> Self {
        Self {
            root_node: *ROOT_NODE.get().expect("ROOT_NODE not initialized"), // we should always have the root node, and if not then we might as well abort
            latest_obstacle_map: IceoryxOccupancyGrid::default(),
            last_lift: None,
            last_bucket: None,
            last_steering: None,
            outgoing_actuator_msg_queue: VecDeque::new(),
            outgoing_steering_msg_queue: VecDeque::new(),
        }
    }
}

impl LunabotBlackboard {
    pub fn update_with_msg(&mut self, msg: &common::FromLunabase) {
        if let Some([lift_actuator1, lift_actuator2]) = msg.get_lift_actuator_commands()
            && let FromLunabase::LiftActuators(val) = msg
        {
            self.last_lift = Some(*val);
            self.outgoing_actuator_msg_queue.push_back(lift_actuator1);
            self.outgoing_actuator_msg_queue.push_back(lift_actuator2);
        }

        if let Some([bucket_actuator1, bucket_actuator2]) = msg.get_bucket_actuator_commands()
            && let FromLunabase::BucketActuators(val) = msg
        {
            self.last_bucket = Some(*val);
            self.outgoing_actuator_msg_queue.push_back(bucket_actuator1);
            self.outgoing_actuator_msg_queue.push_back(bucket_actuator2);
        }

        match msg {
            common::FromLunabase::Steering(steering) => {
                self.last_steering = Some(*steering);
            }
            _ => {}
        }
    }
}
