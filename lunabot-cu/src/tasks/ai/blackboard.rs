use std::collections::VecDeque;

use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::ActuatorCommand;
use iceoryx_types::IceoryxOccupancyGrid;
use simple_motion::StaticNode;

use crate::ROOT_NODE;

#[derive(Clone, Debug)]
pub struct LunabotBlackboard {
    pub root_node: StaticNode,
    pub latest_obstacle_map: Option<IceoryxOccupancyGrid>,
    pub last_lift: Option<i8>,
    pub last_bucket: Option<i8>,
    pub last_steering: Option<Steering>,

    /// populated when the navigate command comes in
    /// there will likely be enough pathfinding params that this will end up encapsulated by some pathfinder struct
    pub navigate_destination: Option<(f32, f32)>,

    /// Queue of actuator commands to be sent to the actuator and motor control tasks
    pub outgoing_actuator_msg_queue: VecDeque<ActuatorCommand>,
    /// we only care about the latest steering msg
    /// the outgoing steering msg will really always be the same as the last_steering msg in manual mode
    /// but for autonomy we can't know that
    pub outgoing_steering_msg: Option<Steering>,

    /// when the user clicks continue mission in lunabase, the stage stored here is what the bot wil continue with
    pub last_mission: LunabotStage,

    pub current_mission: LunabotStage,
}

impl Default for LunabotBlackboard {
    fn default() -> Self {
        Self {
            root_node: *ROOT_NODE.get().expect("ROOT_NODE not initialized"), // we should always have the root node, and if not then we might as well abort
            outgoing_actuator_msg_queue: VecDeque::new(),
            outgoing_steering_msg: Some(Steering::new(0.0, 0.0, 0.0)),
            // when the user clicks continue mission for the first time, we move to manual
            last_mission: LunabotStage::Manual,
            current_mission: LunabotStage::SoftStop,
            latest_obstacle_map: None,
            last_lift: None,
            last_bucket: None,
            last_steering: None,
            navigate_destination: None,
        }
    }
}

impl LunabotBlackboard {
    pub fn update_with_msg(&mut self, msg: &common::FromLunabase) {
        if let FromLunabase::LiftActuators(val) = msg {
            self.last_lift = Some(*val);
        }

        if let FromLunabase::BucketActuators(val) = msg {
            self.last_bucket = Some(*val);
        }

        match msg {
            common::FromLunabase::Steering(steering) => {
                self.last_steering = Some(*steering);
            }
            common::FromLunabase::Navigate(destination) => {
                self.last_mission = LunabotStage::Autonomy;
                self.current_mission = LunabotStage::Autonomy;
                LUNABOT_STAGE.store(LunabotStage::Autonomy);
                self.navigate_destination = Some(*destination);
            }
            common::FromLunabase::DigDump(..) => {
                self.last_mission = LunabotStage::Autonomy;
                self.current_mission = LunabotStage::Autonomy;
                LUNABOT_STAGE.store(LunabotStage::Autonomy);
            }

            // for now if the user cancels autonomy we dump them back into manual on continue mission
            common::FromLunabase::SoftStop => {
                self.last_mission = LunabotStage::Manual;
                self.current_mission = LunabotStage::SoftStop;
                LUNABOT_STAGE.store(LunabotStage::SoftStop);
            }
            common::FromLunabase::Disconnect => {
                eprintln!("Lunabase Disconnected");
                self.current_mission = LunabotStage::SoftStop;
                LUNABOT_STAGE.store(LunabotStage::SoftStop);
            }
            common::FromLunabase::ContinueMission => {
                LUNABOT_STAGE.store(self.last_mission);
                self.current_mission = self.last_mission;
            }
            _ => {}
        }
    }
}
