use std::collections::VecDeque;

use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::ActuatorCommand;
use nalgebra::Vector2;
use simple_motion::StaticNode;

use crate::{
    ROBOT_STATE,
    tasks::{OccupancyGrid, ai::jobs::Job},
};

#[derive(Debug)]
pub struct LunabotBlackboard {
    /// Keeps track of the position of all parts of the robot
    pub kinematic_root: StaticNode,

    /// TODO fill out this comment
    pub latest_local_map: Option<OccupancyGrid>,

    /// stores the last lift actuator message received from lunabase
    pub last_lift: Option<i8>,

    /// stores the last bucket actuator message received from lunabase
    pub last_bucket: Option<i8>,

    /// stores the last steering message recieved from the lunabase
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

    pub current_mission: LunabotStage,

    pub yielded: bool,

    /// if a path following long running task is going, the job will be stored here
    pub path_follower: Option<Job<Steering>>,
    /// if a path finding job is running, it will be stored here
    pub path_finder: Option<Job<Vec<Vector2<f32>>>>,
    /// the calculated path from the path finder job
    pub calculated_path: Option<Vec<Vector2<f32>>>,
}

impl Default for LunabotBlackboard {
    fn default() -> Self {
        Self {
            kinematic_root: ROBOT_STATE
                .get()
                .expect("ROBOT_STATE not initialized")
                .kinematic_root, // we should always have the root node, and if not then we might as well abort

            outgoing_actuator_msg_queue: VecDeque::new(),
            outgoing_steering_msg: Some(Steering::new(0.0, 0.0, 0.0)),
            current_mission: LunabotStage::SoftStop,
            latest_local_map: None,
            last_lift: None,
            last_bucket: None,
            path_finder: None,
            calculated_path: None,
            last_steering: None,
            navigate_destination: None,
            yielded: false,
            path_follower: None,
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
                self.current_mission = LunabotStage::Autonomy;
                LUNABOT_STAGE.store(LunabotStage::Autonomy);
                self.navigate_destination = Some(*destination);
            }
            common::FromLunabase::DigDump(..) => {
                self.current_mission = LunabotStage::Autonomy;
                LUNABOT_STAGE.store(LunabotStage::Autonomy);
            }
            common::FromLunabase::SoftStop => {
                self.current_mission = LunabotStage::SoftStop;
                LUNABOT_STAGE.store(LunabotStage::SoftStop);
            }
            common::FromLunabase::Disconnect => {
                self.current_mission = LunabotStage::SoftStop;
                LUNABOT_STAGE.store(LunabotStage::SoftStop);
            }
            common::FromLunabase::Manual => {
                self.current_mission = LunabotStage::Manual;
                LUNABOT_STAGE.store(LunabotStage::Manual);
            }
            _ => {}
        }
    }
}
