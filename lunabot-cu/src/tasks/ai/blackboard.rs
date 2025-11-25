use std::{collections::VecDeque, sync::{Arc, RwLock}};

use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::ActuatorCommand;
use iceoryx_types::IceoryxOccupancyGrid;
use kalman_filter::{SimpleSquareMatrix, SimpleVector};
use simple_motion::StaticNode;

use crate::{ROBOT_STATE, tasks::ai::jobs::Job};

#[derive(Debug)]
pub struct LunabotBlackboard {
    /// Keeps track of the position of all parts of the robot
    pub kinematic_root: StaticNode,
    /// Represents the robot's overall state (position, velocity, acceleration, orientation, angular velocity)
    pub kalman_state: Arc<RwLock<SimpleVector<15>>>,
    /// Stores the covariance matrix of the robot's overall state
    pub kalman_variances: Arc<RwLock<SimpleSquareMatrix<15>>>,

    /// TODO fill out this comment
    pub latest_obstacle_map: Option<IceoryxOccupancyGrid>,

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

    /// when the user clicks continue mission in lunabase, the stage stored here is what the bot wil continue with
    pub last_mission: LunabotStage,

    pub current_mission: LunabotStage,

    pub yielded: bool,

    /// if a path following long running task is going, the job will be stored here
    pub path_follower: Option<Job<Steering>>,
}

impl Default for LunabotBlackboard {
    fn default() -> Self {
        Self {
            kinematic_root: ROBOT_STATE.get().expect("ROBOT_STATE not initialized").kinematic_root, // we should always have the root node, and if not then we might as well abort
            kalman_state: Arc::clone(&ROBOT_STATE.get().expect("ROBOT_STATE not initialized").kalman_state),
            kalman_variances: Arc::clone(&ROBOT_STATE.get().expect("ROBOT_STATE not initialized").kalman_variances),
            // TODO transfer kalman filter state as well
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
