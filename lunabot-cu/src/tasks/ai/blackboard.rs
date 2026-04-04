use std::collections::VecDeque;

use crate::pathfinding::OccupancyGrid;
use crate::{ROBOT_STATE, tasks::ai::jobs::Job};
use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::ActuatorCommand;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use std::time::Instant;

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
    pub path_follower: Option<Job<Steering, Vec<Vector2<f32>>>>,
    /// if a path finding job is running, it will be stored here
    pub path_finder: Option<Job<Vec<Vector2<f32>>, ()>>,

    // if we're digging moon dirt, the job will be stored here
    pub digger: Option<Job<ActuatorCommand, ()>>,
    pub dumper: Option<Job<ActuatorCommand, ()>>,

    /// rotation shim
    pub rotation_shim: Option<Job<Steering, ()>>,

    /// the calculated path from the path finder job
    pub calculated_path: Option<Vec<Vector2<f32>>>,
    /// any cell with a greater gradient will be expanded
    pub obstacle_gradient_threshold_expander: f32,
    /// threshold considered to be an obstacle by the pathfinder
    pub obstacle_gradient_threshold_pathfinder: f32,

    pub robot_radius: f32,
    pub last_non_zero_steering_pack: Option<Instant>,
    pub last_non_zero_lift_pack: Option<Instant>,
    pub last_non_zero_bucket_pack: Option<Instant>,
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
            digger: None,
            dumper: None,
            rotation_shim: None,
            obstacle_gradient_threshold_expander: 0.5,
            obstacle_gradient_threshold_pathfinder: 0.3,
            robot_radius: 0.5,
            last_non_zero_steering_pack: None,
            last_non_zero_lift_pack: None,
            last_non_zero_bucket_pack: None,
        }
    }
}

impl LunabotBlackboard {
    pub fn update_with_msg(&mut self, msg: &common::FromLunabase) {
        if let FromLunabase::LiftActuators(val) = msg {
            if *val != 0 {
                self.last_non_zero_lift_pack = Some(Instant::now());
            } else {
                self.last_non_zero_lift_pack = None;
            }

            self.last_lift = Some(*val);
        }

        if let FromLunabase::BucketActuators(val) = msg {
            if *val != 0 {
                self.last_non_zero_bucket_pack = Some(Instant::now());
            } else {
                self.last_non_zero_bucket_pack = None;
            }
            self.last_bucket = Some(*val);
        }

        match msg {
            common::FromLunabase::Steering(steering) => {
                let (left, right) = steering.get_left_and_right();
                if left != 0.0 || right != 0.0 {
                    self.last_non_zero_steering_pack = Some(Instant::now());
                } else {
                    self.last_non_zero_steering_pack = None;
                }

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
