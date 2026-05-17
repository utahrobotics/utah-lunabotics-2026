use crate::pathfinding::OccupancyGrid;
use crate::rerun_viz::RECORDER;
use crate::utils::rwlock_write_unpoison;
use crate::{ROBOT_STATE, tasks::ai::jobs::Job};
use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, Steering};
use embedded_common::ActuatorCommand;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use std::collections::VecDeque;
use std::ops::Deref;
use std::sync::{Arc, OnceLock, RwLock};
use std::time::Instant;

/// do not hold onto the locks for too long, it could stall the pipeline.
/// this could be an atomic cell but there might be heap data in it eventually
pub static BLACKBOARD_SHARED: OnceLock<Arc<RwLock<BlackboardShared>>> = OnceLock::new();

#[derive(Debug, Clone, Copy)]
pub struct BlackboardShared {
    /// reset just the local map, read by occupancy grid task
    pub reset_local_map: bool,
    /// reset both local and global, read by both tasks
    pub reset_map: bool,
    pub enable_apriltags: bool,

    /// by default the params come from the config passed to the realsense occupancy grid task, but we can dynamically change them
    /// through the AI
    pub sigma_spatial: Option<f32>,
    pub sigma_range: Option<f32>,
}

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

    /// stores the last bucket actuator message received from lunabase
    pub last_dumper: Option<i8>,

    /// stores the last steering message recieved from the lunabase
    pub last_steering: Option<Steering>,

    /// Queue of actuator commands to be sent to the actuator and motor control tasks
    pub outgoing_actuator_msg_queue: VecDeque<ActuatorCommand>,
    /// we only care about the latest steering msg
    /// the outgoing steering msg will really always be the same as the last_steering msg in manual mode
    /// but for autonomy we can't know that
    pub outgoing_steering_msg: Option<Steering>,

    /// Status msg describing what branch of the bt we are in.
    pub outgoing_bt_status_msg: Option<String>,

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

    /// multi point rotation shim
    pub multi_point_rotation_shim: Option<Job<Steering, ()>>,

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
    pub last_non_zero_dumper_pack: Option<Instant>,

    /// don't hold onto guards for too long (longer than 200 microseconds is bad)
    pub blackboard_shared: Arc<RwLock<BlackboardShared>>,
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
            yielded: false,
            path_follower: None,
            digger: None,
            dumper: None,
            rotation_shim: None,
            multi_point_rotation_shim: None,
            obstacle_gradient_threshold_expander: 0.5,
            obstacle_gradient_threshold_pathfinder: 0.3,
            robot_radius: 0.5,
            last_non_zero_steering_pack: None,
            last_non_zero_lift_pack: None,
            last_non_zero_bucket_pack: None,
            outgoing_bt_status_msg: None,
            last_non_zero_dumper_pack: None,
            last_dumper: None,
            blackboard_shared: Arc::new(RwLock::new(BlackboardShared {
                reset_local_map: false,
                reset_map: false,
                enable_apriltags: true,
                sigma_range: None,
                sigma_spatial: None,
            })),
        }
    }
}

impl LunabotBlackboard {
    pub fn update_with_msg(&mut self, msg: &common::FromLunabase) {
        match msg {
            common::FromLunabase::LiftActuators(val) => {
                if *val != 0 {
                    self.last_non_zero_lift_pack = Some(Instant::now());
                } else {
                    self.last_non_zero_lift_pack = None;
                }
                self.last_lift = Some(*val);
            }
            common::FromLunabase::DumperActuators(val) => {
                if *val != 0 {
                    self.last_non_zero_dumper_pack = Some(Instant::now());
                } else {
                    self.last_non_zero_dumper_pack = None;
                }
                self.last_dumper = Some(*val);
            }
            common::FromLunabase::BucketActuators(val) => {
                if *val != 0 {
                    self.last_non_zero_bucket_pack = Some(Instant::now());
                } else {
                    self.last_non_zero_bucket_pack = None;
                }
                self.last_bucket = Some(*val);
            }
            common::FromLunabase::Steering(steering) => {
                if let Some(rec) = RECORDER.get() {
                    rec.recorder
                        .log("steering", &rerun::TextLog::new(format!("{steering:?}")));
                }
                let (left, right) = steering.get_left_and_right();
                if left != 0.0 || right != 0.0 {
                    self.last_non_zero_steering_pack = Some(Instant::now());
                } else {
                    self.last_non_zero_steering_pack = None;
                }

                self.last_steering = Some(*steering);
            }
            common::FromLunabase::Navigate => {
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
            common::FromLunabase::TestMotors => {
                self.current_mission = LunabotStage::TestMotors;
                LUNABOT_STAGE.store(LunabotStage::TestMotors);
            }
            common::FromLunabase::Dig => {
                self.current_mission = LunabotStage::Dig;
                LUNABOT_STAGE.store(LunabotStage::Dig);
            }
            common::FromLunabase::Dump => {
                self.current_mission = LunabotStage::Dump;
                LUNABOT_STAGE.store(LunabotStage::Dump);
            }
            common::FromLunabase::ResetObstacles => {
                self.latest_local_map = None;
                rwlock_write_unpoison(self.blackboard_shared.deref()).reset_map = true;
            }
            common::FromLunabase::EnableApriltags => {
                rwlock_write_unpoison(&self.blackboard_shared).enable_apriltags = true;
            }
            common::FromLunabase::DisableApriltags => {
                rwlock_write_unpoison(&self.blackboard_shared).enable_apriltags = false;
            }
            common::FromLunabase::SetSigmaSpatial(new_sigma) => {
                rwlock_write_unpoison(&self.blackboard_shared).sigma_spatial = Some(*new_sigma);
            }
            common::FromLunabase::SetSigmaRange(new_sigma) => {
                rwlock_write_unpoison(&self.blackboard_shared).sigma_range = Some(*new_sigma);
            }
            FromLunabase::LiftShake | FromLunabase::StartPercuss | FromLunabase::StopPercuss => {}
        }
    }
}
