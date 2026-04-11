use std::sync::Arc;

use bonsai_bt::{BT, Event, UpdateArgs};
use common::{FromLunabase, Steering};
use cu29::prelude::*;
use cu29::{
    cutask::{CuTask, Freezable},
    input_msg,
};
use embedded_common::ActuatorCommand;

use crate::pathfinding::OccupancyGrid;
use crate::tasks::ai::action::LunabotAction;
use crate::tasks::ai::behaviors::teleop::teleop_behavior;
use crate::tasks::ai::blackboard::{BLACKBOARD_SHARED, LunabotBlackboard};
use crate::utils::nanos_to_secs;

pub struct LunabotAi {
    bt: BT<LunabotAction, LunabotBlackboard>,
    last_tick_nanos: u64,
}

impl Freezable for LunabotAi {}

impl CuTask for LunabotAi {
    // the occupancy grid recved here is the local occupancy grid
    type Input<'m> = input_msg!('m, FromLunabase, OccupancyGrid);

    // (Steering, ActuatorCommand, calculated path)
    type Output<'m> = (CuMsg<Steering>, CuMsg<ActuatorCommand>, CuMsg<Vec<[f32; 2]>>);
    type Resources<'r> = ();

    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let robot_radius_meters = config
            .and_then(|c| {
                c.get::<f64>("robot_radius_meters")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.3) as f32;

        let obstacle_gradient_threshold_expander = config
            .and_then(|c| {
                c.get::<f64>("obstacle_gradient_threshold_expander")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.5) as f32;
        let obstacle_gradient_threshold_pathfinder = config
            .and_then(|c| {
                c.get::<f64>("obstacle_gradient_threshold_pathfinder")
                    .expect("failed to deserialize")
            })
            .unwrap_or(0.3) as f32;
        let mut blackboard = LunabotBlackboard::default();
        BLACKBOARD_SHARED.get_or_init(|| Arc::clone(&blackboard.blackboard_shared));
        blackboard.obstacle_gradient_threshold_expander = obstacle_gradient_threshold_expander;
        blackboard.obstacle_gradient_threshold_pathfinder = obstacle_gradient_threshold_pathfinder;
        blackboard.robot_radius = robot_radius_meters;
        let behavior = teleop_behavior();
        let bt = BT::new(behavior, blackboard);
        Ok(Self {
            bt: bt,
            last_tick_nanos: 0,
        })
    }

    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        self.last_tick_nanos = clock.now().into();
        Ok(())
    }

    /// drains the outgoing steering and actuator command queues, updates the blackboard with Inputs, ticks the behavior tree
    fn process<'i, 'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        let dt = nanos_to_secs(clock.now().as_nanos() - self.last_tick_nanos);

        let e: Event = UpdateArgs { dt }.into();
        if let Some(from_lunabase) = input.0.payload() {
            self.bt.blackboard_mut().update_with_msg(from_lunabase);
        }

        if let Some(map) = input.1.payload() {
            self.bt.blackboard_mut().latest_local_map = Some(map.clone());
        }

        self.bt
            .tick(&e, &mut |args, blackboard| args.action.handle(blackboard));
        self.last_tick_nanos = clock.now().into();
        if let Some(actuator_cmd) = self
            .bt
            .blackboard_mut()
            .outgoing_actuator_msg_queue
            .pop_front()
        {
            output.1.set_payload(actuator_cmd);
        }
        if let Some(steering_cmd) = self.bt.blackboard_mut().outgoing_steering_msg.take() {
            output.0.set_payload(steering_cmd);
        }
        if let Some(ref latest_calculated_path) = self.bt.blackboard().calculated_path {
            output.2.set_payload(latest_calculated_path.iter().map(|node| {
                [node.x, node.y]
            }).collect());
        }
        Ok(())
    }
}
