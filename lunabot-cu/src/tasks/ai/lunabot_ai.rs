use bonsai_bt::{BT, Event, UpdateArgs};
use common::{FromLunabase, Steering};
use cu29::prelude::*;
use cu29::{
    cutask::{CuTask, Freezable},
    input_msg,
};
use embedded_common::ActuatorCommand;

use crate::tasks::OccupancyGrid;
use crate::tasks::ai::action::LunabotAction;
use crate::tasks::ai::behaviors::teleop::teleop_behavior;
use crate::tasks::ai::blackboard::LunabotBlackboard;
use crate::utils::nanos_to_secs;

pub struct LunabotAi {
    bt: BT<LunabotAction, LunabotBlackboard>,
    last_tick_nanos: u64,
}

impl Freezable for LunabotAi {}

impl CuTask for LunabotAi {
    // the occupancy grid recved here is the local occupancy grid
    type Input<'m> = input_msg!('m, FromLunabase, OccupancyGrid);

    // (Steering, ActuatorCommand)
    type Output<'m> = (CuMsg<Steering>, CuMsg<ActuatorCommand>);
    type Resources<'r> = ();

    fn new(_config: Option<&cu29::prelude::ComponentConfig>, _resources: Self::Resources<'_>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let blackboard = LunabotBlackboard::default();
        let behavior = teleop_behavior();
        let mut bt = BT::new(behavior, blackboard);
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
        Ok(())
    }
    
}
