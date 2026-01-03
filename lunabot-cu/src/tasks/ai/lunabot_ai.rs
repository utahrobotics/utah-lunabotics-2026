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
    type Output<'m> = output_msg!((Option<Steering>, Option<ActuatorCommand>));

    fn new(_config: Option<&cu29::prelude::ComponentConfig>) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let blackboard = LunabotBlackboard::default();
        let behavior = teleop_behavior();
        let mut bt = BT::new(behavior, blackboard);
        let viz = bt.get_graphviz();
        println!("GRAPHVIZ: {viz}");
        Ok(Self {
            bt: bt,
            last_tick_nanos: 0,
        })
    }

    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        self.last_tick_nanos = clock.now().into();
        Ok(())
    }

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
            self.bt.blackboard_mut().latest_obstacle_map = Some(map.clone());
        }

        self.bt
            .tick(&e, &mut |args, blackboard| args.action.handle(blackboard));
        self.last_tick_nanos = clock.now().into();
        let mut payload = (None, None);
        if let Some(actuator_cmd) = self
            .bt
            .blackboard_mut()
            .outgoing_actuator_msg_queue
            .pop_front()
        {
            payload.1 = Some(actuator_cmd);
        }
        if let Some(steering_cmd) = self.bt.blackboard_mut().outgoing_steering_msg.take() {
            payload.0 = Some(steering_cmd);
        }
        if payload.0.is_some() || payload.1.is_some() {
            output.set_payload(payload);
            return Ok(());
        } else {
            output.clear_payload();
        }

        Ok(())
    }
}
