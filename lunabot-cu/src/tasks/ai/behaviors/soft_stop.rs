use bonsai_bt::Behavior::{self, Action, Wait};
use common::Steering;
use embedded_common::{Actuator, ActuatorCommand};

use crate::tasks::ai::action::LunabotAction;

pub fn soft_stop_behavior() -> bonsai_bt::Behavior<crate::tasks::ai::action::LunabotAction> {
    Behavior::WhileAll(
        Box::new(Action(LunabotAction::IsSoftStop)),
        vec![
            Action(LunabotAction::SetSteering(Steering::new(0., 0.0, 0.0))),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
            Wait(1.0),
        ],
    )
}
