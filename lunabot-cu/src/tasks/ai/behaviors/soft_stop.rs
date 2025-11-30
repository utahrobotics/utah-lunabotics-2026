use bonsai_bt::Behavior::{self, Action, AlwaysSucceed, Invert, Wait};
use common::Steering;

use crate::tasks::ai::action::LunabotAction;

pub fn soft_stop_behavior() -> bonsai_bt::Behavior<crate::tasks::ai::action::LunabotAction> {
    Behavior::WhileAll(
        Box::new(Action(LunabotAction::IsSoftStop)),
        vec![
            // Action(LunabotAction::Yield),
            Action(LunabotAction::SetSteering(Steering::new(0., 0.0, 0.0))),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
            AlwaysSucceed(Box::new(Action(LunabotAction::CalculatePath))),
            Wait(1.0),
        ],
    )
}
