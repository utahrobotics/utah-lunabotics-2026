use bonsai_bt::Behavior::{self, Action, WhileAll};

use crate::tasks::ai::action::LunabotAction;

pub fn manual_ctrl_behavior() -> Behavior<LunabotAction> {
    WhileAll(
        Box::new(Action(LunabotAction::IsManual)),
        vec![
            Action(LunabotAction::SetLastSteering),
            Action(LunabotAction::SetLastLift),
            Action(LunabotAction::SetLastBucket),
            Action(LunabotAction::Yield),
        ],
    )
}
