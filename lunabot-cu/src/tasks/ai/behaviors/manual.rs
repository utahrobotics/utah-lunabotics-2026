use bonsai_bt::Behavior::{self, Action, Wait, While, WhileAll};
use common::Steering;

use crate::tasks::ai::{action::LunabotAction, blackboard::LunabotBlackboard};

pub fn manual_ctrl_behavior() -> Behavior<LunabotAction> {
    WhileAll(
        Box::new(Action(LunabotAction::IsManual)),
        vec![
            Action(LunabotAction::SetLastSteering),
            Action(LunabotAction::SetLastLift),
            Action(LunabotAction::SetLastBucket),
            Action(LunabotAction::Yield)
        ],
    )
}
