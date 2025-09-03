use bonsai_bt::Behavior::{self, Action, While};
use common::Steering;

use crate::tasks::ai::{action::LunabotAction, blackboard::LunabotBlackboard};

pub fn manual_ctrl_behavior(bb: &LunabotBlackboard) -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsManual)),
        vec![
            Action(LunabotAction::SetSteering(
                bb.last_steering.unwrap_or(Steering::new(0.0, 0.0, 0.0)),
            )),
            // using curly brackets feels like a hacky work around
            Action({
                if bb.last_lift.is_some() {
                    LunabotAction::SetLift(bb.last_lift.unwrap())
                } else {
                    LunabotAction::None
                }
            }),
            Action({
                if bb.last_bucket.is_some() {
                    LunabotAction::SetLift(bb.last_bucket.unwrap())
                } else {
                    LunabotAction::None
                }
            }),
        ],
    )
}
