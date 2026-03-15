use bonsai_bt::Behavior::{self, Action, Race, Sequence, Wait, WaitForever, While};
use common::Steering;

use crate::tasks::ai::action::LunabotAction;

pub fn navigate_behavior() -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)), // this is autonomy node is technically redundant
        vec![Sequence(vec![
            // one extra yield to avoid busy looping, in case the stage was set in the same tick
            Action(LunabotAction::Yield),
            Action(LunabotAction::CalculatePath),
            // hale's path follow shouldn't need this
            // Action(LunabotAction::RotateToFacePath),
            Race(vec![
                Action(LunabotAction::FollowPath),
                While(
                    Box::new(WaitForever),
                    vec![Wait(1.0), Action(LunabotAction::CalculatePath)],
                ),
            ]),
        ])],
    )
}

#[allow(unused)]
fn set_stage(stage: common::LunabotStage) -> Behavior<LunabotAction> {
    Sequence(vec![Action(LunabotAction::SetStage(stage))])
}

#[allow(unused, dead_code)]
pub fn hardcoded_navigation() -> Behavior<LunabotAction> {
    Sequence(vec![
        While(
            Box::new(Behavior::Wait(2.0)),
            vec![
                Action(LunabotAction::SetSteering(Steering::new(-0.5, 0.5, 1.0))),
                Action(LunabotAction::Yield),
            ],
        ),
        While(
            Box::new(Behavior::Wait(2.0)),
            vec![
                Action(LunabotAction::SetSteering(Steering::new(-1.0, 1.0, 1.0))),
                Action(LunabotAction::Yield),
            ],
        ),
        While(
            Box::new(Behavior::Wait(0.3)),
            vec![
                Action(LunabotAction::SetSteering(Steering::new(0.3, 0.3, 1.0))),
                Action(LunabotAction::Yield),
            ],
        ),
    ])
}
