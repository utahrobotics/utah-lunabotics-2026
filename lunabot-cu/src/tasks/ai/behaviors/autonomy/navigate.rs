use bonsai_bt::Behavior::{self, Action, If, Invert, Sequence, While};
use common::Steering;

use crate::tasks::ai::action::LunabotAction;

pub fn navigate_behavior() -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)),
        vec![Sequence(vec![
            Action(LunabotAction::IsObstacleMapReady),
            Action(LunabotAction::CalculatePath),
            While(
                // CheckNavigation returns running if the robot is en route to the destination
                // returns success if the robot has reached the destination
                Box::new(Action(LunabotAction::CheckNavigation)),
                vec![
                    hardcoded_navigation(),
                    // // recalculate every 20 cm for now
                    // Action(LunabotAction::CalculatePath),
                    // If(
                    //     // if following the path fails that means we are stuck
                    //     // TODO: store the meters parameter in the blackboard
                    //     Box::new(Action(LunabotAction::FollowPathFor(0.20))),
                    //     Box::new(Action(LunabotAction::None)), // continue
                    //     // we should probably store some max get unstuck tries parameter somewhere
                    //     Box::new(Action(LunabotAction::GetUnstuck)),
                    // ),
                ],
            ),
        ])],
    )
}

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
