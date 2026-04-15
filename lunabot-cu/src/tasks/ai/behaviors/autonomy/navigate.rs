use bonsai_bt::Behavior::{self, Action, Race, Sequence, Wait, WaitForever, While};
use common::Steering;
use nalgebra::Vector2;
use serde::Deserialize;

use crate::tasks::ai::action::LunabotAction;

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename_all = "lowercase")]
pub enum Arena {
    Artemis,
    UcfRight,
    UcfLeft,
}

#[derive(Clone, Copy, Debug)]
pub enum NavigationGoal {
    DigSite(Arena),
    DumpSite(Arena),
}

impl NavigationGoal {
    /// half width, half height
    pub fn to_center_and_halfsizes(&self) -> (Vector2<f32>, f32, f32) {
        match &self {
            NavigationGoal::DigSite(Arena::Artemis) => {
                (Vector2::new(2.5,3.75), 0.25, 1.0)
            },
            NavigationGoal::DigSite(Arena::UcfLeft) => {
                todo!()
            },
            NavigationGoal::DigSite(Arena::UcfRight) => todo!(),
            NavigationGoal::DumpSite(Arena::Artemis) => {
                (Vector2::new(5.38,1.0), 1.0, 0.5)
            },
            NavigationGoal::DumpSite(Arena::UcfLeft) => todo!(),
            NavigationGoal::DumpSite(Arena::UcfRight) => todo!()
        }
    }
}

pub fn navigate_behavior(goal: NavigationGoal) -> Behavior<LunabotAction> {
    Sequence(vec![
        // one extra yield to avoid busy looping, in case the stage was set in the same tick
        Action(LunabotAction::Yield),
        Action(LunabotAction::CalculatePath(goal)),
        // hale's path follow shouldn't need this
        // Action(LunabotAction::RotateToFacePath),,
        Race(vec![
            Action(LunabotAction::FollowPath),
            While(
                Box::new(WaitForever),
                vec![Wait(1.0), Action(LunabotAction::CalculatePath(goal))],
            ),
        ])
    ])
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
