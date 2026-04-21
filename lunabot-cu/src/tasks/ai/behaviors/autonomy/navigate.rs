use bonsai_bt::Behavior::{
    self, Action, If, Invert, Race, Select, Sequence, Wait, WaitForever, While,
};
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

impl ToString for NavigationGoal {
    fn to_string(&self) -> String {
        match self {
            Self::DigSite(arena) => format!("DigSite-{arena:?}"),
            Self::DumpSite(arena) => format!("DumpSite-{arena:?}"),
        }
    }
}

impl NavigationGoal {
    /// half width, half height
    pub fn to_center_and_halfsizes(&self) -> (Vector2<f32>, f32, f32) {
        match &self {
            NavigationGoal::DigSite(Arena::Artemis) => (Vector2::new(2.5, 3.75), 0.25, 1.0),
            NavigationGoal::DigSite(Arena::UcfLeft) => (Vector2::new(3.75, 2.37), 0.30, 1.0),
            NavigationGoal::DigSite(Arena::UcfRight) => (Vector2::new(3.75, -2.37), 0.30, 1.0),
            NavigationGoal::DumpSite(Arena::Artemis) => (Vector2::new(5.38, 1.0), 1.0, 0.25),
            NavigationGoal::DumpSite(Arena::UcfLeft) => (Vector2::new(6.8, 1.5), 1.0, 0.25),
            NavigationGoal::DumpSite(Arena::UcfRight) => (Vector2::new(6.8, -1.5), 1.0, 0.25),
        }
    }
}

pub fn navigate_behavior(goal: NavigationGoal) -> Behavior<LunabotAction> {
    Sequence(vec![
        // one extra yield to avoid busy looping, in case the stage was set in the same tick
        Action(LunabotAction::Yield),
        Action(LunabotAction::SetBTStatusMsg(format!(
            "Calculating path to {}",
            goal.to_string()
        ))),
        If(
            Box::new(calculate_path_behavior(goal)),
            Box::new(Action(LunabotAction::SetBTStatusMsg(format!(
                "Calculated path to {}",
                goal.to_string()
            )))),
            Box::new(Sequence(vec![
                Action(LunabotAction::SetBTStatusMsg(format!(
                    "Failed to calculate path to {}",
                    goal.to_string()
                ))),
                Action(LunabotAction::SetStage(common::LunabotStage::SoftStop)),
            ])),
        ),
        // hale's path follow shouldn't need this
        // Action(LunabotAction::RotateToFacePath),,
        Action(LunabotAction::SetBTStatusMsg(format!(
            "Moving to {}",
            goal.to_string()
        ))),
        Race(vec![
            Action(LunabotAction::FollowPath),
            While(
                Box::new(WaitForever),
                vec![
                    Wait(1.0),
                    If(
                        Box::new(calculate_path_behavior(goal)),
                        Box::new(Action(LunabotAction::Yield)),
                        Box::new(Sequence(vec![
                            Action(LunabotAction::SetBTStatusMsg(format!(
                                "Failed to calculate path to {}",
                                goal.to_string()
                            ))),
                            Action(LunabotAction::SetStage(common::LunabotStage::SoftStop)),
                        ])),
                    ),
                ],
            ),
        ]),
    ])
}

/// calculates a path to goal, if it fails once, the local obstacles are reset.
/// if the path calc fails once again after local obstacles are reset, then
/// global obstacles are reset also
fn calculate_path_behavior(goal: NavigationGoal) -> Behavior<LunabotAction> {
    Select(vec![
        Action(LunabotAction::CalculatePath(goal)),
        Invert(Box::new(Action(LunabotAction::ResetLocalObstacles))),
        Invert(Box::new(Action(LunabotAction::Yield))),
        // this behavior just waits until a requested obstcle reset is actually fulfilled
        // "obstacle reset requested" and "yield" will always return Success which is why we
        // need the invert to force the outer select to continue on
        Invert(Box::new(wait_for_new_frame())),
        Action(LunabotAction::CalculatePath(goal)),
        Invert(Box::new(Action(LunabotAction::ResetAllObstacles))),
        Invert(Box::new(Action(LunabotAction::Yield))),
        Invert(Box::new(wait_for_new_frame())),
        Action(LunabotAction::CalculatePath(goal)),
    ])
}

fn wait_for_new_frame() -> Behavior<LunabotAction> {
    While(
        Box::new(Sequence(vec![
            Action(LunabotAction::ObstacleResetRequested),
            Action(LunabotAction::LatestLocalMapReady),
        ])),
        vec![Action(LunabotAction::Yield)],
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
