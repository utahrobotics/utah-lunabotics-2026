use bonsai_bt::Behavior::{self, Action, AlwaysSucceed, Wait};
use common::Steering;

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::autonomy::navigate::{Arena, NavigationGoal},
};

pub fn soft_stop_behavior(
    arena: Arena,
) -> bonsai_bt::Behavior<crate::tasks::ai::action::LunabotAction> {
    Behavior::WhileAll(
        Box::new(Action(LunabotAction::IsSoftStop)),
        vec![
            Action(LunabotAction::CancelJobs),
            Action(LunabotAction::SetSteering(Steering::new(0., 0.0, 0.0))),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
            Action(LunabotAction::SetDumper(0)),
            // so we can preview the path for the first place the robot wants to go
            AlwaysSucceed(Box::new(Action(LunabotAction::CalculatePath(
                NavigationGoal::DigSite(arena),
            )))),
            Wait(1.0),
        ],
    )
}
