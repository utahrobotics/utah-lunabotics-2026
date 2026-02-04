use bonsai_bt::Behavior::{self, Action, AlwaysSucceed, Sequence, WaitForever, WhenAll};

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::{
        autonomy::autonomy_main::autonomy_main, manual_ctrl_behavior, soft_stop::soft_stop_behavior,
    },
};

/// the root of the behavior tree
pub fn teleop_behavior() -> Behavior<LunabotAction> {
    Behavior::While(
        Box::new(WaitForever),
        vec![WhenAll(vec![
            // inclination tracker always runs in background
            Action(LunabotAction::TrackInclination),
            // all other behavior stems from this sequence
            Sequence(vec![
                Action(LunabotAction::Yield),
                AlwaysSucceed(Box::new(soft_stop_behavior())),
                AlwaysSucceed(Box::new(manual_ctrl_behavior())),
                AlwaysSucceed(Box::new(autonomy_main())),
            ]),
        ])],
    )
}
