use bonsai_bt::Behavior::{self, Action, AlwaysSucceed, Sequence, WaitForever};

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::{
        autonomy::{autonomy_main::autonomy_main, navigate::Arena}, manual_ctrl_behavior, soft_stop::soft_stop_behavior,
    },
};

/// the root of the behavior tree
pub fn teleop_behavior(arena: Arena) -> Behavior<LunabotAction> {
    Behavior::While(
        Box::new(WaitForever),
        vec![Sequence(vec![
            Action(LunabotAction::Yield),
            AlwaysSucceed(Box::new(soft_stop_behavior(arena))),
            AlwaysSucceed(Box::new(manual_ctrl_behavior())),
            AlwaysSucceed(Box::new(autonomy_main(arena))),
        ])],
    )
}
