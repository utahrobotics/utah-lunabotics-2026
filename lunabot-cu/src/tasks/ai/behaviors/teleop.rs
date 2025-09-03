use bonsai_bt::Behavior::{self, Action, Sequence, WaitForever};

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::{
        autonomy::autonomy_main::autonomy_main, manual_ctrl_behavior, soft_stop::soft_stop_behavior,
    },
};

pub fn teleop_behavior() -> Behavior<LunabotAction> {
    Behavior::While(
        Box::new(WaitForever),
        vec![Sequence(vec![
            soft_stop_behavior(),
            manual_ctrl_behavior(),
            autonomy_main(),
        ])],
    )
}
