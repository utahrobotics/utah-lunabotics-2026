use bonsai_bt::Behavior::{self, Action, AlwaysSucceed, Sequence, WaitForever, While};

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::{
        autonomy::{autonomy_main::{autonomy_main, open_loop_dig_from_starting, open_loop_dump_from_zero}, navigate::Arena}, manual_ctrl_behavior, soft_stop::soft_stop_behavior, test_motors_behavior,
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
            AlwaysSucceed(Box::new(test_motors_behavior())),
            AlwaysSucceed(Box::new(dig_mode())),
            AlwaysSucceed(Box::new(dump_mode()))
        ])],
    )
}

pub fn dig_mode() -> Behavior<LunabotAction> {
    While(Box::new(Action(LunabotAction::IsDig)), vec![
        Action(LunabotAction::SetBTStatusMsg("OPEN LOOP DIG".to_string())),
        open_loop_dig_from_starting(),
        Action(LunabotAction::SetStage(common::LunabotStage::SoftStop)),
        Action(LunabotAction::Yield)
    ])
}

pub fn dump_mode() -> Behavior<LunabotAction> {
    While(Box::new(Action(LunabotAction::IsDump)), vec![
        Action(LunabotAction::SetBTStatusMsg("OPEN LOOP DUMP".to_string())),
        open_loop_dump_from_zero(),
        Action(LunabotAction::SetStage(common::LunabotStage::SoftStop)),
        Action(LunabotAction::Yield)
    ])
}