use bonsai_bt::Behavior::{self, WaitForever};

use crate::tasks::ai::{
    action::LunabotAction,
    blackboard::{self, LunabotBlackboard},
};

pub fn teleop_behavior(bb: &LunabotBlackboard) -> Behavior<LunabotAction> {
    // stay in manual control unless the lunabot stage goes to SoftStop
    // stay in soft stop unless the lunabot stage goes to Autonomy or TeleOp (lets just ignore Autonomy stage for now)
    // go back to soft stop if the lunabot stage is soft stop
    todo!()
}
