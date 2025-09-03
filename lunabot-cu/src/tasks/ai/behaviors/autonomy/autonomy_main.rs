use bonsai_bt::Behavior::{self, Action, While};

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::autonomy::navigate::{self, navigate_behavior},
};

pub fn autonomy_main() -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)),
        vec![navigate_behavior()],
    )
}
