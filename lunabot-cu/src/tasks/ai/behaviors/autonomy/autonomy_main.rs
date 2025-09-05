use bonsai_bt::Behavior::{self, Action, If, While};

use crate::tasks::ai::{
    action::LunabotAction,
    behaviors::autonomy::navigate::{self, navigate_behavior},
};

pub fn autonomy_main() -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)),
        vec![If(
            Box::new(navigate_behavior()),
            Box::new(todo!("dig behavior not yet implemented")),
            Box::new(todo!("navigate failiure handle not yet implemented")),
        )],
    )
}
