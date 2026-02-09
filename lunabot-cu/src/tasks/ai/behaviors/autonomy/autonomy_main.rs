use bonsai_bt::Behavior::{self, Action, While};

use crate::tasks::ai::{action::LunabotAction, behaviors::autonomy::navigate::navigate_behavior};

pub fn autonomy_main() -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)),
        vec![
            navigate_behavior(), // todo: wrap this in an if statement
                                 // Box::new(todo!("dig behavior not yet implemented")),
                                 // Box::new(todo!("navigate failiure handle not yet implemented")),
        ],
    )
}
