use bonsai_bt::Behavior::{self, Action, Wait, While};

use crate::tasks::ai::{action::LunabotAction, behaviors::autonomy::navigate::{Arena, NavigationGoal, back_up_for, navigate_behavior}};

pub fn autonomy_main(arena: Arena) -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)),
        vec![
            // 1. go to dig site
            navigate_behavior(NavigationGoal::DigSite(arena)),
            // 2. dig
            // Action(LunabotAction::Dig),

            // 3. Navigate to dump site
            navigate_behavior(NavigationGoal::DumpSite(arena)),
            back_up_for(0.5),
            Wait(5.5)
            // 4. Dump
            // Action(LunabotAction::Dump),
        ],
    )
}
