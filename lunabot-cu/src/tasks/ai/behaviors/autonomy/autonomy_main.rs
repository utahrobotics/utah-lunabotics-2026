use bonsai_bt::Behavior::{self, Action, While};

use crate::tasks::ai::{action::LunabotAction, behaviors::autonomy::navigate::{Arena, NavigationGoal, navigate_behavior}};

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
            // 4. Dump
            // Action(LunabotAction::Dump),
        ],
    )
}
