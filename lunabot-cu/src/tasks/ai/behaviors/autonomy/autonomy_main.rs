use bonsai_bt::Behavior::{self, Action, Sequence, Wait, While};
use common::Steering;

use crate::tasks::ai::{action::LunabotAction, behaviors::{autonomy::navigate::{Arena, NavigationGoal, back_up_for, navigate_behavior}, steer_for_n_seconds}};
const DIG_TRAVERSAL_TIME: f64 = 4.0;
pub fn autonomy_main(arena: Arena) -> Behavior<LunabotAction> {
    While(
        Box::new(Action(LunabotAction::IsAutonomy)),
        vec![
            // 1. go to dig site
            Action(LunabotAction::SetBTStatusMsg(format!("Moving to dig site for {:?}", arena))),
            steer_for_n_seconds(Steering::new(1.0, 1.0, 1000.0), DIG_TRAVERSAL_TIME),
            // navigate_behavior(NavigationGoal::DigSite(arena)),
            Action(LunabotAction::SetBTStatusMsg(format!("DIGGING"))),
            open_loop_dig_from_starting(),
            // 2. dig
            // Action(LunabotAction::Dig),

            // 3. Navigate to dump site
            navigate_behavior(NavigationGoal::DumpSite(arena)),
            Action(LunabotAction::SetBTStatusMsg(format!("DUMPING"))),
            open_loop_dump_from_zero(),
            back_up_for(0.5),
            Wait(5.5),
            // 4. Dump
            // Action(LunabotAction::Dump),
        ],
    )
}



// This is terrible please don't follow in my footsteps (HB)
pub fn open_loop_dig_from_starting() -> Behavior<LunabotAction> {
    Sequence(
        vec![
            Wait(0.25), // Safety wait
            Action(LunabotAction::SetBucket((0.9 * i8::MAX as f64) as i8)),
            Wait(2.5),
            Action(LunabotAction::SetBucket(0)),
            Action(LunabotAction::SetLift((-0.75 * i8::MAX as f64) as i8)),
            Wait(0.8),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetLift((-0.1 * i8::MAX as f64) as i8)),
            Wait(3.0),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetLift((-0.2 * i8::MAX as f64) as i8)),
            Action(LunabotAction::SetBucket((0.2 * i8::MAX as f64) as i8)),
            While(
                Box::new(Wait(3.0)),
                vec![
                    Action(LunabotAction::SetSteering(Steering::new_ik(-0.25, 0.0, 8000.0))),
                    Wait(0.01)
                ]
            ),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
            Action(LunabotAction::SetSteering(Steering::new_ik(0.0, 0.0, 8000.0))),
            Action(LunabotAction::SetBucket((-0.1 * i8::MAX as f64) as i8)),
            While(
                Box::new(Wait(0.75)),
                vec![
                    Action(LunabotAction::SetSteering(Steering::new_ik(-0.25, 0.0, 8000.0))),
                    Wait(0.01)
                ]
            ),
            Action(LunabotAction::SetSteering(Steering::new_ik(0.0, 0.0, 8000.0))),
            Wait(0.75),
            Action(LunabotAction::SetBucket((-0.75 * i8::MAX as f64) as i8)),
            Wait(2.5),
            Action(LunabotAction::SetBucket(0)),
            Action(LunabotAction::SetLift((0.9 * i8::MAX as f64) as i8)),
            Action(LunabotAction::SetBucket((0.5 * i8::MAX as f64) as i8)),
            Wait(6.0),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
            Action(LunabotAction::SetLift((0.1 * i8::MAX as f64) as i8)),
            Action(LunabotAction::SetBucket((-0.8 * i8::MAX as f64) as i8)),
            Wait(3.0),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
            Wait(4.0), // wait for regolith to pour
            Action(LunabotAction::SetBucket((0.9 * i8::MAX as f64) as i8)),
            Wait(2.0),
            Action(LunabotAction::SetBucket(0)),
            Action(LunabotAction::SetLift((-0.35 * i8::MAX as f64) as i8)),
            Action(LunabotAction::SetBucket((0.9 * i8::MAX as f64) as i8)),
            Wait(2.0),
            Action(LunabotAction::SetLift(0)),
            Action(LunabotAction::SetBucket(0)),
        ]
    )
}



pub fn open_loop_dump_from_zero() -> Behavior<LunabotAction> {
    Sequence(
        vec![
            Wait(1.5),
            Action(LunabotAction::SetDumper((0.75 * i8::MAX as f64) as i8)),
            Wait(5.0),
            Action(LunabotAction::SetDumper(0)),
            Wait(10.0),
        ]
    )
}



// TODO move this to a common location and have open_loop_dump_from_zero call it.
// (I do think this is better as a action creator rather than a phase, given its 
// purpose, but do tell me if I'm wrong - HB)
fn shake_dump() -> Behavior<LunabotAction> {
    Sequence(
        vec![
            While(
                Box::new(Wait(3.0)),
                vec![
                    Action(LunabotAction::SetDumper((1.0 * i8::MAX as f64) as i8)),
                    Wait(0.1),
                    Action(LunabotAction::SetDumper((-1.0 * i8::MAX as f64) as i8)),
                    Wait(0.1),
                ]
            ),
            Action(LunabotAction::SetDumper(0)),
        ]
    )
}

// TODO move this to a common location and have open_loop_dig_from_starting call it.
// (I do think this is better as a action creator rather than a phase, given its 
// purpose, but do tell me if I'm wrong - HB)
fn shake_tilt() -> Behavior<LunabotAction> {
    Sequence(
        vec![
            While(
                Box::new(Wait(3.0)),
                vec![
                    Action(LunabotAction::SetBucket((1.0 * i8::MAX as f64) as i8)),
                    Wait(0.1),
                    Action(LunabotAction::SetBucket((-1.0 * i8::MAX as f64) as i8)),
                    Wait(0.1),
                ]
            ),
            Action(LunabotAction::SetBucket(0)),
        ]
    )
}