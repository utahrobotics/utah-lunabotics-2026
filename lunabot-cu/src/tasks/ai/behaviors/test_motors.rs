use bonsai_bt::Behavior::{self, Action, Sequence, While};
use common::Steering;

use crate::tasks::ai::action::LunabotAction;

pub fn test_motors_behavior() -> Behavior<LunabotAction> {
    Behavior::While(
        Box::new(Action(LunabotAction::IsTestMotors)),
        vec![
            Action(LunabotAction::CancelJobs),
            Action(LunabotAction::SetBTStatusMsg(String::from(
                "TEST LEFT SIDE FORWARD (software's definition of forward, \n the way the depth cam points)",
            ))),
            While(
                Box::new(Behavior::Wait(2.0)),
                vec![
                    Action(LunabotAction::SetSteering(Steering::new(1.0, 0.0, 1000.0))),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Behavior::Wait(1.0),
            Action(LunabotAction::SetBTStatusMsg(String::from(
                "TEST LEFT SIDE REVERSE (software's definition of reverse, \n the opposite way the depth cam points)",
            ))),
            While(
                Box::new(Behavior::Wait(2.0)),
                vec![
                    Action(LunabotAction::SetSteering(Steering::new(-1.0, 0.0, 1000.0))),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Behavior::Wait(1.0),
            Action(LunabotAction::SetBTStatusMsg(String::from(
                "TEST RIGHT SIDE FORWARD (software's definition of forward, \n the way the depth cam points)",
            ))),
            While(
                Box::new(Behavior::Wait(2.0)),
                vec![
                    Action(LunabotAction::SetSteering(Steering::new(0.0, 1.0, 1000.0))),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Behavior::Wait(1.0),
            Action(LunabotAction::SetBTStatusMsg(String::from(
                "TEST RIGHT SIDE REVERSE (software's definition of reverse, \n the opposite way the depth cam points)",
            ))),
            While(
                Box::new(Behavior::Wait(2.0)),
                vec![
                    Action(LunabotAction::SetSteering(Steering::new(0.0, -1.0, 1000.0))),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Behavior::Wait(2.0),
            Action(LunabotAction::SetBTStatusMsg(String::from("TEST LIFT"))),
            While(
                Box::new(Behavior::Wait(0.2)),
                vec![
                    Action(LunabotAction::SetLift(100)),
                    Action(LunabotAction::Yield),
                ],
            ),
            While(
                Box::new(Behavior::Wait(0.2)),
                vec![
                    Action(LunabotAction::SetLift(-100)),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Behavior::Wait(2.0),
            Action(LunabotAction::SetBTStatusMsg(String::from("TEST BUCKET"))),
            While(
                Box::new(Behavior::Wait(0.2)),
                vec![
                    Action(LunabotAction::SetBucket(100)),
                    Action(LunabotAction::Yield),
                ],
            ),
            While(
                Box::new(Behavior::Wait(0.2)),
                vec![
                    Action(LunabotAction::SetBucket(-100)),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Behavior::Wait(2.0),
            Action(LunabotAction::SetBTStatusMsg(String::from("TEST DUMPER"))),
            While(
                Box::new(Behavior::Wait(0.2)),
                vec![
                    Action(LunabotAction::SetDumper(100)),
                    Action(LunabotAction::Yield),
                ],
            ),
            While(
                Box::new(Behavior::Wait(0.2)),
                vec![
                    Action(LunabotAction::SetDumper(-100)),
                    Action(LunabotAction::Yield),
                ],
            ),
            set_zeros(),
            Action(LunabotAction::SetStage(common::LunabotStage::SoftStop)),
            Action(LunabotAction::Yield),
        ],
    )
}

fn set_zeros() -> Behavior<LunabotAction> {
    Sequence(vec![
        Action(LunabotAction::SetSteering(Steering::new(0.0, 0.0, 0.0))),
        Action(LunabotAction::SetLift(0)),
        Action(LunabotAction::SetBucket(0)),
        Action(LunabotAction::SetDumper(0)),
    ])
}
