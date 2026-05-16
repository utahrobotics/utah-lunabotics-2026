#![allow(unused, dead_code)]
/// these are modeled after some of nav2's nodes.
use bonsai_bt::Behavior::{self, Action, Invert, Race, Select, Sequence, Wait, While};
use common::Steering;

use crate::tasks::ai::action::LunabotAction;

/// tries to run the retry_me behavior retries times, and stops on the first success
pub fn recovery_node(retries: usize, retry_me: Behavior<LunabotAction>) -> Behavior<LunabotAction> {
    let retry_sequence: Vec<Behavior<LunabotAction>> = (0..retries)
        .map(|_| retry_me.clone())
        .collect::<Vec<Behavior<LunabotAction>>>();
    Select(retry_sequence)
}

pub fn with_timeout<A: Clone>(behavior: Behavior<A>, timeout: f64) -> Behavior<A> {
    Race(vec![behavior, Invert(Box::new(Wait(timeout)))])
}

pub fn steer_for_n_seconds(steering: Steering, n: f64) -> Behavior<LunabotAction> {
    Sequence(vec![
        While(
            Box::new(Wait(n)),
            vec![Action(LunabotAction::SetSteering(steering)), Wait(0.05)],
        ),
        Action(LunabotAction::SetSteering(Steering::new(0.0, 0.0, 1000.0))),
    ])
}
