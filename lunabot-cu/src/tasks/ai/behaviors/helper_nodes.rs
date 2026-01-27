#![allow(unused, dead_code)]
/// these are modeled after some of nav2's nodes.

use bonsai_bt::Behavior::{self, Select};

use crate::tasks::ai::action::LunabotAction;

/// tries to run the retry_me behavior retries times, and stops on the first success
pub fn recovery_node(retries: usize, retry_me: Behavior<LunabotAction>) -> Behavior<LunabotAction> {
    let retry_sequence = (0..retries).map(|_| {
        retry_me.clone()
    }).collect::<Vec<Behavior<LunabotAction>>>();
    Select(retry_sequence)
}