#![feature(let_chains, try_blocks, result_flattening)]
pub mod tasks;
pub mod rerun_viz;
pub mod utils;
pub mod comms;

use cu29::prelude::*;
use cu29_export::run_cli;
use simple_motion::StaticNode;
use std::sync::OnceLock;

pub static ROOT_NODE: OnceLock<StaticNode> = OnceLock::new();

// This will create the CuMsgs that is specific to your copper project.
// It is used to instruct the log reader how to decode the logs.
gen_cumsgs!("copperconfig.ron");

#[cfg(feature = "logreader")]
fn main() {
    gstreamer::init();
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
