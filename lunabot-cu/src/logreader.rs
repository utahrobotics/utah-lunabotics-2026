#![feature(let_chains, try_blocks, result_flattening, mpmc_channel)]
pub mod comms;
mod motors;
pub mod rerun_viz;
pub mod simple_monitor;
pub mod tasks;
pub mod utils;

use crossbeam_channel::{Receiver, Sender};
use cu29::prelude::*;
use cu29_export::run_cli;
use cu29_helpers::basic_copper_setup;
use embedded_common::{ActuatorCommand, FromPicoV3};
use launcher::ProcessCommand;
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::thread::sleep;
use std::time::Duration;

pub static ROOT_NODE: OnceLock<StaticNode> = OnceLock::new();
const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

// This will create the CuMsgs that is specific to your copper project.
// It is used to instruct the log reader how to decode the logs.
gen_cumsgs!("copperconfig.ron");

#[cfg(feature = "logreader")]
fn main() {
    gstreamer::init();
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
