#![feature(try_blocks, f16, mpmc_channel)]
pub mod comms;

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
mod motors;

pub mod rerun_viz;

pub mod bridges;
pub mod pathfinding;
pub mod robot_state;
pub mod simple_monitor;
pub mod tasks;
pub mod utils;
pub mod kalman_filtering;
pub mod payloads;
use crossbeam::atomic::AtomicCell;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use launcher::ProcessCommand;
use nalgebra::{SMatrix, SVector};
use simple_motion::{ChainBuilder, NodeSerde};
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use std::thread::sleep;
use std::time::Duration;

use robot_state::RobotState;

use crate::default::CuStampedDataSet;
extern crate cu_bincode as bincode;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROBOT_STATE: OnceLock<RobotState> = OnceLock::new();

#[copper_runtime(config = "copperconfig.ron")]
struct LunabotApplication {}

fn main() {
    let robot_chain = NodeSerde::from_reader(
        std::fs::File::open("../robot-layout/lunabot.ron").expect("Failed to read robot chain"),
    )
    .expect("Failed to parse robot chain");

    let robot_chain = ChainBuilder::from(robot_chain).finish_static();
    let _ = ROBOT_STATE.set(RobotState {
        kinematic_root: robot_chain,
        kalman_state: Arc::new(AtomicCell::new(Some(SVector::<f64, 15>::from_element(0.0)))),
        kalman_variances: Arc::new(AtomicCell::new(Some(
            SMatrix::<f64, 15, 15>::from_diagonal_element(1E64),
        ))),
    });

    std::thread::Builder::new()
        .stack_size(20 * 1000000)
    .spawn(move || { 
        cu29_export::run_cli::<CuStampedDataSet>().unwrap();
    }).expect("fuck").join();
}
