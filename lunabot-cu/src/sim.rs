#![feature(try_blocks, f16, mpmc_channel)]
pub mod comms;
pub mod rerun_viz;

pub mod simple_monitor;
pub mod tasks;
pub mod utils;

use crossbeam_channel::{Receiver, Sender};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use embedded_common::{ActuatorCommand, FromPicoV3};
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROOT_NODE: OnceLock<StaticNode> = OnceLock::new();

pub static TARGET_HZ: usize = 1000; // MUST BE THE SAME AS THE TARGET HZ IN COPPERCONFIG.RON

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct LunabotApplication {}

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        default::SimStep::UdevMonitor(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Pointcloud(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Imu(_) => SimOverride::ExecutedBySim,
        default::SimStep::V3Pico(_) => SimOverride::ExecutedBySim,
        default::SimStep::MotorCtrl(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::RealsenseSubscriber(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn sim_callback(step: default::SimStep) -> SimOverride {
    match step {
        default::SimStep::UdevMonitor(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Pointcloud(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Imu(_) => SimOverride::ExecutedBySim,
        default::SimStep::V3Pico(_) => SimOverride::ExecutedBySim,
        default::SimStep::MotorCtrl(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::RealsenseSubscriber(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn main() {
    std::thread::Builder::new()
        .stack_size(20 * 1000000) // this size will depend on how big the copper list is
        .spawn(move || {
            // Setup logging path for resimulation
            let logger_path = "logs/lunabotsim.copper";
            if let Some(parent) = Path::new(logger_path).parent() {
                if !parent.exists() {
                    std::fs::create_dir_all(parent).expect("Failed to create logs directory");
                }
            }
            // Create mock robot clock for simulation
            let (robot_clock, mut robot_clock_mock) = RobotClock::mock();

            let copper_ctx = basic_copper_setup(
                &PathBuf::from(&logger_path),
                PREALLOCATED_STORAGE_SIZE,
                true,
                Some(robot_clock.clone()),
            )
            .expect("Failed to setup logger.");

            rerun_viz::init_rerun(rerun_viz::RerunViz::Viz(rerun_viz::Level::All)).expect(
                "Failed to initialize Rerun. Please check that the rerun binary is in your path.",
            );

            let robot_chain = NodeSerde::from_reader(
                std::fs::File::open("../robot-layout/lunabot.ron")
                    .expect("Failed to read robot chain"),
            )
            .expect("Failed to parse robot chain");

            let robot_chain = ChainBuilder::from(robot_chain).finish_static();
            let _ = ROOT_NODE.set(robot_chain);

            let mut application = LunabotApplicationBuilder::new()
                .with_sim_callback(&mut default_callback)
                .with_context(&copper_ctx)
                .build()
                .expect("Failed to create application.");

            application
                .start_all_tasks(&mut default_callback)
                .expect("Failed to start all tasks.");
            loop {
                application.run_one_iteration(&mut sim_callback);
            }
        })
        .expect("failed to spawn main thread")
        .join()
        .unwrap_or_else(|e| {
            if let Some(panic_msg) = e.downcast_ref::<&str>() {
                panic!("Thread panicked with message: {}", panic_msg);
            } else if let Some(panic_msg) = e.downcast_ref::<String>() {
                panic!("Thread panicked with message: {}", panic_msg);
            } else {
                panic!("Thread panicked with unknown error: {:?}", e);
            }
        });

    debug!("End of log replay.");
}
