#![feature(try_blocks, f16, mpmc_channel)]
pub mod comms;
mod motors;
pub mod rerun_viz;
mod rp2040;
pub mod simple_monitor;
pub mod tasks;
pub mod utils;

use crossbeam_channel::{Receiver, Sender};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use embedded_common::{ActuatorCommand, FromPicoV3};
use launcher::ProcessCommand;
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::thread::sleep;
use std::time::Duration;

use crate::rp2040::enumerate_v3picos;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROOT_NODE: OnceLock<StaticNode> = OnceLock::new();

pub static PICO_TX: OnceLock<Sender<ActuatorCommand>> = OnceLock::new();
pub static PICO_RX: OnceLock<Receiver<FromPicoV3>> = OnceLock::new();

#[copper_runtime(config = "copperconfig.ron")]
struct LunabotApplication {}

fn main() {
    launch_subprocs();
    let logger_path = "logs/lunabot.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(&logger_path),
        PREALLOCATED_STORAGE_SIZE,
        true,
        None,
    )
    .expect("Failed to setup logger.");

    rerun_viz::init_rerun(rerun_viz::RerunViz::Grpc(
        rerun_viz::Level::Minimal,
        "192.168.0.100".to_string(),
    ))
    .expect("Failed to initialize rerun viz.");

    let (pico_tx, pico_rx) = enumerate_v3picos();

    PICO_RX.set(pico_rx).expect("Failed to set PICO_RX");
    PICO_TX.set(pico_tx).expect("Failed to set PICO_TX");

    let robot_chain = NodeSerde::from_reader(
        std::fs::File::open("../robot-layout/lunabot.ron").expect("Failed to read robot chain"),
    )
    .expect("Failed to parse robot chain");

    let robot_chain = ChainBuilder::from(robot_chain).finish_static();
    let _ = ROOT_NODE.set(robot_chain);

    let mut application = LunabotApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application.");
    let clock = copper_ctx.clock.clone();
    info!("Running... starting clock: {}.", clock.now());

    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}

fn launch_subprocs() {
    let mut launcher = launcher::ProcessLauncher::new();
    let suppress_output = false;
    let mut unilidar_cmd = ProcessCommand::new("./unilidar_publisher")
        .with_detach(true)
        .with_working_directory("../unilidar_iceoryx_publisher/bazel-bin/");
    if suppress_output {
        unilidar_cmd = unilidar_cmd.with_suppress_output(true);
    }
    // launcher.add_command("unilidar publisher", unilidar_cmd);

    let mut realsense_cmd = ProcessCommand::new("cargo")
        .with_args(vec!["run", "--release"])
        .with_working_directory("../external-tasks/realsense");
    if suppress_output {
        realsense_cmd = realsense_cmd.with_suppress_output(true);
    }
    launcher.add_command("realsense publisher", realsense_cmd);

    let mut ai_cmd = ProcessCommand::new("cargo")
        .with_args(vec!["run", "--release"])
        .with_working_directory("../external-tasks/lunabot-ai2");
    if suppress_output {
        ai_cmd = ai_cmd.with_suppress_output(true);
    }
    launcher.add_command("lunabot ai", ai_cmd);

    launcher.launch_all().expect("failed to launch commands");
    let pids = launcher.pids.clone();
    std::panic::set_hook(Box::new(move |info| {
        use std::process::Command;

        for pid in &pids {
            let _ = Command::new("kill").arg(pid.to_string()).output();
        }
        eprintln!("Panic: {}", info);
    }));
}
