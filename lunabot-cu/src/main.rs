#![feature(let_chains, try_blocks, f16, result_flattening)]
pub mod tasks;
pub mod rerun_viz;
pub mod utils;
pub mod comms;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use launcher::ProcessCommand;
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::sync::OnceLock;
use std::thread::sleep;
use std::time::Duration;
use std::path::{Path,PathBuf};
use crate::tasks::*;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROOT_NODE: OnceLock<StaticNode> = OnceLock::new();


#[copper_runtime(config = "copperconfig.ron")]
struct LunabotApplication {}

fn main() {
    let mut launcher = launcher::ProcessLauncher::new();
    let suppress_output = cfg!(not(debug_assertions));
    let mut unilidar_cmd = ProcessCommand::new("./unilidar_publisher")
        .with_detach(true)
        .with_working_directory("../unilidar_iceoryx_publisher/bazel-bin/");
    if suppress_output {
        unilidar_cmd = unilidar_cmd.with_suppress_output(true);
    }
    launcher.add_command("unilidar publisher", unilidar_cmd);

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
    let logger_path = "logs/lunabot.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }
    let copper_ctx =
        basic_copper_setup(&PathBuf::from(&logger_path), PREALLOCATED_STORAGE_SIZE, true, None).expect("Failed to setup logger.");

    info!("Logger created at {}.", logger_path);
    info!("Creating application... ");
    info!("Initializing rerun viz... ");
    rerun_viz::init_rerun(rerun_viz::RerunViz::Viz(rerun_viz::Level::All)).expect("Failed to initialize rerun viz.");

    let robot_chain = NodeSerde::from_reader(
        std::fs::File::open("../robot-layout/lunabot.ron").expect("Failed to read robot chain"),
    ).expect("Failed to parse robot chain");
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
