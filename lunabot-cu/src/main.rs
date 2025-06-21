#![feature(let_chains, try_blocks, f16)]
pub mod tasks;
pub mod rerun_viz;
pub mod utils;
pub mod common;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use simple_motion::{ChainBuilder, NodeSerde};
use std::thread::sleep;
use std::time::Duration;
use std::path::{Path,PathBuf};
use crate::tasks::*;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

#[copper_runtime(config = "copperconfig.ron")]
struct LunabotApplication {}

fn main() {
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
