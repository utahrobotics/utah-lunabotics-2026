#![feature(let_chains)]
pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
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
