use cu29::config::Value;
use rerun::{RecordingStream, RecordingStreamBuilder};
pub mod task_impl;
use serde::{Deserialize, Serialize};
pub use task_impl::*;

#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct RerunInput {
    pub is_static: bool,
    pub log_path: Box<String>,
    pub data: Box<Vec<u8>>,
}

pub struct RerunIpc {
    recorder: RecordingStream,
    options: RerunOptions
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
enum Level {
    /// all logs, including point clouds
    All,
    /// [Default] just localization, obstacle map, apriltags
    Minimal,
    None,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
enum LogDestination {
    Grpc {
        addr: String,
    },
    File {
        path: String,
    },
    /// spawn a rerun window
    Spawn
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RerunOptions {
    pub level: Level,
    pub log_destination: LogDestination,
}

impl RerunIpc {
    fn new(options: RerunOptions) -> Self {
        let rec = match &options.log_destination {
            LogDestination::Grpc { addr } => {
                RecordingStreamBuilder::new("lunabot").connect_grpc_opts(addr, rerun::default_flush_timeout())
            },
            LogDestination::File {path} => {
                RecordingStreamBuilder::new("lunabot").save(path)
            },
            LogDestination::Spawn => {
                RecordingStreamBuilder::new("lunabot").spawn()
            },
        }.expect("RERUN FAILED TO START");

        Self {
            options,
            recorder: rec,
        }
    }
}