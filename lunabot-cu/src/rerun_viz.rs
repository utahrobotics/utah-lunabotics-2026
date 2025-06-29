use std::{f32::consts::PI, sync::{OnceLock, RwLock}, time::Instant};
use std::process::{Command, Stdio};
use std::net::TcpStream;
use std::time::Duration;

use crossbeam::atomic::AtomicCell;
use cu29::CuError;
use nalgebra::{UnitQuaternion, Vector3};
use rerun::{Asset3D, RecordingStream, RecordingStreamResult, SpawnOptions, ViewCoordinates};
use serde::Deserialize;

pub const ROBOT: &str = "/robot";
pub const ROBOT_STRUCTURE: &str = "/robot/structure";

pub static RECORDER: OnceLock<RecorderData> = OnceLock::new();

pub struct RecorderData {
    pub recorder: RecordingStream,
    pub level: Level,
    pub last_logged_obstacle_map: AtomicCell<Instant> // used to throttle the logging to conserve bandwidth
}

#[derive(Deserialize, Default, Debug)]
pub enum RerunViz {
    Grpc(Level,String),
    Log(Level),
    Viz(Level),
    #[default]
    Disabled,
}

#[derive(Deserialize, Default, Debug, PartialEq)]
pub enum Level {
    /// Only logs robots isometry, expanded obstacle map, and april tags.
    #[default]
    Minimal,
    /// Logs everything including height maps and depth camera point cloud.
    All,
}

impl Level {
    /// returns true if the log level is All
    pub fn is_all(&self) -> bool {
        *self == Level::All
    }
}

pub fn init_rerun(rerun_viz: RerunViz) -> Result<(), CuError> {
    let opts = SpawnOptions {
        memory_limit: "25%".to_string(),
        ..Default::default()
    };
    let (recorder, level) = match rerun_viz {
        RerunViz::Viz(level) => {
            // First try to spawn a Rerun viewer process, but with stdout/stderr redirected to
            // `/dev/null` so that its banner and logs do not pollute our own stdout.
            // If the viewer is already running, spawning will fail (port in use) – that is
            // fine, we will simply connect to it.

            let port = opts.port;

            // Only spawn if no process is currently listening on the port.
            let viewer_running = TcpStream::connect_timeout(&opts.connect_addr(), Duration::from_millis(200)).is_ok();
            if !viewer_running {
                let _ = Command::new(&opts.executable_path())
                    .arg(format!("--port={port}"))
                    .arg(format!("--memory-limit={}", opts.memory_limit))
                    .arg("--expect-data-soon")
                    .arg("--hide-welcome-screen")
                    .stdin(Stdio::null())
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .spawn();
            }

            let url = format!("rerun+http://127.0.0.1:{port}/proxy");
            (
                match rerun::RecordingStreamBuilder::new("lunabot").connect_grpc_opts(&url, None) {
                    Ok(x) => x,
                    Err(e) => {
                        return Err(CuError::new_with_cause("Failed to connect to rerun viewer", e));
                    }
                },
                level,
            )
        },
        RerunViz::Grpc(level, url) => (
            match rerun::RecordingStreamBuilder::new("lunabot").connect_grpc_opts(&url, None) {
                Ok(x) => {
                    x
                },
                Err(e) => {
                    return Err(CuError::new_with_cause("Failed to make recording stream", e));
                }
            },
            level,
        ),
        RerunViz::Log(level) => (
            match rerun::RecordingStreamBuilder::new("lunabot").save("recording.rrd") {
                Ok(x) => x,
                Err(e) => {
                    return Err(CuError::new_with_cause("Failed to start rerun file logging", e));
                }
            },
            level,
        ),
        RerunViz::Disabled => {
            return Err(CuError::new_with_cause("Rerun visualization disabled", std::io::Error::new(std::io::ErrorKind::Other, "Rerun visualization disabled")));
        }
    };
    let result: RecordingStreamResult<()> = try {
        recorder.log_static("/", &ViewCoordinates::RIGHT_HAND_Z_UP())?;
        recorder.log_static(
            format!("{ROBOT_STRUCTURE}/xyz"),
            &rerun::Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]]),
        )?;
    };
    if let Err(e) = result {
        return Err(CuError::new_with_cause("Failed to setup rerun environment", e));
    }

    let _ = RECORDER.set(RecorderData { recorder, level, last_logged_obstacle_map: AtomicCell::new(Instant::now())});

    std::thread::spawn(|| {
        let recorder = &RECORDER.get().unwrap().recorder;

        let asset = match Asset3D::from_file_path("3d-models/simplify_lunabot.stl") {
            Ok(x) => x,
            Err(e) => {
                return Err(CuError::new_with_cause("Failed to open 3d-models/simplify_lunabot.stl", e));
            }
        };

        if let Err(e) = recorder.log_static(format!("{ROBOT_STRUCTURE}/mesh"), &asset) {
            return Err(CuError::new_with_cause("Failed to log robot structure mesh", e));
        }

        Ok(())
    });
    Ok(())
}