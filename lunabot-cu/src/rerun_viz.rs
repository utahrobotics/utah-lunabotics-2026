use std::{sync::OnceLock, time::Instant};

use crossbeam::atomic::AtomicCell;
use cu29::CuError;
use rerun::{
    Asset3D, RecordingStream, RecordingStreamResult, SpawnOptions, Transform3D, ViewCoordinates,
};
use serde::Deserialize;

pub const ROBOT: &str = "/robot";
pub const ROBOT_STRUCTURE: &str = "/robot/structure";

pub static RECORDER: OnceLock<RecorderData> = OnceLock::new();

pub struct RecorderData {
    pub recorder: RecordingStream,
    pub level: Level,
    pub last_logged_obstacle_map: AtomicCell<Instant>, // used to throttle the logging to conserve bandwidth
}

#[derive(Deserialize, Default, Debug)]
pub enum RerunViz {
    Grpc(Level, String),
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
        RerunViz::Viz(level) => (
            match rerun::RecordingStreamBuilder::new("lunabot").spawn_opts(&opts) {
                Ok(rec) => rec,
                Err(e) => {
                    return Err(CuError::new_with_cause(
                        "Failed to make recording stream",
                        e,
                    ));
                }
            },
            level,
        ),
        RerunViz::Grpc(level, ip) => (
            match rerun::RecordingStreamBuilder::new("lunabot")
                .connect_grpc_opts(&format!("rerun+http://{ip}:9876/proxy"))
            {
                Ok(x) => x,
                Err(e) => {
                    return Err(CuError::new_with_cause(
                        "Failed to make recording stream",
                        e,
                    ));
                }
            },
            level,
        ),
        RerunViz::Log(level) => (
            match rerun::RecordingStreamBuilder::new("lunabot").save("recording.rrd") {
                Ok(x) => x,
                Err(e) => {
                    return Err(CuError::new_with_cause(
                        "Failed to start rerun file logging",
                        e,
                    ));
                }
            },
            level,
        ),
        RerunViz::Disabled => {
            return Err(CuError::new_with_cause(
                "Rerun visualization disabled",
                std::io::Error::new(std::io::ErrorKind::Other, "Rerun visualization disabled"),
            ));
        }
    };
    let result: RecordingStreamResult<()> = try {
        recorder.log_static("/", &ViewCoordinates::RIGHT_HAND_Z_UP())?;
        recorder.log_static(
            format!("{ROBOT_STRUCTURE}/xyz"),
            &rerun::Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                .with_labels(vec!["x", "y", "z"]),
        )?;
    };
    if let Err(e) = result {
        return Err(CuError::new_with_cause(
            "Failed to setup rerun environment",
            e,
        ));
    }

    let _ = RECORDER.set(RecorderData {
        recorder,
        level,
        last_logged_obstacle_map: AtomicCell::new(Instant::now()),
    });

    std::thread::spawn(|| {
        let recorder = &RECORDER.get().unwrap().recorder;

        let asset = match Asset3D::from_file_path("3d-models/simplify_lunabot.stl") {
            Ok(x) => x,
            Err(e) => {
                return Err(CuError::new_with_cause(
                    "Failed to open 3d-models/simplify_lunabot.stl",
                    e,
                ));
            }
        };

        if let Err(e) = recorder.log_static(format!("{ROBOT_STRUCTURE}/mesh"), &asset) {
            return Err(CuError::new_with_cause(
                "Failed to log robot structure mesh",
                e,
            ));
        }
        if let Err(e) = recorder.log_static(
            format!("{ROBOT_STRUCTURE}/mesh"),
            &Transform3D::from_translation([-0.50, 0.20, 0.0]),
        ) {
            return Err(CuError::new_with_cause(
                "Failed to log robot structure mesh",
                e,
            ));
        }

        Ok(())
    });
    Ok(())
}
