use std::sync::Arc;

use crossbeam::atomic::AtomicCell;
use once_cell::sync::Lazy;

use crate::LunabotStage;

/// global shared Lunabot stage atomic cell to synchronize stage information across components.
/// every ping sent to the lunabase contains a byte saying which stage the lunabot is in so it knows the status of the lunabot at all times.
/// this static is set by the behavior tree
pub static LUNABOT_STAGE: Lazy<Arc<AtomicCell<LunabotStage>>> =
    Lazy::new(|| Arc::new(AtomicCell::new(LunabotStage::SoftStop)));

pub const COMMAND_STREAM_ID: u8 = 0;
pub const POSE_STREAM_ID: u8 = 1;
pub const ERROR_STREAM_ID: u8 = 2;
pub const BT_STATUS_STREAM_ID: u8 = 3;

