use std::sync::Arc;

use crossbeam::atomic::AtomicCell;
use once_cell::sync::Lazy;

use crate::LunabotStage;

pub const THALASSIC_CELL_SIZE: f32 = 0.03125;
pub const THALASSIC_WIDTH: u32 = 160;
pub const THALASSIC_HEIGHT: u32 = 224;
pub const THALASSIC_CELL_COUNT: u32 = THALASSIC_WIDTH * THALASSIC_HEIGHT;

/// global shared Lunabot stage atomic cell to synchronize stage information across components.
/// every ping sent to the lunabase contains a byte saying which stage the lunabot is in so it knows the status of the lunabot at all times.
/// this static is set by the behavior tree
pub static LUNABOT_STAGE: Lazy<Arc<AtomicCell<LunabotStage>>> =
    Lazy::new(|| Arc::new(AtomicCell::new(LunabotStage::SoftStop)));
