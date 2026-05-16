use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize, Clone, Copy, Default, PartialEq)]
pub enum Rotation {
    #[default]
    None,
    Cw90,
    Ccw90,
    R180,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct CameraFeed {
    pub id: String,
    pub address: String,
    #[serde(default)]
    pub rotation: Rotation,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct GridLayout {
    pub dimensions: (usize, usize),
    pub feeds: Vec<CameraFeed>,
}

pub fn get_config() -> Result<GridLayout, Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "camera_layout.ron".to_string());
    let s = std::fs::read_to_string(&path)
        .map_err(|e| format!("Failed to read config at {path:?}: {e}"))?;
    let config: GridLayout = ron::from_str(&s)
        .map_err(|e| format!("Failed to parse config: {e}"))?;
    Ok(config)
}