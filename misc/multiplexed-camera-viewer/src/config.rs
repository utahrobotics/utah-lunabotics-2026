use serde::{Deserialize, Serialize};

static CONFIG_STR: &'static str = include_str!("../camera_layout.ron");

#[derive(Serialize, Deserialize, Debug)]
pub struct GridLayout {
    pub dimensions: (usize, usize),
    pub feed_descriptors: Vec<CameraFeed>,
}

#[derive(Serialize, Debug, Deserialize)]
pub struct CameraFeed {
    pub id: String,
    pub address: String,
}

pub fn get_config() -> Result<GridLayout, ron::Error> {
    Ok(ron::from_str(CONFIG_STR)?)
}

impl Default for GridLayout {
    fn default() -> Self {
        Self {
            dimensions: (2, 2),
            feed_descriptors: vec![],
        }
    }
}
