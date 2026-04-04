use crate::config::{GridLayout, get_config};

pub mod config;


fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 880.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Lunabot Video Feed",
        options,
        Box::new(|cc| {
            egui_extras::install_image_loaders(&cc.egui_ctx);
            Ok(Box::<CameraMultiplexerApp>::new(CameraMultiplexerApp::from_config()))
        }),
    )
}


pub struct CameraMultiplexerApp {
    config: GridLayout
}

impl CameraMultiplexerApp {
    /// panics if the config fails to parse
    fn from_config() -> Self {
        Self {
            config: get_config().expect("Failed to parse config")
        }
    }
}

impl eframe::App for CameraMultiplexerApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.label("hello")
        });
    }
}
