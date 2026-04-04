use egui::{Color32, Key, Vec2};

use crate::config::{GridLayout, get_config};

pub mod config;

fn main() -> eframe::Result {
    let config = get_config().expect("Failed to parse config");
    let (rows, cols) = config.dimensions;

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([(cols as f32) * 340.0, (rows as f32) * 300.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Lunabot Video Feed",
        options,
        Box::new(|cc| {
            egui_extras::install_image_loaders(&cc.egui_ctx);
            Ok(Box::new(CameraMultiplexerApp::new(config)))
        }),
    )
}

pub struct CameraMultiplexerApp {
    config: GridLayout,
    enabled: Vec<bool>,
}

impl CameraMultiplexerApp {
    fn new(config: GridLayout) -> Self {
        let count = config.feed_descriptors.len();
        Self {
            config,
            enabled: vec![true; count],
        }
    }
}

const HOTKEYS: [Key; 9] = [
    Key::Num1, Key::Num2, Key::Num3,
    Key::Num4, Key::Num5, Key::Num6,
    Key::Num7, Key::Num8, Key::Num9,
];

impl eframe::App for CameraMultiplexerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        for (i, key) in HOTKEYS.iter().enumerate() {
            if i >= self.enabled.len() {
                break;
            }
            if ctx.input(|input| input.key_pressed(*key)) {
                self.enabled[i] = !self.enabled[i];
            }
        }

        let cols = self.config.dimensions.1;
        let available = ui.available_size();
        let cell_width = available.x / cols as f32 - 8.0;
        let cell_height = available.y / self.config.dimensions.0 as f32 - 8.0;

        egui::Grid::new("camera_grid")
            .spacing([8.0, 8.0])
            .show(ui, |ui| {
                for (i, feed) in self.config.feed_descriptors.iter().enumerate() {
                    ui.vertical(|ui| {
                        let label = if self.enabled[i] {
                            format!("[{}] {} (ON)", i + 1, feed.id)
                        } else {
                            format!("[{}] {} (OFF)", i + 1, feed.id)
                        };

                        if ui.button(&label).clicked() {
                            self.enabled[i] = !self.enabled[i];
                        }

                        let (rect, _) = ui.allocate_exact_size(
                            Vec2::new(cell_width, cell_height - 30.0),
                            egui::Sense::hover(),
                        );

                        if self.enabled[i] {
                            ui.painter().rect_filled(rect, 4.0, Color32::from_gray(40));
                            ui.painter().text(
                                rect.center(),
                                egui::Align2::CENTER_CENTER,
                                format!("{}\n{}", feed.id, feed.address),
                                egui::FontId::proportional(14.0),
                                Color32::from_gray(140),
                            );
                        } else {
                            ui.painter().rect_filled(rect, 4.0, Color32::from_gray(20));
                            ui.painter().text(
                                rect.center(),
                                egui::Align2::CENTER_CENTER,
                                "Disabled",
                                egui::FontId::proportional(14.0),
                                Color32::from_gray(80),
                            );
                        }
                    });

                    if (i + 1) % cols == 0 {
                        ui.end_row();
                    }
                }
            });
    }
}
