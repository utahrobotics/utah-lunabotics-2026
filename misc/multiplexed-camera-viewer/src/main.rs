use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use egui::{Color32, Key, TextureHandle, TextureOptions, Vec2};

use crate::config::{GridLayout, get_config};
use crate::video::{SharedFrame, spawn_receiver};

pub mod config;
pub mod video;

fn main() -> eframe::Result {
    let config = get_config().expect("Failed to parse config");
    let (rows, cols) = config.dimensions;

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([(cols as f32) * 340.0, (rows as f32) * 300.0]),
        vsync: false,
        renderer: eframe::Renderer::Glow,
        ..Default::default()
    };
    eframe::run_native(
        "Lunabot Video Feed",
        options,
        Box::new(|cc| {
            egui_extras::install_image_loaders(&cc.egui_ctx);
            Ok(Box::new(CameraMultiplexerApp::new(config, cc.egui_ctx.clone())))
        }),
    )
}

struct FeedState {
    enabled: bool,
    active: Arc<AtomicBool>,
    frame: SharedFrame,
    texture: Option<TextureHandle>,
}

pub struct CameraMultiplexerApp {
    config: GridLayout,
    feeds: Vec<FeedState>,
    ctx: egui::Context,
    last_frame_time: Instant,
    fps_reset_time: Instant,
    frame_count: u32,
    fps: f32,
}

impl CameraMultiplexerApp {
    fn new(config: GridLayout, ctx: egui::Context) -> Self {
        let feeds = config
            .feed_descriptors
            .iter()
            .map(|feed| {
                let frame: SharedFrame = Arc::new(Mutex::new(None));
                let active = Arc::new(AtomicBool::new(true));
                spawn_receiver(&feed.address, Arc::clone(&frame), Arc::clone(&active), ctx.clone());
                FeedState {
                    enabled: true,
                    active,
                    frame,
                    texture: None,
                }
            })
            .collect();

        Self {
            config,
            feeds,
            ctx,
            last_frame_time: Instant::now(),
            fps_reset_time: Instant::now(),
            frame_count: 0,
            fps: 0.0,
        }
    }

    fn toggle_feed(&mut self, i: usize) {
        let feed = &mut self.feeds[i];
        feed.enabled = !feed.enabled;
        if feed.enabled {
            feed.active = Arc::new(AtomicBool::new(true));
            feed.frame = Arc::new(Mutex::new(None));
            feed.texture = None;
            spawn_receiver(
                &self.config.feed_descriptors[i].address,
                Arc::clone(&feed.frame),
                Arc::clone(&feed.active),
                self.ctx.clone()
            );
        } else {
            feed.active.store(false, Ordering::Relaxed);
            feed.texture = None;
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
        let now = Instant::now();
        let ctx = ui.ctx().clone();

        // FPS counter
        self.frame_count += 1;
        let elapsed_sec = now.duration_since(self.fps_reset_time).as_secs_f32();
        if elapsed_sec >= 1.0 {
            self.fps = self.frame_count as f32 / elapsed_sec;
            self.frame_count = 0;
            self.fps_reset_time = now;
        }
        let mut toggles = Vec::new();
        for (i, key) in HOTKEYS.iter().enumerate() {
            if i >= self.feeds.len() {
                break;
            }
            if ctx.input(|input| input.key_pressed(*key)) {
                toggles.push(i);
            }
        }
        for i in toggles {
            self.toggle_feed(i);
        }

        for (i, feed) in self.feeds.iter_mut().enumerate() {
            if !feed.enabled {
                continue;
            }
            if let Ok(mut lock) = feed.frame.try_lock() {
                if let Some(image) = lock.take() {
                    match &mut feed.texture {
                        Some(tex) => {
                            tex.set(image, TextureOptions::NEAREST)
                        },
                        None => {
                            feed.texture = Some(ctx.load_texture(
                                format!("feed_{i}"),
                                image,
                                TextureOptions::NEAREST,
                            ));
                        }
                    }
                }
            }
        }

        let cols = self.config.dimensions.1;
        let available = ui.available_size();
        let cell_width = available.x / cols as f32 - 8.0;
        let cell_height = available.y / self.config.dimensions.0 as f32 - 8.0;

        let mut clicked = Vec::new();

        egui::Grid::new("camera_grid")
            .spacing([8.0, 8.0])
            .show(ui, |ui| {
                for (i, descriptor) in self.config.feed_descriptors.iter().enumerate() {
                    let feed = &self.feeds[i];
                    ui.vertical(|ui| {
                        let label = if feed.enabled {
                            format!("[{}] {} (ON)", i + 1, descriptor.id)
                        } else {
                            format!("[{}] {} (OFF)", i + 1, descriptor.id)
                        };

                        if ui.button(&label).clicked() {
                            clicked.push(i);
                        }

                        let video_size = Vec2::new(cell_width, cell_height - 30.0);

                        if feed.enabled {
                            if let Some(tex) = &feed.texture {
                                let tex_size = tex.size_vec2();
                                let scale = (video_size.x / tex_size.x)
                                    .min(video_size.y / tex_size.y);
                                let display_size = tex_size * scale;
                                ui.image(egui::load::SizedTexture::new(tex.id(), display_size));
                            } else {
                                let (rect, _) = ui.allocate_exact_size(
                                    video_size,
                                    egui::Sense::hover(),
                                );
                                ui.painter()
                                    .rect_filled(rect, 4.0, Color32::from_gray(40));
                                ui.painter().text(
                                    rect.center(),
                                    egui::Align2::CENTER_CENTER,
                                    format!("Waiting for feed...\n{}", descriptor.address),
                                    egui::FontId::proportional(14.0),
                                    Color32::from_gray(140),
                                );
                            }
                        } else {
                            let (rect, _) =
                                ui.allocate_exact_size(video_size, egui::Sense::hover());
                            ui.painter()
                                .rect_filled(rect, 4.0, Color32::from_gray(20));
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

        for i in clicked {
            self.toggle_feed(i);
        }
        self.last_frame_time = now;
    }
}
