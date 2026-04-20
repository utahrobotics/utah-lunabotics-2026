use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use egui::{Color32, Key, TextureHandle, TextureOptions, Vec2};
use egui_tiles::{Tile, TileId, Tiles};
use serde::{Deserialize, Serialize};

use crate::config::{get_config, GridLayout};
use crate::video::{SharedFrame, spawn_receiver};

pub mod config;
pub mod video;

fn tree_save_path() -> PathBuf {
    let mut path = std::env::current_exe().unwrap_or_else(|_| PathBuf::from("."));
    path.pop();
    path.push("camera_layout_state.ron");
    path
}

fn save_tree(tree: &egui_tiles::Tree<CameraPane>) {
    match ron::to_string(tree) {
        Ok(s) => {
            if let Err(e) = std::fs::write(tree_save_path(), s) {
                eprintln!("Failed to save tree: {e}");
            }
        }
        Err(e) => eprintln!("Failed to serialize tree: {e}"),
    }
}

fn load_tree(config: &GridLayout) -> egui_tiles::Tree<CameraPane> {
    let path = tree_save_path();
    if path.exists() {
        match std::fs::read_to_string(&path) {
            Ok(s) => match ron::from_str(&s) {
                Ok(tree) => {
                    eprintln!("Loaded tree from {}", path.display());
                    return tree;
                }
                Err(e) => eprintln!("Failed to deserialize tree, using default: {e}"),
            },
            Err(e) => eprintln!("Failed to read tree file, using default: {e}"),
        }
    }
    CameraMultiplexerApp::build_tree(config)
}

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
            Ok(Box::new(CameraMultiplexerApp::new(
                config,
                cc.egui_ctx.clone(),
            )))
        }),
    )
}

struct FeedState {
    enabled: bool,
    active: Arc<AtomicBool>,
    frame: SharedFrame,
    texture: Option<TextureHandle>,
    last_frame_time: Option<Instant>
}

#[derive(Deserialize, Serialize)]
struct CameraPane {
    feed_index: usize,
}

struct CameraBehavior<'a> {
    feeds: &'a mut Vec<FeedState>,
    config: &'a GridLayout,
    ctx: &'a egui::Context,
    clicked: Vec<usize>,
    cols: usize,
}

impl<'a> egui_tiles::Behavior<CameraPane> for CameraBehavior<'a> {
    fn pane_ui(
        &mut self,
        ui: &mut egui::Ui,
        _tile_id: TileId,
        pane: &mut CameraPane,
    ) -> egui_tiles::UiResponse {
        let i = pane.feed_index;
        let feed = &self.feeds[i];
        let descriptor = &self.config.feed_descriptors[i];

        let label = if feed.enabled {
            format!("[{}] {} (ON)", i + 1, descriptor.id)
        } else {
            format!("[{}] {} (OFF)", i + 1, descriptor.id)
        };

        let mut drag_started = false;

        let stale = feed.last_frame_time.map_or(false, |t| {
            t.elapsed() > std::time::Duration::from_millis(500)
        });

        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                let (drag_rect, drag_response) = ui.allocate_exact_size(
                    Vec2::new(20.0, 20.0),
                    egui::Sense::click_and_drag(),
                );
                ui.painter()
                    .rect_filled(drag_rect, 2.0, Color32::from_gray(80));
                ui.painter().text(
                    drag_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "::",
                    egui::FontId::proportional(12.0),
                    Color32::from_gray(180),
                );
                drag_response.clone().on_hover_cursor(egui::CursorIcon::Grab);
                if drag_response.dragged() {
                    drag_started = true;
                }

                if ui.button(&label).clicked() {
                    self.clicked.push(i);
                }

                if stale && feed.enabled {
                    ui.colored_label(Color32::RED, "No signal");
                }
            });

            let available = ui.available_size();
            let video_size = Vec2::new(available.x, (available.y - 30.0).max(0.0));

            if feed.enabled {
                if let Some(tex) = &feed.texture {
                    let tex_size = tex.size_vec2();
                    let scale =
                        (video_size.x / tex_size.x).min(video_size.y / tex_size.y);
                    let display_size = tex_size * scale;
                    ui.image(egui::load::SizedTexture::new(tex.id(), display_size));
                } else {
                    let (rect, _) =
                        ui.allocate_exact_size(video_size, egui::Sense::hover());
                    ui.painter().rect_filled(rect, 4.0, Color32::from_gray(40));
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

        if drag_started {
            egui_tiles::UiResponse::DragStarted
        } else {
            egui_tiles::UiResponse::None
        }
    }

    fn tab_title_for_pane(&mut self, pane: &CameraPane) -> egui::WidgetText {
        self.config.feed_descriptors[pane.feed_index]
            .id
            .clone()
            .into()
    }

    fn grid_auto_column_count(
        &self,
        _num_visible_children: usize,
        _rect: egui::Rect,
        _gap: f32,
    ) -> usize {
        self.cols
    }

    fn simplification_options(&self) -> egui_tiles::SimplificationOptions {
        egui_tiles::SimplificationOptions {
            prune_empty_tabs: true,
            prune_single_child_tabs: true,
            prune_empty_containers: true,
            prune_single_child_containers: true,
            all_panes_must_have_tabs: false,
            join_nested_linear_containers: true,
            ..Default::default()
        }
    }
}

pub struct CameraMultiplexerApp {
    config: GridLayout,
    feeds: Vec<FeedState>,
    ctx: egui::Context,
    tree: egui_tiles::Tree<CameraPane>,
    last_frame_time: Instant,
    fps_reset_time: Instant,
    frame_count: u32,
    fps: f32,
}

impl CameraMultiplexerApp {
    fn new(config: GridLayout, ctx: egui::Context) -> Self {
        let feeds: Vec<FeedState> = config
            .feed_descriptors
            .iter()
            .map(|feed| {
                let frame: SharedFrame = Arc::new(Mutex::new(None));
                let active = Arc::new(AtomicBool::new(true));
                spawn_receiver(
                    &feed.address,
                    Arc::clone(&frame),
                    Arc::clone(&active),
                    ctx.clone(),
                );
                FeedState {
                    enabled: true,
                    active,
                    frame,
                    texture: None,
                    last_frame_time: None,
                }
            })
            .collect();

        let tree = load_tree(&config);

        Self {
            config,
            feeds,
            ctx,
            tree,
            last_frame_time: Instant::now(),
            fps_reset_time: Instant::now(),
            frame_count: 0,
            fps: 0.0,
        }
    }

    fn build_tree(config: &GridLayout) -> egui_tiles::Tree<CameraPane> {
        let mut tiles = Tiles::default();
        let n = config.feed_descriptors.len();

        let pane_ids: Vec<TileId> = (0..n)
            .map(|i| tiles.insert_pane(CameraPane { feed_index: i }))
            .collect();

        let root = tiles.insert_grid_tile(pane_ids);

        if let Some(Tile::Container(egui_tiles::Container::Grid(grid))) =
            tiles.get_mut(root)
        {
            grid.layout = egui_tiles::GridLayout::Auto;
        }

        egui_tiles::Tree::new("camera_tree", root, tiles)
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
                self.ctx.clone(),
            );
        } else {
            feed.active.store(false, Ordering::Relaxed);
            feed.texture = None;
        }
    }

    fn enable_all(&mut self) {
        for i in 0..self.feeds.len() {
            if !self.feeds[i].enabled {
                self.feeds[i].enabled = true;
                self.feeds[i].active = Arc::new(AtomicBool::new(true));
                self.feeds[i].frame = Arc::new(Mutex::new(None));
                self.feeds[i].texture = None;
                spawn_receiver(
                    &self.config.feed_descriptors[i].address,
                    Arc::clone(&self.feeds[i].frame),
                    Arc::clone(&self.feeds[i].active),
                    self.ctx.clone(),
                );
            }
        }
    }

    fn disable_all(&mut self) {
        for feed in &mut self.feeds {
            if feed.enabled {
                feed.enabled = false;
                feed.active.store(false, Ordering::Relaxed);
                feed.texture = None;
            }
        }
    }

    fn all_enabled(&self) -> bool {
        self.feeds.iter().all(|f| f.enabled)
    }
}

const HOTKEYS: [Key; 9] = [
    Key::Num1, Key::Num2, Key::Num3,
    Key::Num4, Key::Num5, Key::Num6,
    Key::Num7, Key::Num8, Key::Num9,
];

impl eframe::App for CameraMultiplexerApp {
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
        save_tree(&self.tree);
    }

    fn auto_save_interval(&self) -> std::time::Duration {
        std::time::Duration::from_secs(5)
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        save_tree(&self.tree);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let ctx = ui.ctx().clone();

        let elapsed_sec = now.duration_since(self.fps_reset_time).as_secs_f32();
        if elapsed_sec >= 1.0 {
            self.fps = self.frame_count as f32 / elapsed_sec;
            self.frame_count = 0;
            self.fps_reset_time = now;
        }

        // Hotkeys
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

        // Update textures from incoming frames
        for (i, feed) in self.feeds.iter_mut().enumerate() {
            if !feed.enabled {
                continue;
            }
            if let Ok(mut lock) = feed.frame.try_lock() {
                if let Some(image) = lock.take() {
                    self.frame_count += 1; 
                    feed.last_frame_time = Some(Instant::now());
                    match &mut feed.texture {
                        Some(tex) => tex.set(image, TextureOptions::NEAREST),
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

        egui::TopBottomPanel::top("fps_bar").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("FPS: {:.1}", self.fps));

                ui.separator();

                let all_on = self.all_enabled();
                let btn_label = if all_on { "Disable All" } else { "Enable All" };
                let btn_color = if all_on {
                    Color32::from_rgb(180, 60, 60)
                } else {
                    Color32::from_rgb(60, 160, 80)
                };

                if ui
                    .add(egui::Button::new(btn_label).fill(btn_color))
                    .clicked()
                {
                    if all_on {
                        self.disable_all();
                    } else {
                        self.enable_all();
                    }
                }
            });
        });

        egui::CentralPanel::default().show_inside(ui, |ui| {
            let cols = self.config.dimensions.1;
            let mut behavior = CameraBehavior {
                feeds: &mut self.feeds,
                config: &self.config,
                ctx: &ctx,
                clicked: Vec::new(),
                cols,
            };

            self.tree.ui(&mut behavior, ui);

            let clicked = behavior.clicked.clone();
            for i in clicked {
                self.toggle_feed(i);
            }
        });

        self.last_frame_time = now;
    }
}