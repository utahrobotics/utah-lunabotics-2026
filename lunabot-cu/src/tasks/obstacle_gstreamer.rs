#[cfg(feature = "production")]
pub mod implementation {
    use std::sync::{Arc, RwLock};

    use cu29::prelude::*;
    use cu29::{
        cutask::{CuSinkTask, Freezable},
        input_msg,
    };
    use gstreamer::ElementFactory;
    use gstreamer::{Pipeline, prelude::*};
    use gstreamer_app::AppSrc;
    use gstreamer_video::prelude::*;

    use crate::pathfinding::OccupancyGrid;
    use crate::tasks::realsense_occupancy_grid::GLOBAL_MAP;
    use crate::utils::{rwlock_read_unpoison, rwlock_write_unpoison};

    pub struct ObstacleStreamer {
        pipeline: Pipeline,
        appsrc: AppSrc,
        video_info: gstreamer_video::VideoInfo,
        buffer_pool: gstreamer::BufferPool,
        frame_count: u64,
        fps: i32,
        img: Vec<u8>,
        vid_w: usize,
        vid_h: usize,
        latest_path: Arc<RwLock<Option<Vec<[f32; 2]>>>>,
    }

    impl Freezable for ObstacleStreamer {}

    impl CuSinkTask for ObstacleStreamer {
        type Input<'m> = input_msg!('m, OccupancyGrid, Vec<[f32; 2]>);
        type Resources<'r> = ();

        fn new(
            config: Option<&cu29::prelude::ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> cu29::CuResult<Self>
        where
            Self: Sized,
        {
            if !gstreamer::INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
                gstreamer::init()
                    .map_err(|e| CuError::new_with_cause("Failed to initialize gstreamer.", e))?;
            }

            let bitrate: u32 = config
                .and_then(|c| c.get("bitrate").expect("failed to deserialize bitrate"))
                .expect("Provide bitrate for ObstacleStreamer");
            let width: u32 = config
                .and_then(|c| c.get("width").expect("failed to deserialize width"))
                .expect("Provide width for ObstacleStreamer");
            let height: u32 = config
                .and_then(|c| c.get("height").expect("failed to deserialize height"))
                .expect("Provide height for ObstacleStreamer");
            let fps: i32 = config
                .and_then(|c| c.get("fps").expect("failed to deserialize fps"))
                .expect("Provide fps for ObstacleStreamer");
            let host: String = config
                .and_then(|c| c.get("host").expect("failed to deserialize host"))
                .expect("Provide host for ObstacleStreamer");
            let port: i32 = config
                .and_then(|c| c.get("port").expect("failed to deserialize port"))
                .expect("Provide port for ObstacleStreamer");

            let vid_w = width as usize;
            let vid_h = height as usize;

            let video_info = gstreamer_video::VideoInfo::builder(
                gstreamer_video::VideoFormat::Gray8,
                width,
                height,
            )
            .fps(gstreamer::Fraction::new(fps, 1))
            .build()
            .map_err(|e| CuError::new_with_cause("Failed to create VideoInfo", e))?;

            let buffer_pool = gstreamer::BufferPool::new();
            let mut pool_config = buffer_pool.config();
            pool_config.set_params(
                Some(&video_info.to_caps().unwrap()),
                video_info.size() as u32,
                8,
                8,
            );
            buffer_pool
                .set_config(pool_config)
                .map_err(|_| CuError::from("failed to configure buffer pool"))?;
            buffer_pool
                .set_active(true)
                .map_err(|_| CuError::from("failed to activate buffer pool"))?;

            let pipeline = gstreamer::Pipeline::default();
            let appsrc = gstreamer_app::AppSrc::builder()
                .caps(&video_info.to_caps().unwrap())
                .format(gstreamer::Format::Time)
                .is_live(true)
                .property_from_str("leaky-type", "downstream")
                .property("max-buffers", 2u64)
                .build();

            let videoconvert = ElementFactory::make("videoconvert")
                .build()
                .map_err(|e| CuError::from(e.message.to_string()))?;

            let encoder = ElementFactory::make("vaapih264enc")
                .build()
                .map_err(|e| CuError::from(e.message.to_string()))?;
            encoder.set_property("bitrate", bitrate);
            encoder.set_property_from_str("rate-control", "cbr");

            let muxer = ElementFactory::make("mpegtsmux")
                .build()
                .map_err(|e| CuError::from(e.message.to_string()))?;
            let tcp_sink = ElementFactory::make("tcpserversink")
                .property("host", host)
                .property("port", port)
                .property("sync", false)
                .build()
                .map_err(|e| CuError::from(e.message.to_string()))?;

            pipeline
                .add_many([
                    appsrc.upcast_ref(),
                    &videoconvert,
                    &encoder,
                    &muxer,
                    &tcp_sink,
                ])
                .map_err(|e| CuError::from(e.message.to_string()))?;
            gstreamer::Element::link_many([
                appsrc.upcast_ref(),
                &videoconvert,
                &encoder,
                &muxer,
                &tcp_sink,
            ])
            .map_err(|e| CuError::from(e.message.to_string()))?;

            Ok(Self {
                pipeline,
                appsrc,
                video_info,
                buffer_pool,
                frame_count: 0,
                fps,
                img: vec![0u8; vid_w * vid_h],
                vid_w,
                vid_h,
                latest_path: Arc::new(RwLock::new(Some(Vec::new()))),
            })
        }

        fn start(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
            self.pipeline
                .set_state(gstreamer::State::Playing)
                .map_err(|e| CuError::new_with_cause("Failed to set pipeline to Playing", e))?;

            let bus = self.pipeline.bus().expect("Pipeline has no bus");
            std::thread::spawn(move || {
                for msg in bus.iter_timed(gstreamer::ClockTime::NONE) {
                    match msg.view() {
                        gstreamer::MessageView::Error(err) => {
                            eprintln!(
                                "[ObstacleStreamer] GStreamer error from {:?}: {} (debug: {:?})",
                                err.src().map(|s| s.path_string()),
                                err.error(),
                                err.debug()
                            );
                            break;
                        }
                        gstreamer::MessageView::Warning(warn) => {
                            eprintln!(
                                "[ObstacleStreamer] GStreamer warning: {} (debug: {:?})",
                                warn.error(),
                                warn.debug()
                            );
                        }
                        gstreamer::MessageView::Eos(..) => {
                            eprintln!("[ObstacleStreamer] End of stream");
                            break;
                        }
                        _ => {}
                    }
                }
            });
            Ok(())
        }

        fn process<'i>(
            &mut self,
            _clock: &cu29::prelude::RobotClock,
            input: &Self::Input<'i>,
        ) -> cu29::CuResult<()> {
            *rwlock_write_unpoison(&self.latest_path) = input.1.payload().cloned();

            let Some(local_grid) = input.0.payload() else {
                return Ok(());
            };

            let global_guard = GLOBAL_MAP.get().and_then(|m| m.try_read().ok());
            let latest_path = rwlock_read_unpoison(&self.latest_path);
            render_occupancy_grid(
                &mut self.img,
                self.vid_w,
                self.vid_h,
                local_grid,
                global_guard.as_deref(),
                &*latest_path,
            );

            let mut buffer = self
                .buffer_pool
                .acquire_buffer(None)
                .map_err(|_| CuError::from("failed to acquire buffer from pool"))?;

            {
                let buffer = buffer.get_mut().unwrap();
                buffer.set_pts(
                    self.frame_count
                        * gstreamer::ClockTime::from_nseconds(1_000_000_000 / self.fps as u64),
                );

                let mut vframe = gstreamer_video::VideoFrameRef::from_buffer_ref_writable(
                    buffer,
                    &self.video_info,
                )
                .map_err(|_| CuError::from("failed to map video frame"))?;

                let stride = vframe.plane_stride()[0] as usize;
                let w = vframe.width() as usize;
                let dest = vframe.plane_data_mut(0).unwrap();

                for (y, line) in dest.chunks_exact_mut(stride).enumerate() {
                    let src_offset = y * w;
                    if src_offset + w <= self.img.len() {
                        line[..w].copy_from_slice(&self.img[src_offset..src_offset + w]);
                    }
                }
            }

            self.frame_count += 1;

            if self.appsrc.push_buffer(buffer).is_err() {
                eprintln!("[ObstacleStreamer] Failed to push buffer");
            }

            Ok(())
        }

        fn stop(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
            let _ = self.pipeline.set_state(gstreamer::State::Null);
            let _ = self.buffer_pool.set_active(false);
            Ok(())
        }
    }

    fn render_occupancy_grid(
        img: &mut [u8],
        vid_w: usize,
        vid_h: usize,
        local_grid: &OccupancyGrid,
        global_grid: Option<&OccupancyGrid>,
        latest_path: &Option<Vec<[f32; 2]>>,
    ) {
        let ref_grid = global_grid.unwrap_or(local_grid);
        let grid_w = ref_grid.cells_x();
        let grid_h = ref_grid.cells_y();
        if grid_w == 0 || grid_h == 0 {
            return;
        }

        img.fill(0);

        if let Some(global) = global_grid {
            for py in 0..vid_h {
                for px in 0..vid_w {
                    let gx = px * grid_w / vid_w;
                    let gy = py * grid_h / vid_h;
                    let grid_idx = gx + gy * grid_w;
                    if grid_idx >= global.gradient_map.len() {
                        continue;
                    }
                    let gradient = global.gradient_map[grid_idx].as_float();
                    if gradient == f32::MIN {
                        continue;
                    }
                    img[px + py * vid_w] = (gradient.clamp(0.0, 1.0) * 255.0) as u8;
                }
            }
        }

        let local_w = local_grid.cells_x();
        let local_h = local_grid.cells_y();
        for ly in 0..local_h {
            for lx in 0..local_w {
                let local_idx = lx + ly * local_w;
                if local_idx >= local_grid.gradient_map.len() {
                    continue;
                }
                let gradient = local_grid.gradient_map[local_idx].as_float();
                if gradient == f32::MIN {
                    continue;
                }
                let Ok((world_x, world_y)) = local_grid.cell_to_world(lx, ly) else {
                    continue;
                };
                let Ok((gx, gy)) = ref_grid.world_to_cell(world_x, world_y) else {
                    continue;
                };
                let px = gx * vid_w / grid_w;
                let py = gy * vid_h / grid_h;
                if px < vid_w && py < vid_h {
                    img[px + py * vid_w] = (gradient.clamp(0.0, 1.0) * 255.0) as u8;
                }
            }
        }

        let path_cells: Vec<(usize, usize)> = latest_path
            .iter()
            .flatten()
            .filter_map(|[x, y]| ref_grid.world_to_cell(*x, *y).ok())
            .collect();
        for (cx, cy) in path_cells {
            let px = cx * vid_w / grid_w;
            let py = cy * vid_h / grid_h;
            if px < vid_w && py < vid_h {
                img[px + py * vid_w] = 255;
            }
        }
    }
}

#[cfg(not(feature = "production"))]
pub mod implementation {
    use cu29::prelude::*;

    use crate::pathfinding::OccupancyGrid;
    pub struct ObstacleStreamer {}
    impl Freezable for ObstacleStreamer {}

    impl CuSinkTask for ObstacleStreamer {
        type Input<'m> = input_msg!('m, OccupancyGrid, Vec<[f32; 2]>);
        type Resources<'r> = ();

        fn new(_cfg: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
            Ok(Self {})
        }
        fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
            Ok(())
        }
        fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
            Ok(())
        }
        fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
        fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
            Ok(())
        }
    }
}