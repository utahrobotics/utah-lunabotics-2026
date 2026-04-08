#[cfg(feature="production")]
pub mod implementation {
    use crossbeam::queue::ArrayQueue;
use cu29::prelude::*;
use cu29::{
    cutask::{CuSinkTask, Freezable},
    input_msg,
};
use gstreamer::ElementFactory;
use gstreamer::{Pipeline, prelude::*};
use gstreamer_app::{AppSrc, AppSrcCallbacks};
use gstreamer_video::prelude::*;

use crate::pathfinding::OccupancyGrid;

pub struct ObstacleStreamer {
    pipeline: Pipeline,
    grid_queue: &'static ArrayQueue<OccupancyGrid>,
}

impl Freezable for ObstacleStreamer {}

impl CuSinkTask for ObstacleStreamer {
    type Input<'m> = input_msg!(OccupancyGrid);

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
            .and_then(|config| {
                config
                    .get("bitrate")
                    .expect("failed to deserialize bitrate")
            })
            .expect("Provide bitrate for obstacle streamer");

        let width: u32 = config
            .and_then(|config| config.get("width").expect("failed to deserialize width"))
            .expect("Provide width for obstacle streamer");

        let height: u32 = config
            .and_then(|config| config.get("height").expect("failed to deserialize height"))
            .expect("Provide height for obstacle streamer");

        let fps: i32 = config
            .and_then(|config| config.get("fps").expect("failed to deserialize fps"))
            .expect("Provide fps for obstacle streamer");

        let host: String = config
            .and_then(|config| config.get("host").expect("failed to deserialize host"))
            .expect("please provide host");

        let port: i32 = config
            .and_then(|config| config.get("port").expect("failed to deserialize port"))
            .expect("please provide port");

        let video_info =
            gstreamer_video::VideoInfo::builder(gstreamer_video::VideoFormat::Gray8, width, height)
                .fps(gstreamer::Fraction::new(fps, 1))
                .build()
                .map_err(|e| CuError::new_with_cause("Failed to create VideoInfo", e))?;

        let pipeline = gstreamer::Pipeline::default();
        let appsrc = gstreamer_app::AppSrc::builder()
            .caps(&video_info.to_caps().unwrap())
            .format(gstreamer::Format::Time)
            .is_live(true)
            .build();
        let videoconvert = ElementFactory::make("videoconvert")
            .build()
            .map_err(|e| CuError::from(e.message.to_string()))?;
        let encoder = ElementFactory::make("vaapih264enc")
            .property("bitrate", bitrate)
            .build()
            .map_err(|e| CuError::from(e.message.to_string()))?;
        let payload_enc = ElementFactory::make("rtph264pay")
            .build()
            .map_err(|e| CuError::from(e.message.to_string()))?;
        let udp_sink = ElementFactory::make("udpsink")
            .property("host", host)
            .property("port", port)
            .property("sync", false)
            .build()
            .map_err(|e| CuError::from(e.message.to_string()))?;
        pipeline
            .add_many([appsrc.upcast_ref(), &videoconvert, &encoder, &payload_enc, &udp_sink])
            .map_err(|e| CuError::from(e.message.to_string()))?;
        gstreamer::Element::link_many([appsrc.upcast_ref(), &videoconvert, &encoder, &payload_enc, &udp_sink])
            .map_err(|e| CuError::from(e.message.to_string()))?;

        let vid_w = width as usize;
        let vid_h = height as usize;
        let mut img: Vec<u8> = vec![0u8; vid_w * vid_h];
        let latest_grid_queue: &'static ArrayQueue<OccupancyGrid> =
            Box::leak(Box::new(ArrayQueue::new(20)));
        let mut frame_count: u64 = 0;

        appsrc.set_callbacks(
            AppSrcCallbacks::builder()
                .need_data(move |appsrc, _| {
                    while let Some(grid) = latest_grid_queue.pop() {
                        render_occupancy_grid(&mut img, vid_w, vid_h, &grid);
                    }

                    let mut buffer =
                        gstreamer::Buffer::with_size(video_info.size()).expect("failed to allocate buffer");
                    {
                        let buffer = buffer.get_mut().unwrap();
                        buffer.set_pts(frame_count * gstreamer::ClockTime::from_nseconds(1_000_000_000 / fps as u64));

                        let mut vframe =
                            gstreamer_video::VideoFrameRef::from_buffer_ref_writable(buffer, &video_info)
                                .expect("failed to map video frame");

                        let stride = vframe.plane_stride()[0] as usize;
                        let w = vframe.width() as usize;

                        for (y, line) in vframe
                            .plane_data_mut(0)
                            .unwrap()
                            .chunks_exact_mut(stride)
                            .enumerate()
                        {
                            let src_offset = y * w;
                            if src_offset + w <= img.len() {
                                line[..w].copy_from_slice(&img[src_offset..src_offset + w]);
                            }
                        }
                    }

                    frame_count += 1;

                    if appsrc.push_buffer(buffer).is_err() {
                        eprintln!("[ObstacleStreamer] Failed to push buffer to appsrc");
                    }
                })
                .build(),
        );

        Ok(Self {
            pipeline,
            grid_queue: latest_grid_queue,
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
        let Some(new_grid) = input.payload() else {
            return Ok(());
        };
        if self.grid_queue.push(new_grid.clone()).is_err() {
            let _ = self.grid_queue.pop();
            let _ = self.grid_queue.push(new_grid.clone());
        }
        Ok(())
    }

    fn stop(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
        let _ = self.pipeline.set_state(gstreamer::State::Null);
        Ok(())
    }
}

/// Renders the occupancy grid into the fixed-size video frame buffer.
/// Maps grid cells to video pixels using nearest-neighbor scaling.
fn render_occupancy_grid(img: &mut [u8], vid_w: usize, vid_h: usize, grid: &OccupancyGrid) {
    let grid_w = grid.cells_x();
    let grid_h = grid.cells_y();
    if grid_w == 0 || grid_h == 0 {
        return;
    }

    // Clear the image before rendering
    img.fill(0);

    for py in 0..vid_h {
        for px in 0..vid_w {
            // Map video pixel to grid cell (nearest-neighbor)
            let gx = px * grid_w / vid_w;
            let gy = py * grid_h / vid_h;
            let grid_idx = gx + gy * grid_w;

            if grid_idx >= grid.gradient_map.len() {
                continue;
            }

            let gradient = grid.gradient_map[grid_idx];
            if gradient == f32::MIN {
                continue;
            }

            let normalized = gradient.clamp(0.0, 1.0);
            let pixel = (normalized * 255.0) as u8;
            let img_idx = px + py * vid_w;
            img[img_idx] = pixel;
        }
    }
}

}


#[cfg(not(feature="production"))] 
pub mod implementation {
use cu29::prelude::*;

use crate::pathfinding::OccupancyGrid;
pub struct ObstacleStreamer {

}
impl Freezable for ObstacleStreamer{}

impl CuSinkTask for ObstacleStreamer {
    type Input<'m> = input_msg!(OccupancyGrid);
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

    fn process(
        &mut self,
        _clock: &RobotClock,
        _input: &Self::Input<'_>,
    ) -> CuResult<()> {
        Err(CuError::new_with_cause(
            "no frames received",
            std::io::Error::other("no frames received"),
        ))
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
}
