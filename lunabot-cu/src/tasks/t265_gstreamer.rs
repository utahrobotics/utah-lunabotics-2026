#[cfg(feature = "production")]
pub mod implementation {
    use std::ops::Deref;

    use cu_sensor_payloads::CuImage;
    use cu29::prelude::*;
    use cu29::{
        cutask::{CuSinkTask, Freezable},
        input_msg,
    };
    use gstreamer::ElementFactory;
    use gstreamer::{Pipeline, prelude::*};
    use gstreamer_app::AppSrc;
    use gstreamer_video::prelude::*;

    pub struct T265Streamer {
        pipeline: Pipeline,
        appsrc: AppSrc,
        video_info: gstreamer_video::VideoInfo,
        buffer_pool: gstreamer::BufferPool,
        frame_count: u64,
        fps: i32,
    }

    impl Freezable for T265Streamer {}

    impl CuSinkTask for T265Streamer {
        type Input<'m> = input_msg!('m, CuImage<Vec<u8>>);
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
                .expect("Provide bitrate for T265Streamer");
            let width: u32 = config
                .and_then(|c| c.get("width").expect("failed to deserialize width"))
                .expect("Provide width for T265Streamer");
            let height: u32 = config
                .and_then(|c| c.get("height").expect("failed to deserialize height"))
                .expect("Provide height for T265Streamer");
            let fps: i32 = config
                .and_then(|c| c.get("fps").expect("failed to deserialize fps"))
                .expect("Provide fps for T265Streamer");
            let host: String = config
                .and_then(|c| c.get("host").expect("failed to deserialize host"))
                .expect("Provide host for T265Streamer");
            let port: i32 = config
                .and_then(|c| c.get("port").expect("failed to deserialize port"))
                .expect("Provide port for T265Streamer");

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
                8, // min buffers
                8, // max buffers
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
                .property("bitrate", bitrate)
                .build()
                .map_err(|e| CuError::from(e.message.to_string()))?;
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
                                "[T265Streamer] GStreamer error from {:?}: {} (debug: {:?})",
                                err.src().map(|s| s.path_string()),
                                err.error(),
                                err.debug()
                            );
                            break;
                        }
                        gstreamer::MessageView::Warning(warn) => {
                            eprintln!(
                                "[T265Streamer] GStreamer warning: {} (debug: {:?})",
                                warn.error(),
                                warn.debug()
                            );
                        }
                        gstreamer::MessageView::Eos(..) => {
                            eprintln!("[T265Streamer] End of stream");
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
            let Some(img) = input.payload() else {
                return Ok(());
            };

            let src_stride = img.format.stride as usize;

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

                let dest_stride = vframe.plane_stride()[0] as usize;
                let width = vframe.width() as usize;
                let height = vframe.height() as usize;
                let dest = vframe.plane_data_mut(0).unwrap();

                img.buffer_handle.with_inner(|buf: &CuHandleInner<Vec<u8>>| {
                    let src: &[u8] = buf.deref();
                    if src_stride == dest_stride {
                        dest[..height * dest_stride]
                            .copy_from_slice(&src[..height * src_stride]);
                    } else {
                        for row in 0..height {
                            dest[row * dest_stride..row * dest_stride + width]
                                .copy_from_slice(&src[row * src_stride..row * src_stride + width]);
                        }
                    }
                });
            }

            self.frame_count += 1;

            if self.appsrc.push_buffer(buffer).is_err() {
                eprintln!("[T265Streamer] Failed to push buffer");
            }

            Ok(())
        }

        fn stop(&mut self, _clock: &cu29::prelude::RobotClock) -> cu29::CuResult<()> {
            let _ = self.pipeline.set_state(gstreamer::State::Null);
            let _ = self.buffer_pool.set_active(false);
            Ok(())
        }
    }
}

#[cfg(not(feature = "production"))]
pub mod implementation {
    use cu_sensor_payloads::CuImage;
    use cu29::prelude::*;

    pub struct T265Streamer {}
    impl Freezable for T265Streamer {}

    impl CuSinkTask for T265Streamer {
        type Input<'m> = input_msg!('m, CuImage<Vec<u8>>);
        type Resources<'r> = ();

        fn new(_cfg: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
            Ok(Self {})
        }
        fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
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