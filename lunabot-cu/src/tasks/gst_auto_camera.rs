// GstAutoCam: dynamically opens a GStreamer pipeline when the desired camera gets plugged in.
// For non-Linux platforms we supply an empty stub so the crate builds on macOS/Windows.

#[cfg(target_os = "linux")]
mod linux_impl {
    use cu29::prelude::*;
    use cu_gstreamer::{CuGstBuffer, CuGStreamer};
    use std::time::{Duration, Instant};
    use crate::tasks::udev_monitor::NewDevice as DevMsg;
    use bincode::{Encode, Decode};

    #[derive(Default, Debug, Clone, Encode, Decode)]
    pub struct CuGstBuffer(pub Vec<u8>);

    pub struct GstAutoCam {
        gst: Option<CuGStreamer<8>>, // internally we use a fixed pool size of 8 frames
        desired_port: String,
        pipeline_template: String,
        caps: String,
        timeout: Duration,
        last_frame_time: Option<Instant>,
        camera_id: Box<String>,
    }

    impl Freezable for GstAutoCam {}

    impl<'cl> CuTask<'cl> for GstAutoCam {
        type Output = output_msg!('cl, CuGstBuffer);
        type Input = input_msg!('cl, DevMsg);

        fn new(config: Option<&ComponentConfig>) -> CuResult<Self> where Self: Sized {
            let cfg = config.ok_or::<CuError>("No config provided".into())?;

            let desired_port = cfg.get::<String>("device_port").unwrap_or_default();
            let pipeline_template = cfg.get::<String>("pipeline").unwrap_or_else(|| "v4l2src device={device} ! videoconvert ! appsink name=copper".to_string());
            let caps = cfg.get::<String>("caps").unwrap_or_else(|| "video/x-raw, format=GRAY8".to_string());
            let timeout_ms = cfg.get::<u32>("timeout_ms").unwrap_or(500);
            let camera_id = cfg.get::<String>("camera_id").unwrap_or_default();
            Ok(Self {
                gst: None,
                desired_port,
                pipeline_template,
                caps,
                timeout: Duration::from_millis(timeout_ms as u64),
                last_frame_time: None,
                camera_id: Box::new(camera_id),
            })
        }

        fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
            Ok(())
        }

        fn process(&mut self, clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()> {
            // Open pipeline when device detected
            if self.gst.is_none() {
                if let Some(dev) = input.payload() {
                    if dev.port.as_str() == self.desired_port {
                        self.open_pipeline(&dev.dev_path, clock)?;
                    }
                }
            }

            if let Some(gst) = self.gst.as_mut() {
                // Delegate frame extraction to the inner CuGStreamer instance
                gst.process(clock, output)?;
                if output.payload().is_some() {
                    self.last_frame_time = Some(Instant::now());
                }

                // Handle timeout / restart logic
                if let Some(last) = self.last_frame_time {
                    if last.elapsed() > self.timeout {
                        debug!("GstAutoCam: No frames for {:?}, restarting pipeline", self.timeout);
                        gst.stop(clock)?;
                        self.gst = None;
                        self.last_frame_time = None;
                        return Err("No frames received in the last timeout window".into());
                    }
                } else {
                    return Err("No frames received yet".into());
                }
            } else {
                // we are still waiting for the camera
                output.clear_payload();
                return Err("Camera not ready".into());
            }
            Ok(())
        }

        fn stop(&mut self, clock: &RobotClock) -> CuResult<()> {
            if let Some(gst) = self.gst.as_mut() {
                gst.stop(clock)?;
            }
            Ok(())
        }
    }

    impl GstAutoCam {
        fn open_pipeline(&mut self, dev_path: &str, clock: &RobotClock) -> CuResult<()> {
            if self.gst.is_some() {
                return Ok(());
            }
            let pipeline_str = if self.pipeline_template.contains("{device}") {
                self.pipeline_template.replace("{device}", dev_path)
            } else {
                format!("v4l2src device={} ! {}", dev_path, self.pipeline_template)
            };
            debug!("GstAutoCam: Using pipeline: {}", pipeline_str);
            let mut gst = CuGStreamer::<8>::create_with_pipeline_and_caps(&pipeline_str, &self.caps, Some(self.timeout))?;
            gst.start(clock)?;
            self.gst = Some(gst);
            Ok(())
        }
    }
}

// Non-Linux stub so the crate builds.
#[cfg(not(target_os = "linux"))]
mod empty_impl {
    use cu29::prelude::*;
    use crate::tasks::udev_monitor::NewDevice;

    // Minimal stand-in so code compiles on non-Linux targets without pulling in GStreamer
    #[derive(Default, Debug, Clone)]
    pub struct CuGstBuffer(Vec<u8>);

    #[derive(Default)]
    pub struct GstAutoCam;

    impl Freezable for GstAutoCam {}

    impl<'cl> CuTask<'cl> for GstAutoCam {
        type Output = output_msg!('cl, CuGstBuffer);
        type Input = input_msg!('cl, NewDevice);
        fn new(_: Option<&ComponentConfig>) -> CuResult<Self> { Ok(Self) }
        fn start(&mut self, _: &RobotClock) -> CuResult<()> { Ok(()) }
        fn process(&mut self, _: &RobotClock, _: Self::Input, _out: Self::Output) -> CuResult<()> { Ok(()) }
        fn stop(&mut self, _: &RobotClock) -> CuResult<()> { Ok(()) }
    }
}

// Re-export the platform-specific implementation so callers can use crate::tasks::GstAutoCam
#[cfg(target_os = "linux")]
pub use linux_impl::GstAutoCam;
#[cfg(not(target_os = "linux"))]
pub use empty_impl::GstAutoCam; 