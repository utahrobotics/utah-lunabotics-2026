// auto cam task:
// This allows this module to be used on simulation on Windows and MacOS

#[cfg(not(target_os = "linux"))]
pub use empty_impl::V4lAutoCam;

#[cfg(target_os = "linux")]
pub use linux_impl::V4lAutoCam;

#[cfg(target_os = "linux")]
mod linux_impl {
    use bincode::de;
    use std::any;
    use std::f32::consts::E;
    use std::ops::Deref;
    use std::time::{Duration, Instant};
    use v4l::video::Capture;

    use crate::tasks::udev_monitor::NewDevice;
    use crate::tasks::v4lstream::CuV4LStream;
    use cu29::cutask::CuMsg;
    use cu29::prelude::*;
    use cu_sensor_payloads::{CuImage, CuImageBufferFormat};

    use nix::time::{clock_gettime, ClockId};

    pub use v4l::buffer::Type;
    pub use v4l::framesize::FrameSizeEnum;
    pub use v4l::io::traits::{CaptureStream, Stream};
    pub use v4l::prelude::*;
    pub use v4l::video::capture::Parameters;
    pub use v4l::{Format, FourCC, Timestamp};

    // A Copper task that will open a V4L camera when the udev monitor notifies us that
    // the correct device has been plugged in. Until the matching device is detected the
    // task does nothing. Once the camera is opened it behaves exactly like the previous
    // implementation and streams images out at each call to `process`.
    pub struct V4lAutoCam {
        // Open and active V4L stream once the device is detected. `None` while waiting.
        stream: Option<CuV4LStream>,
        // Format negotiated with the camera. Only valid once `stream` is `Some`.
        settled_format: Option<CuImageBufferFormat>,
        // Offset between V4L monotonic timestamps and Copper `RobotClock`.
        v4l_clock_time_offset_ns: i64,

        // Configuration parameters parsed from the RON configuration file.
        desired_port: String,
        req_width: Option<u32>,
        req_height: Option<u32>,
        req_fps: Option<u32>,
        req_fourcc: Option<String>,
        req_buffers: u32,
        req_timeout: Duration,
        last_frame_time: Option<Instant>,
        // Track if we've successfully sent at least one frame
        has_sent_frame: bool,
    }

    impl Freezable for V4lAutoCam {}

    fn cutime_from_v4ltime(offset_ns: i64, v4l_time: Timestamp) -> CuTime {
        let duration: Duration = v4l_time.into();
        ((duration.as_nanos() as i64 + offset_ns) as u64).into()
    }

    impl CuTask for V4lAutoCam {
        type Output<'m> = output_msg!(CuImage<Vec<u8>>);
        type Input<'m> = input_msg!(NewDevice); // will have the camera port and device path

        fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            // reasonable defaults
            let mut desired_port = String::new();
            let mut req_width: Option<u32> = None;
            let mut req_height: Option<u32> = None;
            let mut req_fps: Option<u32> = None;
            let mut req_fourcc: Option<String> = None;
            let mut req_buffers: u32 = 4;
            let mut req_timeout: Duration = Duration::from_millis(1); // Non-blocking timeout

            if let Some(cfg) = config {
                if let Some(port) = cfg.get::<String>("device_port") {
                    desired_port = port;
                }
                if let Some(width) = cfg.get::<u32>("width") {
                    req_width = Some(width);
                }
                if let Some(height) = cfg.get::<u32>("height") {
                    req_height = Some(height);
                }
                if let Some(fps) = cfg.get::<u32>("fps") {
                    req_fps = Some(fps);
                }
                if let Some(fourcc) = cfg.get::<String>("fourcc") {
                    req_fourcc = Some(fourcc);
                }
                if let Some(buffers) = cfg.get::<u32>("buffers") {
                    req_buffers = buffers;
                }
                if let Some(timeout) = cfg.get::<u32>("timeout_ms") {
                    req_timeout = Duration::from_millis(timeout as u64);
                }
            }

            Ok(Self {
                stream: None,
                settled_format: None,
                v4l_clock_time_offset_ns: 0,
                desired_port,
                req_width,
                req_height,
                req_fps,
                req_fourcc,
                req_buffers,
                req_timeout,
                last_frame_time: None,
                has_sent_frame: false,
            })
        }

        fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
            // We cannot start streaming until the camera is plugged in and detected.
            Ok(())
        }

        fn process(
            &mut self,
            clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if self.stream.is_none() {
                if let Some(dev) = input.payload() {
                    if dev.port.to_string() == self.desired_port {
                        if let Err(e) = self.open_stream(&dev.dev_path, clock) {
                            error!("Failed to open V4L stream: {}", e);
                        }
                    }
                }
            }

            if let Some(stream) = self.stream.as_mut() {
                match stream.try_next() {
                    Ok(Some((handle, meta))) => {
                        if meta.bytesused != 0 {
                            if let Some(ref fmt) = self.settled_format {
                                let cutime = cutime_from_v4ltime(
                                    self.v4l_clock_time_offset_ns,
                                    meta.timestamp,
                                );
                                let image = CuImage::new(*fmt, handle.clone());

                                output.set_payload(image);
                                output.tov = Tov::Time(cutime);
                                self.last_frame_time = Some(Instant::now());

                                // Requeue buffer after successful processing
                                if let Err(e) = stream.requeue(stream.last_dequeued_index()) {
                                    error!("Failed to requeue buffer: {}", e.to_string());
                                }
                            } else {
                                debug!("V4L: No settled format, skipping frame");
                                // Still need to requeue buffer
                                if let Err(e) = stream.requeue(stream.last_dequeued_index()) {
                                    error!("Failed to requeue buffer: {}", e.to_string());
                                }
                                return Err("V4L stream format not negotiated".into());
                            }
                        } else {
                            debug!("V4L: Received empty frame");
                            // Requeue buffer even for empty frame
                            if let Err(e) = stream.requeue(stream.last_dequeued_index()) {
                                error!("Failed to requeue buffer: {}", e.to_string());
                            }
                            return Err("Received empty frame from V4L stream".into());
                        }
                    }
                    Ok(None) => {
                        // No frame available, normal non-blocking case
                    }
                    Err(e) => {
                        error!("V4L: Error reading frame: {}", e.to_string());
                        return Err("error reading frame".into());
                    }
                }
            }

            // Reset stream if no frames for 1 second
            if let Some(last_frame_time) = self.last_frame_time {
                if last_frame_time.elapsed().as_secs() > 1 {
                    debug!("V4L: No frames for 1s, resetting stream");
                    self.stream = None;
                    self.settled_format = None;
                    self.last_frame_time = None;
                }
            }

            Ok(())
        }

        fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
            if let Some(stream) = self.stream.as_mut() {
                if let Err(e) = stream.stop() {
                    error!("Error stopping stream: {}", e.to_string());
                }
            }
            Ok(())
        }
    }

    impl V4lAutoCam {
        // Helper that attempts to open the camera corresponding to `dev_path`. It negotiates the
        // format according to the configuration stored in `self` and starts streaming.
        fn open_stream(&mut self, dev_path: &str, robot_clock: &RobotClock) -> CuResult<()> {
            if self.stream.is_some() {
                return Ok(()); // already opened
            }

            // Open device using its path (e.g. /dev/video4)
            let dev = Device::with_path(dev_path)
                .map_err(|e| CuError::new_with_cause("Failed to open camera", e))?;

            // List all formats supported by the device
            let formats = dev
                .enum_formats()
                .map_err(|e| CuError::new_with_cause("Failed to enum formats", e))?;

            if formats.is_empty() {
                return Err("The V4l device did not provide any video format.".into());
            }

            // Choose fourcc
            let fourcc: FourCC = if let Some(ref fourcc_str) = self.req_fourcc {
                if fourcc_str.len() != 4 {
                    return Err("Invalid fourcc provided".into());
                }
                FourCC::new(fourcc_str.as_bytes()[0..4].try_into().unwrap())
            } else {
                debug!("No fourcc provided, just use the first one we can find.");
                formats.first().unwrap().fourcc
            };
            debug!("V4L: Using fourcc: {}", fourcc.to_string());

            // Find format with selected fourcc, fallback to MJPG if not found
            let actual_fmt = if let Some(format) = formats.iter().find(|f| f.fourcc == fourcc) {
                let resolutions = dev
                    .enum_framesizes(format.fourcc)
                    .map_err(|e| CuError::new_with_cause("Failed to enum frame sizes", e))?;

                // Choose resolution
                let (width, height) = if let (Some(w), Some(h)) = (self.req_width, self.req_height)
                {
                    let mut selected: (u32, u32) = (0, 0);
                    for frame in resolutions.iter() {
                        let FrameSizeEnum::Discrete(size) = &frame.size else {
                            continue;
                        };
                        if size.width == w && size.height == h {
                            selected = (size.width, size.height);
                            break;
                        }
                    }
                    selected
                } else {
                    let fs = resolutions.first().unwrap();
                    let FrameSizeEnum::Discrete(size) = &fs.size else {
                        return Err("Unsupported frame size type".into());
                    };
                    (size.width, size.height)
                };

                let req_fmt = Format::new(width, height, fourcc);
                let actual_fmt = dev
                    .set_format(&req_fmt)
                    .map_err(|e| CuError::new_with_cause("Failed to set format", e))?;

                // Set FPS if requested
                if let Some(fps) = self.req_fps {
                    debug!("V4L: Set fps to {}", fps);
                    let new_params = Parameters::with_fps(fps);
                    dev.set_params(&new_params)
                        .map_err(|e| CuError::new_with_cause("Failed to set params", e))?;
                }
                debug!(
                    "V4L: Negotiated resolution: {}x{}",
                    actual_fmt.width, actual_fmt.height
                );
                actual_fmt
            } else {
                // Try MJPG fallback
                debug!("The V4l device does not provide a format with the FourCC {}. Trying MJPG fallback...", fourcc.to_string());
                let mjpg_fourcc = FourCC::new(b"MJPG");
                if let Some(format) = formats.iter().find(|f| f.fourcc == mjpg_fourcc) {
                    let resolutions = dev
                        .enum_framesizes(format.fourcc)
                        .map_err(|e| CuError::new_with_cause("Failed to enum frame sizes", e))?;
                    let (width, height) =
                        if let (Some(w), Some(h)) = (self.req_width, self.req_height) {
                            let mut selected: (u32, u32) = (0, 0);
                            for frame in resolutions.iter() {
                                let FrameSizeEnum::Discrete(size) = &frame.size else {
                                    continue;
                                };
                                if size.width == w && size.height == h {
                                    selected = (size.width, size.height);
                                    break;
                                }
                            }
                            selected
                        } else {
                            let fs = resolutions.first().unwrap();
                            let FrameSizeEnum::Discrete(size) = &fs.size else {
                                return Err("Unsupported frame size type".into());
                            };
                            (size.width, size.height)
                        };
                    let req_fmt = Format::new(width, height, mjpg_fourcc);
                    let actual_fmt = dev
                        .set_format(&req_fmt)
                        .map_err(|e| CuError::new_with_cause("Failed to set MJPG format", e))?;
                    if let Some(fps) = self.req_fps {
                        debug!("V4L: Set fps to {}", fps);
                        let new_params = Parameters::with_fps(fps);
                        dev.set_params(&new_params)
                            .map_err(|e| CuError::new_with_cause("Failed to set params", e))?;
                    }
                    debug!(
                        "V4L: Negotiated MJPG resolution: {}x{}",
                        actual_fmt.width, actual_fmt.height
                    );
                    actual_fmt
                } else {
                    return Err(format!(
                        "The V4l device does not provide a format with the FourCC {} or MJPG.",
                        fourcc
                    )
                    .into());
                }
            };

            // Create stream
            let mut stream = CuV4LStream::with_buffers(
                &dev,
                Type::VideoCapture,
                self.req_buffers,
                CuHostMemoryPool::new(
                    format!("V4L Host Pool {}", dev_path).as_str(),
                    self.req_buffers as usize + 1,
                    || vec![0; actual_fmt.size as usize],
                )
                .map_err(|e| {
                    CuError::new_with_cause(
                        "Could not create host memory pool backing the V4lStream",
                        e,
                    )
                })?,
            )
            .map_err(|e| CuError::new_with_cause("Could not create the V4lStream", e))?;

            // Set a very short timeout for non-blocking behavior
            stream.set_timeout(self.req_timeout);

            // Compute clock offset BEFORE starting the stream
            let rb_ns = robot_clock.now().as_nanos();
            clock_gettime(ClockId::CLOCK_MONOTONIC)
                .map(|ts| {
                    self.v4l_clock_time_offset_ns =
                        ts.tv_sec() * 1_000_000_000 + ts.tv_nsec() - rb_ns as i64;
                })
                .map_err(|e| CuError::new_with_cause("Failed to get the current time", e))?;

            stream
                .start()
                .map_err(|e| CuError::new_with_cause("could not start stream", e))?;

            let cuformat = CuImageBufferFormat {
                width: actual_fmt.width,
                height: actual_fmt.height,
                stride: actual_fmt.stride,
                pixel_format: actual_fmt.fourcc.repr,
            };

            self.settled_format = Some(cuformat);
            self.stream = Some(stream);
            info!("V4L: Opened stream on device {}", dev_path);
            Ok(())
        }
    }
}
