use cu29::prelude::*;
use bincode::{Decode, Encode};
use cu_apriltag::AprilTagDetections;
use crossbeam_channel::{bounded, Receiver};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::thread;
use udev::{EventType};
use v4l::{buffer::Type as V4lType, io::traits::CaptureStream, prelude::MmapStream, video::Capture};
use image::codecs::jpeg::JpegDecoder;
use image::ImageDecoder;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::prelude::CuHandle;
use std::io::Cursor;
use std::time::{Instant, Duration};
use cu29::CuError;

// Sink that logs AprilTag detections
#[derive(Default)]
pub struct DetectionLogger;

impl Freezable for DetectionLogger {}

impl<'cl> CuSinkTask<'cl> for DetectionLogger {
    type Input = input_msg!('cl, AprilTagDetections);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        if let Some(dets) = input.payload() {
        }
        Ok(())
    }
}

pub struct UdevMonitor {
    /// Underlying udev monitor socket. We create an iterator on-demand each
    /// time we wish to poll for the next event.
    monitor_socket: Option<udev::MonitorSocket>,
}

#[derive(Debug, Clone, Encode, Decode)]
pub struct NewDevice {
    port: String,
    dev_path: String,
}


impl Default for NewDevice {
    fn default() -> Self {
        Self { port: "pci-0000:35:00.4-usb-0:1:1.3".to_string(), dev_path: "/dev/video4".to_string() }
    }
}


impl Freezable for UdevMonitor {}

impl<'cl> CuSrcTask<'cl> for UdevMonitor {
    type Output = output_msg!('cl, NewDevice);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self { monitor_socket: None})
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        // Create a udev monitor socket listening for video4linux subsystem events.
        let monitor_socket = udev::MonitorBuilder::new()
            .and_then(|m| m.match_subsystem("video4linux"))
            .and_then(|m| m.listen())
            .map_err(|e| CuError::new_with_cause("Failed to initialize udev monitor", e))?;
        self.monitor_socket = Some(monitor_socket);
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        if let Some(monitor_socket) = &self.monitor_socket {
            // Create an iterator from the socket and fetch the next event if any.
            if let Some(event) = monitor_socket.iter().next() { // blocking call, todo make non-blocking
                debug!("got event");
                match event.event_type() {
                    EventType::Add => {
                        if let Some(devnode) = event.devnode() && event.property_value("ID_PATH").is_some() {
                            if let Some(path_str) = devnode.to_str() {
                                if path_str.starts_with("/dev/video") {
                                    debug!("got add event for {}", path_str);
                                    output.set_payload(NewDevice { 
                                        port: event.property_value("ID_PATH").unwrap().to_string_lossy().to_string(), 
                                        dev_path: path_str.to_string() 
                                    });
                                }
                            }
                        }
                    }
                    _ => {
                    }
                }
            }
        }
        Ok(())
    }
}


/// Copper source task that waits for a particular USB camera port to appear and then streams
/// grayscale frames (as `CuImage<Vec<u8>>`).
pub struct AutoCamera {
    last_frame: Instant,
    camera_thread: Option<thread::JoinHandle<()>>,
    rx: Option<Receiver<CuImage<Vec<u8>>>>,
    port: String,
}

impl Freezable for AutoCamera {}

impl<'cl> CuTask<'cl> for AutoCamera {
    type Input = input_msg!('cl, NewDevice);

    type Output = output_msg!('cl, CuImage<Vec<u8>>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized {
        let port: String = config
            .and_then(|c| c.get::<String>("port"))
            .unwrap_or_default();
        Ok(Self { last_frame: Instant::now(), camera_thread: None, rx: None, port })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        if let Some(NewDevice { port, dev_path }) = input.payload() && port.clone() == self.port {
            let cam_path = dev_path.clone();
            let (tx, rx) = bounded::<CuImage<Vec<u8>>>(3);
            self.camera_thread = Some(thread::spawn(move || {
                let _ = run_camera_loop(PathBuf::from(cam_path), &tx);
            }));
            self.rx = Some(rx);
        }
        if let Some(rx) = &self.rx {
            if let Ok(img) = rx.try_recv() {
                output.set_payload(img);
                self.last_frame = Instant::now();
            }
            if self.last_frame.elapsed() > Duration::from_secs(2) {
                // return Err("camera timeout".into());
            }
        }
        Ok(())
    }
}

fn run_camera_loop(path: PathBuf, tx: &crossbeam_channel::Sender<CuImage<Vec<u8>>>) -> anyhow::Result<()> {
    use anyhow::Context;
    let mut dev = v4l::Device::with_path(path).context("open camera")?;
    let fmt = dev.format().context("query format")?;
    // Build stream.
    let mut stream = MmapStream::with_buffers(&mut dev, V4lType::VideoCapture, 4)
        .context("create mmap stream")?;

    let width = fmt.width;
    let height = fmt.height;
    let mut rgb_buf = vec![0u8; width as usize * height as usize * 3];

    loop {
        let (jpeg_bytes, _) = stream.next().context("grab frame")?;
        // Decode JPEG to RGB.
        let decoder = match JpegDecoder::new(Cursor::new(jpeg_bytes)) {
            Ok(dec) => dec,
            Err(_) => continue, // skip bad frame
        };
        if decoder.dimensions() != (width, height) {
            // Unexpected, skip.
            continue;
        }
        if decoder.read_image(&mut rgb_buf).is_err() {
            continue;
        }
        // Convert to grayscale.
        let mut gray = Vec::with_capacity((width * height) as usize);
        for chunk in rgb_buf.chunks_exact(3) {
            let [r, g, b] = [chunk[0] as f32, chunk[1] as f32, chunk[2] as f32];
            let y = 0.299 * r + 0.587 * g + 0.114 * b;
            gray.push(y as u8);
        }
        // Build CuImage.
        let format = CuImageBufferFormat {
            width,
            height,
            stride: width, // packed
            pixel_format: *b"GRAY",
        };
        let img = CuImage::new(format, CuHandle::new_detached(gray));
        let _ = tx.try_send(img); // drop frame if queue full
    }
}
