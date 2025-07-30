use cu29::prelude::*;
use gstreamer::prelude::*;

use circular_buffer::CircularBuffer;
use gstreamer::{parse, BufferRef, Caps, FlowSuccess, Pipeline};
use gstreamer_app::{AppSink, AppSinkCallbacks};
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::tasks::NewDevice;

// Re-export CuGstBuffer from the shared cu_gstreamer crate so we don't redefine it and cause
// type mismatches in Copper's type-matching logic.
pub use cu_gstreamer::CuGstBuffer;

pub type CuDefaultAutoGStreamer = CuAutoGStreamer<16>;

/// Automatically starts a GStreamer pipeline when the desired camera is detected by `UdevMonitor`.
/// While the pipeline is running it feeds the most recent buffers through a circular queue.
/// If no frame is received for `timeout_ms` the pipeline is torn down so it can be rebuilt when the
/// device re-appears (e.g. hot-unplug/plug).
pub struct CuAutoGStreamer<const N: usize> {
    // Desired USB port – matches `NewDevice.port`.
    desired_port: String,
    // User-friendly camera identifier (used only for logging).
    _camera_id: String,
    // Template pipeline string – contains the placeholder `<devpath>`.
    pipeline_template: String,
    // Caps string applied to the pipeline's appsink.
    caps_str: String,
    // Frame timeout.
    req_timeout: Duration,

    // Runtime state -------------------------------------------------------------------------
    pipeline: Option<Pipeline>,
    _appsink: Option<AppSink>,
    circular_buffer: Arc<Mutex<CircularBuffer<N, CuGstBuffer>>>,
    last_frame_time: Option<Instant>,
    // `Some` after we received a matching `NewDevice`, but before the pipeline is created.
    pending_dev_path: Option<String>,
    teardown_requested: bool,
}

impl<const N: usize> Freezable for CuAutoGStreamer<N> {}

impl<const N: usize> CuTask for CuAutoGStreamer<N> {
    type Input<'m> = input_msg!(NewDevice);
    type Output<'m> = output_msg!(CuGstBuffer);

    // -------------------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------------------
    fn new(cfg: Option<&ComponentConfig>) -> CuResult<Self> {
        // Ensure GStreamer is initialised exactly once.
        if !gstreamer::INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            gstreamer::init()
                .map_err(|e| CuError::new_with_cause("Failed to initialise gstreamer", e))?;
        }

        let cfg = cfg.ok_or("No config provided for CuAutoGStreamer")?;

        let desired_port = cfg
            .get::<String>("device_port")
            .ok_or("'device_port' missing from config")?;
        let camera_id = cfg
            .get::<String>("camera_id")
            .ok_or("'camera_id' missing from config")?;
        let pipeline_template = cfg
            .get::<String>("pipeline")
            .ok_or("'pipeline' missing from config")?;
        let caps_str = cfg
            .get::<String>("caps")
            .ok_or("'caps' missing from config")?;
        let req_timeout = Duration::from_millis(cfg.get::<u32>("timeout_ms").unwrap_or(500) as u64);

        Ok(Self {
            desired_port,
            _camera_id: camera_id,
            pipeline_template,
            caps_str,
            req_timeout,
            pipeline: None,
            _appsink: None,
            circular_buffer: Arc::new(Mutex::new(CircularBuffer::new())),
            last_frame_time: None,
            pending_dev_path: None,
            teardown_requested: false,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn preprocess(&mut self, clock: &RobotClock) -> CuResult<()> {
        // Handle requested teardowns first
        if self.teardown_requested {
            self.teardown_requested = false;
            info!("GStreamer: Teardown requested");
            self.stop_pipeline();
        }

        // Then handle pipeline creation
        if self.pipeline.is_none() {
            if let Some(dev_path) = self.pending_dev_path.take() {
                self.open_pipeline(&dev_path)?;
            }
        }
        Ok(())
    }

    fn process(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if self.pipeline.is_none() {
            if let Some(dev) = input.payload() {
                if *dev.port == self.desired_port {
                    info!("GStreamer: Found device {}", &dev.dev_path);
                    self.pending_dev_path = Some(dev.dev_path.to_string());
                }
            }
            output.clear_payload();
            return Ok(());
        }

        // Handle frame timeout without blocking
        if let Some(last) = self.last_frame_time {
            if last.elapsed().as_millis() > self.req_timeout.as_millis() {
                info!(
                    "GStreamer: Frame timeout (>{}). Requesting teardown.",
                    self.req_timeout
                );
                self.teardown_requested = true;
                self.last_frame_time = None;
            }
        }

        // Process frames (non-blocking)
        let frame = self.circular_buffer.lock().unwrap().pop_front();
        if let Some(buffer) = frame {
            output.tov = clock.now().into();
            output.set_payload(buffer);
            self.last_frame_time = Some(Instant::now());
        } else {
            output.clear_payload();
        }
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.stop_pipeline();
        Ok(())
    }
}

impl<const N: usize> CuAutoGStreamer<N> {
    fn open_pipeline(&mut self, dev_path: &str) -> CuResult<()> {
        let pipeline_str = self.pipeline_template.replace("<devpath>", dev_path);
        let pipeline = parse::launch(&pipeline_str)
            .map_err(|e| CuError::new_with_cause("Failed to parse pipeline", e))?;
        let pipeline = pipeline
            .dynamic_cast::<Pipeline>()
            .map_err(|_| CuError::from("Parsed element is not a Pipeline"))?;
        println!("appsink name: {}", format!("copper_{}", self._camera_id));
        let appsink = pipeline
            .by_name(&format!("copper_{}", self._camera_id))
            .ok_or("Appsink element named 'copper' not found in pipeline")?
            .dynamic_cast::<AppSink>()
            .map_err(|_| CuError::from("Element 'copper' is not an AppSink"))?;
        let caps = Caps::from_str(&self.caps_str)
            .map_err(|e| CuError::new_with_cause("Failed to parse caps", e))?;
        appsink.set_caps(Some(&caps));

        self.circular_buffer.lock().unwrap().clear();
        let circular_buffer = self.circular_buffer.clone();

        appsink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample(move |appsink| {
                    let sample = appsink
                        .pull_sample()
                        .map_err(|_| gstreamer::FlowError::Eos)?;
                    let buffer: &BufferRef = sample.buffer().ok_or(gstreamer::FlowError::Error)?;
                    circular_buffer
                        .lock()
                        .unwrap()
                        .push_back(CuGstBuffer(buffer.to_owned()));
                    Ok(FlowSuccess::Ok)
                })
                .build(),
        );

        // Start playing.
        pipeline
            .set_state(gstreamer::State::Playing)
            .map_err(|e| CuError::new_with_cause("Failed to set pipeline to Playing", e))?;

        self.pipeline = Some(pipeline);
        self._appsink = Some(appsink);
        self.last_frame_time = None;
        Ok(())
    }

    fn stop_pipeline(&mut self) {
        info!("GStreamer: Stopping pipeline");
        if let Some(pipeline) = &self.pipeline {
            let _ = pipeline.set_state(gstreamer::State::Null);
        }
        self.pipeline = None;
        self._appsink = None;
        self.circular_buffer.lock().unwrap().clear();
        self.last_frame_time = None;
    }
}
