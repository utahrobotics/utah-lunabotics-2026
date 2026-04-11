use cu29::prelude::*;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use gstreamer::prelude::*;

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use circular_buffer::CircularBuffer;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use gstreamer::{BufferRef, Caps, FlowSuccess, Pipeline, parse};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use gstreamer_app::{AppSink, AppSinkCallbacks};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::str::FromStr;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::sync::{Arc, Mutex};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::time::{Duration, Instant};

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
use serde::{Deserialize, Serialize};

use crate::tasks::NewDevice;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::tasks::ai::blackboard::BLACKBOARD_SHARED;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::utils::rwlock_read_unpoison;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub use cu_gstreamer::CuGstBuffer;

pub type CuDefaultAutoGStreamer = CuAutoGStreamer<16>;

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub struct CuAutoGStreamer<const N: usize> {
    desired_port: String,
    camera_id: String,
    pipeline_template: String,
    caps_str: String,

    pipeline: Option<Pipeline>,
    appsink: Option<AppSink>,
    circular_buffer: Arc<Mutex<CircularBuffer<N, CuGstBuffer>>>,
    last_frame_time: Option<Instant>,
    pending_dev_path: Option<String>,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl<const N: usize> Freezable for CuAutoGStreamer<N> {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl<const N: usize> CuTask for CuAutoGStreamer<N> {
    type Input<'m> = input_msg!(NewDevice);
    type Output<'m> = output_msg!(CuGstBuffer);
    type Resources<'r> = ();

    fn new(cfg: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        if !gstreamer::INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            gstreamer::init()
                .map_err(|e| CuError::new_with_cause("Failed to initialise gstreamer", e))?;
        }

        let cfg = cfg.ok_or("No config provided for CuAutoGStreamer")?;

        Ok(Self {
            desired_port: cfg
                .get::<String>("device_port")?
                .ok_or("'device_port' missing from config")?,
            camera_id: cfg
                .get::<String>("camera_id")?
                .ok_or("'camera_id' missing from config")?,
            pipeline_template: cfg
                .get::<String>("pipeline")?
                .ok_or("'pipeline' missing from config")?,
            caps_str: cfg
                .get::<String>("caps")?
                .ok_or("'caps' missing from config")?,
            pipeline: None,
            appsink: None,
            circular_buffer: Arc::new(Mutex::new(CircularBuffer::new())),
            last_frame_time: None,
            pending_dev_path: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(dev_path) = self.pending_dev_path.take() {
            self.stop_pipeline();
            if let Err(e) = self.open_pipeline(&dev_path) {
                eprintln!(
                    "Gstreamer [{}]: Failed to open pipeline: {}",
                    self.camera_id, e
                );
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
        output.clear_payload();

        if let Some(bb) = BLACKBOARD_SHARED.get() && !rwlock_read_unpoison(&bb).enable_apriltags {
            return Ok(());
        }

        if let Some(dev) = input.payload() {
            if *dev.port == self.desired_port {
                println!(
                    "GStreamer [{}]: Device detected at {}",
                    self.camera_id, &dev.dev_path
                );
                self.pending_dev_path = Some(dev.dev_path.to_string());
            }
        }

        if let Some(buffer) = self.circular_buffer.lock().unwrap().pop_front() {
            output.tov = clock.now().into();
            output.set_payload(buffer);
            self.last_frame_time = Some(Instant::now());
        }

        match self.last_frame_time {
            Some(t) if t.elapsed() <= Duration::from_millis(500) => Ok(()),
            Some(_) => Err(CuError::new_with_cause(
                "no frames received in 500 ms",
                std::io::Error::other("no frames received in 500 ms"),
            )),
            None => Err(CuError::new_with_cause(
                "no frames received",
                std::io::Error::other("no frames received"),
            )),
        }
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.stop_pipeline();
        Ok(())
    }
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl<const N: usize> CuAutoGStreamer<N> {
    fn open_pipeline(&mut self, dev_path: &str) -> CuResult<()> {
        let pipeline_str = self.pipeline_template.replace("<devpath>", dev_path);
        let pipeline = parse::launch(&pipeline_str)
            .map_err(|e| CuError::new_with_cause("Failed to parse pipeline", e))?
            .dynamic_cast::<Pipeline>()
            .map_err(|_| CuError::from("Parsed element is not a Pipeline"))?;

        let appsink = pipeline
            .by_name(&format!("copper_{}", self.camera_id))
            .ok_or("Appsink not found in pipeline")?
            .dynamic_cast::<AppSink>()
            .map_err(|_| CuError::from("Element is not an AppSink"))?;

        appsink
            .set_caps(Some(&Caps::from_str(&self.caps_str).map_err(|e| {
                CuError::new_with_cause("Failed to parse caps", e)
            })?));

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
        println!(
            "GStreamer [{}]: Callbacks set for {}",
            self.camera_id, dev_path
        );

        pipeline
            .set_state(gstreamer::State::Playing)
            .map_err(|e| CuError::new_with_cause("Failed to set pipeline to Playing", e))?;

        println!(
            "GStreamer [{}]: Pipeline started for {}",
            self.camera_id, dev_path
        );

        self.pipeline = Some(pipeline);
        self.appsink = Some(appsink);
        self.last_frame_time = None;
        Ok(())
    }

    fn stop_pipeline(&mut self) {
        if let Some(pipeline) = self.pipeline.take() {
            let _ = pipeline.set_state(gstreamer::State::Null);
            println!("GStreamer [{}]: Pipeline stopped", self.camera_id);
        }
        self.appsink = None;
        self.circular_buffer.lock().unwrap().clear();
        self.last_frame_time = None;
    }
}

// ============================================================================
// Stub for non-Linux / simulation
// ============================================================================

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
pub struct CuAutoGStreamer<const N: usize> {}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
#[derive(Debug, Clone, cu_bincode::Encode, cu_bincode::Decode, Serialize, Deserialize, Default)]
pub struct CuGstBuffer {}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
impl<const N: usize> Freezable for CuAutoGStreamer<N> {}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
impl<const N: usize> CuTask for CuAutoGStreamer<N> {
    type Input<'m> = input_msg!(NewDevice);
    type Output<'m> = output_msg!(CuGstBuffer);
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
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.clear_payload();
        Err(CuError::new_with_cause(
            "no frames received",
            std::io::Error::other("no frames received"),
        ))
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
