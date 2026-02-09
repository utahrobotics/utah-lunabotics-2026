/// Taken from Naj's gputter-core
use std::sync::OnceLock;

use pollster::FutureExt;
pub use wgpu;
use wgpu::ExperimentalFeatures;

pub struct GpuDevice {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

static GPU_DEVICE: OnceLock<GpuDevice> = OnceLock::new();

pub async fn init_gpu() -> anyhow::Result<()> {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });

    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        })
        .await
        .or_else(|e| Err(anyhow::anyhow!("Failed to request adapter: {e}")))?;

    let (device, queue) = adapter
        .request_device(&wgpu::DeviceDescriptor {
            trace: wgpu::Trace::Off,
            required_features: wgpu::Features::empty(),
            // WebGL doesn't support all of wgpu's features, so if
            // we're building for the web, we'll have to disable some.
            required_limits: if cfg!(target_arch = "wasm32") {
                wgpu::Limits::downlevel_webgl2_defaults()
            } else {
                wgpu::Limits::default()
            },
            memory_hints: wgpu::MemoryHints::Performance,
            label: None,
            experimental_features: unsafe { ExperimentalFeatures::enabled() },
        })
        .await?;
    let _ = GPU_DEVICE.set(GpuDevice { device, queue });
    Ok(())
}

pub fn get_device() -> &'static GpuDevice {
    GPU_DEVICE
        .get()
        .expect("GpuDevice was not initialized. Call init_gputter first")
}

pub fn init_gpu_blocking() -> anyhow::Result<()> {
    init_gpu().block_on()
}

pub fn is_gpu_initialized() -> bool {
    GPU_DEVICE.get().is_some()
}
