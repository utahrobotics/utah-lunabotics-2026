#![feature(associated_const_equality)]
use std::sync::OnceLock;

use pollster::FutureExt;
pub use wgpu;

pub mod buffers;
pub mod compute;
pub mod shader;
pub mod size;
pub mod tuple;
pub mod types;

pub struct GpuDevice {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

static GPU_DEVICE: OnceLock<GpuDevice> = OnceLock::new();

pub async fn init_gputter() -> anyhow::Result<()> {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });

    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::default(),
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

pub fn init_gputter_blocking() -> anyhow::Result<()> {
    init_gputter().block_on()
}

pub fn is_gputter_initialized() -> bool {
    GPU_DEVICE.get().is_some()
}
