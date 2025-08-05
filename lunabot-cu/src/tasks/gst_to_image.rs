use cu_gstreamer::CuGstBuffer;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::prelude::*;
use std::ops::DerefMut;
use std::sync::Arc;
use std::time::Instant;

/// A fast task that converts CuGstBuffer to CuImage<Vec<u8>> using memory pool optimization.
/// Uses pre-allocated buffers and fast memory copy to minimize overhead.
pub struct GstToImage {
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    width: u32,
    height: u32,
    pixel_format: [u8; 4],
    expected_buffer_size: usize,
}

impl Freezable for GstToImage {}

impl CuTask for GstToImage {
    type Input<'m> = input_msg!(CuGstBuffer);
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.expect("No config provided");
        let width = config.get::<u32>("width").expect("No width provided");
        let height = config.get::<u32>("height").expect("No height provided");
        let pixel_format_str = config
            .get::<String>("pixel_format")
            .unwrap_or_else(|| "GRAY".to_string());

        let pixel_format: [u8; 4] = pixel_format_str
            .as_bytes()
            .try_into()
            .map_err(|_| CuError::from("Pixel format must be exactly 4 characters"))?;

        let expected_buffer_size = match pixel_format_str.as_str() {
            "GRAY" | "Y800" => (width * height) as usize, // 1 byte per pixel
            "RGB " | "BGR " => (width * height * 3) as usize, // 3 bytes per pixel
            "RGBA" | "BGRA" => (width * height * 4) as usize, // 4 bytes per pixel
            "YUY2" | "UYVY" => (width * height * 2) as usize, // 2 bytes per pixel (YUV 4:2:2)
            _ => {
                return Err(CuError::from(format!(
                    "Unsupported pixel format: {}",
                    pixel_format_str
                )));
            }
        };

        let pool = CuHostMemoryPool::new("gst_to_image", 10, move || {
            let mut vec = Vec::with_capacity(expected_buffer_size);
            unsafe {
                vec.set_len(expected_buffer_size);
            }
            vec
        })?;

        Ok(GstToImage {
            pool,
            width,
            height,
            pixel_format,
            expected_buffer_size,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if input.payload().is_none() {
            debug!("GstToImage: No payload in input message, skipping.");
            return Ok(());
        }

        let gst_buffer = input.payload().ok_or(CuError::from("No payload"))?;

        let handle = self
            .pool
            .acquire()
            .ok_or(CuError::from("Failed to acquire buffer from pool"))?;

        let buffer_hold = gst_buffer
            .as_ref()
            .map_readable()
            .map_err(|e| CuError::new_with_cause("Could not map the gstreamer buffer", e))?;
        let src = buffer_hold.as_slice();

        if src.len() != self.expected_buffer_size {
            return Err(CuError::from(format!(
                "Buffer size mismatch: expected {}, got {}",
                self.expected_buffer_size,
                src.len()
            )));
        }

        {
            let mut dst_guard = handle
                .lock()
                .map_err(|e| CuError::new_with_cause("Failed to lock buffer", e))?;
            let dst = dst_guard.deref_mut();

            unsafe {
                let src_ptr = src.as_ptr();
                let dst_ptr = dst.as_mut_ptr();
                let len = src.len();

                std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, len);
            }
        }

        let image = CuImage::new(
            CuImageBufferFormat {
                width: self.width,
                height: self.height,
                stride: self.width,
                pixel_format: self.pixel_format,
            },
            handle,
        );

        output.tov = input.tov;
        output.set_payload(image);
        Ok(())
    }
}
