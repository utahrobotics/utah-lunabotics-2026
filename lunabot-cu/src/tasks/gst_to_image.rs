use cu29::prelude::*;
use cu_gstreamer::CuGstBuffer;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use std::sync::Arc;
use std::time::Instant;
use std::ops::DerefMut;

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

impl<'cl> CuTask<'cl> for GstToImage {
    type Input = input_msg!('cl, CuGstBuffer);
    type Output = output_msg!('cl, CuImage<Vec<u8>>);

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

        // Convert pixel format string to 4-byte array
        let pixel_format: [u8; 4] = pixel_format_str
            .as_bytes()
            .try_into()
            .map_err(|_| CuError::from("Pixel format must be exactly 4 characters"))?;

        // Calculate expected buffer size based on pixel format
        let expected_buffer_size = match pixel_format_str.as_str() {
            "GRAY" | "Y800" => (width * height) as usize,           // 1 byte per pixel
            "RGB " | "BGR " => (width * height * 3) as usize,       // 3 bytes per pixel
            "RGBA" | "BGRA" => (width * height * 4) as usize,       // 4 bytes per pixel
            "YUY2" | "UYVY" => (width * height * 2) as usize,       // 2 bytes per pixel (YUV 4:2:2)
            _ => return Err(CuError::from(format!("Unsupported pixel format: {}", pixel_format_str))),
        };

        let pool = CuHostMemoryPool::new("gst_to_image", 10, move || {
            // Pre-allocate with capacity to avoid reallocations
            let mut vec = Vec::with_capacity(expected_buffer_size);
            unsafe { vec.set_len(expected_buffer_size); }
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
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {        
        if input.payload().is_none() {
            debug!("GstToImage: No payload in input message, skipping.");
            return Ok(());
        }

        let gst_buffer = input.payload().ok_or(CuError::from("No payload"))?;
        
        // Get a pre-allocated buffer from the pool FIRST to minimize lock time
        let handle = self
            .pool
            .acquire()
            .ok_or(CuError::from("Failed to acquire buffer from pool"))?;
        
        let map_start = Instant::now();
        // Map the buffer as readable to get access to the data
        let buffer_hold = gst_buffer
            .as_ref()
            .map_readable()
            .map_err(|e| CuError::new_with_cause("Could not map the gstreamer buffer", e))?;
        let src = buffer_hold.as_slice();

        // Fast size check without additional allocations
        if src.len() != self.expected_buffer_size {
            return Err(CuError::from(format!(
                "Buffer size mismatch: expected {}, got {}",
                self.expected_buffer_size,
                src.len()
            )));
        }

        let copy_start = Instant::now();
        // Minimize lock scope - do the fastest possible copy
        {
            let mut dst_guard = handle
                .lock()
                .map_err(|e| CuError::new_with_cause("Failed to lock buffer", e))?;
            let dst = dst_guard.deref_mut();

            // Try different copy strategies based on buffer size and performance
            unsafe {
                let src_ptr = src.as_ptr();
                let dst_ptr = dst.as_mut_ptr();
                let len = src.len();
                
                // For 640x480 images, try chunk-based copying for better cache behavior
                if len == 307200 {
                    // Copy in 64KB chunks to stay in L1 cache
                    const CHUNK_SIZE: usize = 65536;
                    let mut offset = 0;
                    while offset + CHUNK_SIZE <= len {
                        std::ptr::copy_nonoverlapping(
                            src_ptr.add(offset),
                            dst_ptr.add(offset),
                            CHUNK_SIZE
                        );
                        offset += CHUNK_SIZE;
                    }
                    // Copy remaining bytes
                    if offset < len {
                        std::ptr::copy_nonoverlapping(
                            src_ptr.add(offset),
                            dst_ptr.add(offset),
                            len - offset
                        );
                    }
                } else {
                    // For other sizes, use standard copy
                    std::ptr::copy_nonoverlapping(src_ptr, dst_ptr, len);
                }
            }
            // Lock is dropped here immediately
        }

        // Create the CuImage with the buffer handle (no additional copy)
        let image = CuImage::new(
            CuImageBufferFormat {
                width: self.width,
                height: self.height,
                stride: self.width,
                pixel_format: self.pixel_format,
            },
            handle, // Transfer ownership of the buffer to CuImage
        );

        // Preserve timestamp and set the output
        output.tov = input.tov;
        output.set_payload(image);
        Ok(())
    }
}

#[cfg(test)]
#[cfg(feature = "gst")]
mod tests {
    use super::*;
    use gstreamer::Buffer;
    use std::ops::Deref;

    #[test]
    fn test_gst_to_image_grayscale() -> Result<(), Box<dyn std::error::Error>> {
        let width = 4;
        let height = 4;
        gstreamer::init().unwrap();

        let mut config = ComponentConfig::default();
        config.set("width", width as u32);
        config.set("height", height as u32);
        config.set("pixel_format", "GRAY".to_string());

        let mut converter = GstToImage::new(Some(&config))?;

        let input_data = vec![
            100, 120, 140, 160, // L1
            110, 130, 150, 170, // L2
            120, 140, 160, 180, // L3
            130, 150, 170, 190, // L4
        ];

        // Create a GStreamer buffer with test data
        let gstreamer_buffer = Buffer::from_mut_slice(input_data.clone());
        let cu_gst_buffer = CuGstBuffer(gstreamer_buffer);
        let input_msg = CuStampedData::new(Some(cu_gst_buffer));
        let mut output_image_msg = CuStampedData::<CuImage<Vec<u8>>, CuMsgMetadata>::default();
        let output: <GstToImage as CuTask>::Output = &mut output_image_msg;

        let clock = cu29::clock::RobotClock::new();
        let result = converter.process(&clock, &input_msg, output);
        assert!(result.is_ok());

        // Verify the output
        let output_image = output_image_msg.payload().unwrap();
        let hold = output_image.buffer_handle.lock().unwrap();
        let output_data = hold.deref();

        assert_eq!(output_data, &input_data);
        assert_eq!(output_image.format.width, width);
        assert_eq!(output_image.format.height, height);
        assert_eq!(output_image.format.pixel_format, *b"GRAY");

        Ok(())
    }

    #[test]
    fn test_gst_to_image_rgb() -> Result<(), Box<dyn std::error::Error>> {
        let width = 2;
        let height = 2;
        gstreamer::init().unwrap();

        let mut config = ComponentConfig::default();
        config.set("width", width as u32);
        config.set("height", height as u32);
        config.set("pixel_format", "RGB ".to_string());

        let mut converter = GstToImage::new(Some(&config))?;

        let input_data = vec![
            255, 0, 0,    // Red pixel
            0, 255, 0,    // Green pixel
            0, 0, 255,    // Blue pixel
            255, 255, 255, // White pixel
        ];

        let gstreamer_buffer = Buffer::from_mut_slice(input_data.clone());
        let cu_gst_buffer = CuGstBuffer(gstreamer_buffer);
        let input_msg = CuStampedData::new(Some(cu_gst_buffer));
        let mut output_image_msg = CuStampedData::<CuImage<Vec<u8>>, CuMsgMetadata>::default();
        let output: <GstToImage as CuTask>::Output = &mut output_image_msg;

        let clock = cu29::clock::RobotClock::new();
        let result = converter.process(&clock, &input_msg, output);
        assert!(result.is_ok());

        let output_image = output_image_msg.payload().unwrap();
        let hold = output_image.buffer_handle.lock().unwrap();
        let output_data = hold.deref();

        assert_eq!(output_data, &input_data);
        assert_eq!(output_image.format.pixel_format, *b"RGB ");

        Ok(())
    }
}
