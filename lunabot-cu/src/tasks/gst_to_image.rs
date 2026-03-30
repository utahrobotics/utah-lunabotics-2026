#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
use cu_sensor_payloads::CuImage;
///! Adapted from DynThreshold, original author Guillame Binet
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::prelude::*;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::cmp::{max, min};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::ops::DerefMut;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::sync::Arc;

use crate::tasks::auto_gstreamer::CuGstBuffer;

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub trait PixelReadAccess<U> {
    fn get_pixel(&self, x: usize, y: usize, width: usize) -> U;
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub trait PixelWriteAccess<U> {
    fn put_pixel(&mut self, x: usize, y: usize, width: usize, value: U);
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl<U: Copy, T: AsRef<[U]>> PixelReadAccess<U> for T {
    #[inline]
    fn get_pixel(&self, x: usize, y: usize, width: usize) -> U {
        unsafe {
            let slice = self.as_ref();
            *slice.get_unchecked(x + y * width)
        }
    }
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl<U: Copy, T: AsMut<[U]>> PixelWriteAccess<U> for T {
    #[inline]
    fn put_pixel(&mut self, x: usize, y: usize, width: usize, value: U) {
        unsafe {
            *self.as_mut().get_unchecked_mut(x + y * width) = value;
        }
    }
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
fn integral_image(src: &[u8], mut dst: &mut [u32], width: u32, height: u32) {
    let (width, height) = (width as usize, height as usize);
    let out_width = width + 1;

    for y in 0..height {
        let mut sum = 0;
        for x in 0..width {
            sum += src.get_pixel(x, y, width) as u32;

            let above = dst.get_pixel(x + 1, y, out_width);
            dst.put_pixel(x + 1, y + 1, out_width, above + sum);
        }
    }
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
fn sum_image_pixels(
    integral_image: &[u32],
    left: usize,
    top: usize,
    right: usize,
    bottom: usize,
    width: usize,
) -> u32 {
    let (a, b, c, d) = (
        integral_image.get_pixel(right + 1, bottom + 1, width) as i64,
        integral_image.get_pixel(left, top, width) as i64,
        integral_image.get_pixel(right + 1, top, width) as i64,
        integral_image.get_pixel(left, bottom + 1, width) as i64,
    );
    let sum = a + b - c - d;
    assert!(sum >= 0);
    sum as u32
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
fn adaptive_threshold(
    src: &[u8],
    integral: &[u32],
    mut dst: &mut [u8],
    block_radius: u32,
    width: usize,
    height: usize,
) {
    assert!(block_radius > 0);

    for y in 0..height {
        for x in 0..width {
            let current_pixel = src.get_pixel(x, y, width) as u32;

            // Traverse all neighbors in (2 * block_radius + 1) x (2 * block_radius + 1)
            let (y_low, y_high) = (
                max(0, y as i32 - block_radius as i32) as usize,
                min(height - 1, y + block_radius as usize),
            );
            let (x_low, x_high) = (
                max(0, x as i32 - block_radius as i32) as usize,
                min(width - 1, x + block_radius as usize),
            );

            // Number of pixels in the block, adjusted for edge cases.
            let w = ((y_high - y_low + 1) * (x_high - x_low + 1)) as u32;
            let sum = sum_image_pixels(integral, x_low, y_low, x_high, y_high, width + 1);

            if current_pixel * w > sum {
                dst.put_pixel(x, y, width, 255);
            } else {
                dst.put_pixel(x, y, width, 0);
            }
        }
    }
}

/// A task that computes a dynamic threshold for a grayscale image.
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub struct GstToImage {
    integral_img: Vec<u32>,
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    height: u32,
    width: u32,
    block_radius: u32,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl Freezable for GstToImage {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuTask for GstToImage {
    type Input<'m> = input_msg!(CuGstBuffer);
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.expect("No config provided");
        let width = config.get::<u32>("width")?.expect("No width provided");
        let height = config.get::<u32>("height")?.expect("No height provided");
        let block_radius = config
            .get::<u32>("block_radius")?
            .expect("No block_radius provided");

        let pool =
            CuHostMemoryPool::new("dynthreshold", 4, || vec![0u8; (width * height) as usize])?;
        Ok(GstToImage {
            integral_img: vec![0; ((width + 1) * (height + 1)) as usize],
            pool,
            width,
            height,
            block_radius,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.clear_payload();
  
        let Some(buffer_hold) = input.payload() else {
            return Ok(());  
        };
        let buffer_hold = buffer_hold
            .as_ref()
            .map_readable()
            .map_err(|e| CuError::new_with_cause("Could not map the gstreamer buffer", e))?;
        let src = buffer_hold.as_slice();

        if src.len() != (self.width * self.height) as usize {
            return Err(CuError::from(format!(
                "Input buffer size does not match the expected size {}, slice {}",
                self.width * self.height,
                src.len(),
            )));
        }

        let handle = self
            .pool
            .acquire()
            .ok_or(CuError::from("Failed to acquire buffer from pool"))?;
        {
            let mut dst = handle
                .write()
                .map_err(|e| CuError::new_with_cause("Failed to lock buffer", std::io::Error::other(e.to_string())))?;
            let dst = dst.deref_mut().deref_mut();

            integral_image(src, &mut self.integral_img, self.width, self.height);

            adaptive_threshold(
                src,
                &self.integral_img,
                dst,
                self.block_radius,
                self.width as usize,
                self.height as usize,
            );
        }
        let image = CuImage::new(
            CuImageBufferFormat {
                width: self.width,
                height: self.height,
                stride: self.width,
                pixel_format: "GRAY"
                    .as_bytes()
                    .try_into()
                    .map_err(|_| CuError::from("Failed to convert pixel format to byte array"))?,
            },
            handle,
        );

        output.tov = input.tov;
        output.set_payload(image);
        Ok(())
    }
}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
pub struct GstToImage;

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
impl Freezable for GstToImage {}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
impl CuTask for GstToImage {
    type Input<'m> = input_msg!(CuGstBuffer);
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
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
