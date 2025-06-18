#[cfg(unix)]
use std::mem::ManuallyDrop;

#[cfg(unix)]
use apriltag::{Detector, DetectorBuilder, Family, Image, TagParams};

#[cfg(unix)]
use apriltag_sys::image_u8_t;

use bincode::de::Decoder;
use bincode::error::DecodeError;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_sensor_payloads::CuImage;
use cu_spatial_payloads::Pose as CuPose;
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serialize};

// the maximum number of detections that can be returned by the detector
const MAX_DETECTIONS: usize = 16;

// Defaults
#[cfg(not(windows))]
const TAG_SIZE: f64 = 0.14;
#[cfg(not(windows))]
const FX: f64 = 2600.0;
#[cfg(not(windows))]
const FY: f64 = 2600.0;
#[cfg(not(windows))]
const CX: f64 = 900.0;
#[cfg(not(windows))]
const CY: f64 = 520.0;
#[cfg(not(windows))]
const FAMILY: &str = "tag16h5";

pub type ImageWithId = (Box<String>, CuImage<Vec<u8>>);

#[derive(Default, Debug, Clone, Encode)]
pub struct AprilTagDetections {
    pub camera_id: Box<String>,
    pub ids: CuArrayVec<usize, MAX_DETECTIONS>,
    pub poses: CuArrayVec<CuPose<f32>, MAX_DETECTIONS>,
    pub decision_margins: CuArrayVec<f32, MAX_DETECTIONS>,
}

impl Decode<()> for AprilTagDetections {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let ids = CuArrayVec::<usize, MAX_DETECTIONS>::decode(decoder)?;
        let poses = CuArrayVec::<CuPose<f32>, MAX_DETECTIONS>::decode(decoder)?;
        let decision_margins = CuArrayVec::<f32, MAX_DETECTIONS>::decode(decoder)?;
        let camera_id: Box<String> = Box::decode(decoder)?;
        Ok(AprilTagDetections {
            ids,
            poses,
            decision_margins,
            camera_id
        })
    }
}
// implement serde support for AprilTagDetections
// This is so it can be logged with debug!.
impl Serialize for AprilTagDetections {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let CuArrayVec(ids) = &self.ids;
        let CuArrayVec(poses) = &self.poses;
        let CuArrayVec(decision_margins) = &self.decision_margins;
        let camera_id= &self.camera_id;
        let mut tup = serializer.serialize_tuple(ids.len() + 1)?;

        ids.iter()
            .zip(poses.iter())
            .zip(decision_margins.iter())
            .map(|((id, pose), margin)| (id, pose, margin))
            .for_each(|(id, pose, margin)| {
                tup.serialize_element(&(id, pose, margin)).unwrap();
            });
        tup.serialize_element(camera_id)?;
        tup.end()
    }
}

impl<'de> Deserialize<'de> for AprilTagDetections {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct AprilTagDetectionsVisitor;

        impl<'de> serde::de::Visitor<'de> for AprilTagDetectionsVisitor {
            type Value = AprilTagDetections;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("a tuple of (id, pose, decision_margin) and camera_id")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mut detections = AprilTagDetections::new();
                for _ in 0..MAX_DETECTIONS {
                    if let Some((id, pose, decision_margin)) = seq.next_element()? {
                        let CuArrayVec(ids) = &mut detections.ids;
                        ids.push(id);
                        let CuArrayVec(poses) = &mut detections.poses;
                        poses.push(pose);
                        let CuArrayVec(decision_margins) = &mut detections.decision_margins;
                        decision_margins.push(decision_margin);
                    } else {
                        break;
                    }
                }
                let camera_id = seq.next_element()?.ok_or_else(|| serde::de::Error::invalid_length(MAX_DETECTIONS, &self))?;
                detections.camera_id = camera_id;
                Ok(detections)
            }
        }

        deserializer.deserialize_tuple(MAX_DETECTIONS + 1, AprilTagDetectionsVisitor)
    }
}

impl AprilTagDetections {
    fn new() -> Self {
        Self::default()
    }
    pub fn filtered_by_decision_margin(
        &self,
        threshold: f32,
    ) -> impl Iterator<Item = (usize, &CuPose<f32>, f32)> {
        let CuArrayVec(ids) = &self.ids;
        let CuArrayVec(poses) = &self.poses;
        let CuArrayVec(decision_margins) = &self.decision_margins;

        ids.iter()
            .zip(poses.iter())
            .zip(decision_margins.iter())
            .filter_map(move |((id, pose), margin)| {
                (*margin > threshold).then_some((*id, pose, *margin))
            })
    }
}

#[cfg(unix)]
pub struct AprilTags {
    detector: Detector,
    tag_params: TagParams,
    scratch: Vec<u8>,
    camera_id: Box<String>,
}

#[cfg(not(unix))]
pub struct AprilTags {}

#[cfg(not(windows))]
use jpeg_decoder::{Decoder as JpegDecoder, PixelFormat as JpegPixelFormat};
#[cfg(not(windows))]
use std::io::Cursor;

#[cfg(not(windows))]
fn image_from_cuimage<A>(cu_image: &CuImage<A>) -> ManuallyDrop<Image>
where
    A: ArrayLike<Element = u8>,
{
    unsafe {
        // Try to emulate what the C code is doing on the heap to avoid double free
        let buffer_ptr = cu_image.buffer_handle.with_inner(|inner| inner.as_ptr());
        let low_level_img = Box::new(image_u8_t {
            buf: buffer_ptr as *mut u8,
            width: cu_image.format.width as i32,
            height: cu_image.format.height as i32,
            stride: cu_image.format.stride as i32,
        });
        let ptr = Box::into_raw(low_level_img);
        ManuallyDrop::new(Image::from_raw(ptr))
    }
}

#[cfg(not(windows))]
fn image_from_raw_parts(ptr: *mut u8, width: u32, height: u32, stride: u32) -> ManuallyDrop<Image> {
    unsafe {
        let low_level_img = Box::new(image_u8_t {
            buf: ptr,
            width: width as i32,
            height: height as i32,
            stride: stride as i32,
        });
        let ptr = Box::into_raw(low_level_img);
        ManuallyDrop::new(Image::from_raw(ptr))
    }
}

impl Freezable for AprilTags {}

#[cfg(windows)]
impl<'cl> CuTask<'cl> for AprilTags {
    type Input = input_msg!('cl, CuImage<Vec<u8>>);
    type Output = output_msg!('cl, AprilTagDetections);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        _input: Self::Input,
        _output: Self::Output,
    ) -> CuResult<()> {
        Ok(())
    }
}

// pub type ImageWithId = (Box<String>, CuImage<Vec<u8>>);

#[cfg(not(windows))]
impl<'cl> CuTask<'cl> for AprilTags {
    // camera_id, image
    type Input = input_msg!('cl, CuImage<Vec<u8>>);
    type Output = output_msg!('cl, AprilTagDetections);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if let Some(config) = _config {
            let family_cfg: String = config.get("family").unwrap_or(FAMILY.to_string());
            let family: Family = family_cfg.parse().unwrap();
            let bits_corrected: u32 = config.get("bits_corrected").unwrap_or(1);
            let tagsize = config.get("tag_size").unwrap_or(TAG_SIZE);
            let fx = config.get("fx").unwrap_or(FX);
            let fy = config.get("fy").unwrap_or(FY);
            let cx = config.get("cx").unwrap_or(CX);
            let cy = config.get("cy").unwrap_or(CY);
            let tag_params = TagParams {
                fx,
                fy,
                cx,
                cy,
                tagsize,
            };

            let detector = DetectorBuilder::default()
                .add_family_bits(family, bits_corrected as usize)
                .build()
                .unwrap();
            let camera_id = if let Some(id) = config.get::<String>("camera_id") {
                Box::new(id)
            } else {
                Box::new(String::new())
            };
            return Ok(Self {
                detector,
                tag_params,
                scratch: Vec::new(),
                camera_id,
            });
        }
        Ok(Self {
            detector: DetectorBuilder::default()
                .add_family_bits(FAMILY.parse::<Family>().unwrap(), 1)
                .build()
                .unwrap(),
            tag_params: TagParams {
                fx: FX,
                fy: FY,
                cx: CX,
                cy: CY,
                tagsize: TAG_SIZE,
            },
            scratch: Vec::new(),
            camera_id: Box::new(String::new()),
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        let mut result = AprilTagDetections::new();
        if let Some(payload) = input.payload() {
            // Set camera ID either from config or leave empty
            result.camera_id = self.camera_id.clone();
            let payload = payload;
            // Fast grayscale conversion / extraction based on pixel format
            let pixel_format = std::str::from_utf8(&payload.format.pixel_format)
                .unwrap_or("")
                .trim_end_matches('\0');

            let image = match pixel_format {
                // Already 8-bit luma plane – no conversion needed.
                "GRAY" | "NV12" => image_from_cuimage(payload),

                // YUYV 4:2:2 – extract every other byte (Y channel)
                "YUYV" => {
                    let width = payload.format.width as usize;
                    let height = payload.format.height as usize;
                    let src_stride = payload.format.stride as usize;

                    let required = width * height;
                    if self.scratch.len() < required {
                        self.scratch.resize(required, 0);
                    }

                    payload.buffer_handle.with_inner(|raw_buffer| {
                        let src: &[u8] = raw_buffer;
                        for y in 0..height {
                            let src_row = &src[y * src_stride..y * src_stride + src_stride];
                            let dst_row = &mut self.scratch[y * width..(y + 1) * width];
                            for x in 0..width {
                                // Y is stored at even indices (U0 Y0 V0 Y1)
                                dst_row[x] = src_row[x * 2];
                            }
                        }
                    });

                    image_from_raw_parts(
                        self.scratch.as_mut_ptr(),
                        payload.format.width,
                        payload.format.height,
                        payload.format.width, // stride == width for luma image
                    )
                }
                // MJPEG / JPEG – decode compressed bitstream to grayscale.
                "MJPG" | "JPEG" | "JPG" => {
                    let jpeg_bytes = payload.buffer_handle.with_inner(|raw| raw.to_vec());

                    let mut decoder = JpegDecoder::new(Cursor::new(jpeg_bytes));
                    let pixels = match decoder.decode() {
                        Ok(p) => p,
                        Err(e) => {
                            println!(
                                "[cu_apriltag] JPEG decode failed: {e}, skipping detection."
                            );
                            output.metadata.tov = input.metadata.tov;
                            output.set_payload(result);
                            return Ok(());
                        }
                    };

                    let info = decoder.info().ok_or_else(|| {
                        CuError::from("jpeg-decoder missing info after decode")
                    })?;

                    let (width, height) = (info.width as usize, info.height as usize);

                    // Ensure scratch buffer size
                    let required = width * height;
                    if self.scratch.len() < required {
                        self.scratch.resize(required, 0);
                    }

                    match info.pixel_format {
                        JpegPixelFormat::L8 => {
                            self.scratch[..required].copy_from_slice(&pixels[..required]);
                        }
                        JpegPixelFormat::RGB24 => {
                            // Convert RGB to grayscale using BT.601 luma approximation
                            for i in 0..required {
                                let r = pixels[3 * i] as u32;
                                let g = pixels[3 * i + 1] as u32;
                                let b = pixels[3 * i + 2] as u32;
                                self.scratch[i] = ((r * 299 + g * 587 + b * 114) / 1000) as u8;
                            }
                        }
                        other => {
                            return Err(CuError::new_with_cause(
                                "Unsupported JPEG pixel format",
                                std::io::Error::new(
                                    std::io::ErrorKind::Other,
                                    format!("Unsupported JPEG pixel format: {other:?}"),
                                ),
                            ));
                        }
                    }

                    image_from_raw_parts(
                        self.scratch.as_mut_ptr(),
                        width as u32,
                        height as u32,
                        width as u32,
                    )
                }
                // Unsupported / unrecognised formats – skip detection gracefully.
                other => {
                    return Err(CuError::new_with_cause(
                        "Unsupported pixel format", 
                        std::io::Error::new(std::io::ErrorKind::Other, 
                        other))
                    );
                }
            };

            let detections = self.detector.detect(&image);
            for detection in detections {

                if let Some(aprilpose) = detection.estimate_tag_pose(&self.tag_params) {
                    let translation = aprilpose.translation();
                    let rotation = aprilpose.rotation();
                    let mut pose = CuPose::<f32>::default();
                    pose.mat[0][3] = translation.data()[0] as f32;
                    pose.mat[1][3] = translation.data()[1] as f32;
                    pose.mat[2][3] = translation.data()[2] as f32;
                    pose.mat[0][0] = rotation.data()[0] as f32;
                    pose.mat[0][1] = rotation.data()[3] as f32;
                    pose.mat[0][2] = rotation.data()[2 * 3] as f32;
                    pose.mat[1][0] = rotation.data()[1] as f32;
                    pose.mat[1][1] = rotation.data()[1 + 3] as f32;
                    pose.mat[1][2] = rotation.data()[1 + 2 * 3] as f32;
                    pose.mat[2][0] = rotation.data()[2] as f32;
                    pose.mat[2][1] = rotation.data()[2 + 3] as f32;
                    pose.mat[2][2] = rotation.data()[2 + 2 * 3] as f32;
                    let CuArrayVec(detections) = &mut result.poses;
                    detections.push(pose);
                    let CuArrayVec(decision_margin) = &mut result.decision_margins;
                    decision_margin.push(detection.decision_margin());
                    let CuArrayVec(ids) = &mut result.ids;
                    ids.push(detection.id());
                }
            }
        };
        output.metadata.tov = input.metadata.tov;
        output.set_payload(result);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)]
    use super::*;
    use anyhow::Context;
    use anyhow::Result;
    use image::{imageops::crop, imageops::resize, imageops::FilterType, Luma};
    use image::{ImageBuffer, ImageReader};

    #[cfg(not(windows))]
    use cu_sensor_payloads::CuImageBufferFormat;

    #[allow(dead_code)]
    fn process_image(path: &str) -> Result<ImageBuffer<Luma<u8>, Vec<u8>>> {
        let reader = ImageReader::open(path).with_context(|| "Failed to open image")?;
        let mut img = reader
            .decode()
            .context("Failed to decode image")?
            .into_luma8();
        let (orig_w, orig_h) = img.dimensions();

        let new_h = (orig_w as f32 * 9.0 / 16.0) as u32;
        let crop_y = (orig_h - new_h) / 2; // Center crop

        let cropped = crop(&mut img, 0, crop_y, orig_w, new_h).to_image();
        Ok(resize(&cropped, 1920, 1080, FilterType::Lanczos3))
    }

    #[test]
    #[cfg(not(windows))]
    fn test_end2end_apriltag() -> Result<()> {
        let img = process_image("tests/data/simple.jpg")?;
        let format = CuImageBufferFormat {
            width: img.width(),
            height: img.height(),
            stride: img.width(),
            pixel_format: "GRAY".as_bytes().try_into()?,
        };
        let buffer_handle = CuHandle::new_detached(img.into_raw());
        let cuimage = CuImage::new(format, buffer_handle);

        let mut config = ComponentConfig::default();
        config.set("tag_size", 0.14);
        config.set("fx", 2600.0);
        config.set("fy", 2600.0);
        config.set("cx", 900.0);
        config.set("cy", 520.0);
        config.set("family", "tag16h5".to_string());

        let mut task = AprilTags::new(Some(&config))?;
        let input = CuMsg::<CuImage<Vec<u8>>>::new(Some(cuimage));
        let mut output = CuMsg::<AprilTagDetections>::default();

        let clock = RobotClock::new();
        let result = task.process(&clock, &input, &mut output);
        assert!(result.is_ok());

        if let Some(detections) = output.payload() {
            let detections = detections
                .filtered_by_decision_margin(150.0)
                .collect::<Vec<_>>();
            assert_eq!(detections.len(), 1);
            assert_eq!(detections[0].0, 4);
            return Ok(());
        }
        Err(anyhow::anyhow!("No output"))
    }
}
