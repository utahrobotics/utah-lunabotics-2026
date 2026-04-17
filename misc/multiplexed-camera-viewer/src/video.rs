use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Instant;

use egui::{ColorImage, Vec2};
use ffmpeg_next as ffmpeg;

pub type SharedFrame = Arc<Mutex<Option<ColorImage>>>;

/// Spawns a background thread that connects to a tcpserversink serving MPEG-TS/H.264.
/// Exits when `active` is set to false.
pub fn spawn_receiver(address: &str, frame: SharedFrame, active: Arc<AtomicBool>, ctx: egui::Context) {
    let address = address.to_string();
    thread::spawn(move || {
        ffmpeg::init().expect("Failed to init ffmpeg");

        let url = format!("tcp://{address}");
        let mut opts = ffmpeg::Dictionary::new();
        opts.set("timeout", "1000000");

        let mut input_ctx = loop {
            if !active.load(Ordering::Relaxed) {
                return;
            }
            match ffmpeg::format::input_with_dictionary(&url, opts.clone()) {
                Ok(ctx) => break ctx,
                Err(_) => {
                    thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                }
            }
        };

        let video_stream_index = match input_ctx
            .streams()
            .best(ffmpeg::media::Type::Video)
        {
            Some(s) => s.index(),
            None => {
                eprintln!("No video stream found in {address}");
                return;
            }
        };

        let codec_params = input_ctx.stream(video_stream_index).unwrap().parameters();
        let mut decoder = ffmpeg::codec::Context::from_parameters(codec_params)
            .and_then(|c| c.decoder().video())
            .expect("Failed to open H.264 decoder");

        let mut decoded_frame = ffmpeg::frame::Video::empty();
        let mut scaler: Option<ffmpeg::software::scaling::Context> = None;
        let mut _last_frame_time = Instant::now();
        let mut _frames_decoded = 0u64;

        for (stream, packet) in input_ctx.packets() {
            if !active.load(Ordering::Relaxed) {
                break;
            }

            if stream.index() != video_stream_index {
                continue;
            }

            if decoder.send_packet(&packet).is_err() {
                continue;
            }

            while decoder.receive_frame(&mut decoded_frame).is_ok() {
                let src_w = decoded_frame.width();
                let src_h = decoded_frame.height();

                let scaler = scaler.get_or_insert_with(|| {
                    ffmpeg::software::scaling::Context::get(
                        decoded_frame.format(),
                        src_w,
                        src_h,
                        ffmpeg::format::Pixel::RGB24,
                        src_w,
                        src_h,
                        ffmpeg::software::scaling::Flags::BILINEAR,
                    )
                    .expect("Failed to create scaler")
                });

                let mut rgb_frame = ffmpeg::frame::Video::empty();
                scaler.run(&decoded_frame, &mut rgb_frame).expect("Failed to scale");

                let image = frame_to_color_image(&rgb_frame, src_w as usize, src_h as usize);
                if let Ok(mut lock) = frame.lock() {
                    *lock = Some(image);
                }
                _frames_decoded += 1;
                _last_frame_time = Instant::now();
                ctx.request_repaint();
            }
        }
    });
}

const MAX_TEX_WIDTH: usize = 426 * 3;
const MAX_TEX_HEIGHT: usize = 240 * 3;

fn frame_to_color_image(rgb_frame: &ffmpeg::frame::Video, src_w: usize, src_h: usize) -> ColorImage {
    let rgb_data = rgb_frame.data(0);
    let stride = rgb_frame.stride(0);

    let (dst_w, dst_h, pixels) = if src_w > MAX_TEX_WIDTH || src_h > MAX_TEX_HEIGHT {
        let scale = (MAX_TEX_WIDTH as f32 / src_w as f32)
            .min(MAX_TEX_HEIGHT as f32 / src_h as f32);
        let dst_w = (src_w as f32 * scale) as usize;
        let dst_h = (src_h as f32 * scale) as usize;

        let mut downscaled = Vec::with_capacity(dst_w * dst_h);
        for y in 0..dst_h {
            let sy = y * src_h / dst_h;
            for x in 0..dst_w {
                let sx = x * src_w / dst_w;
                let i = sy * stride + sx * 3;
                downscaled.push(egui::Color32::from_rgb(
                    rgb_data[i],
                    rgb_data[i + 1],
                    rgb_data[i + 2],
                ));
            }
        }
        (dst_w, dst_h, downscaled)
    } else {
        let mut pixels = Vec::with_capacity(src_w * src_h);
        for y in 0..src_h {
            for x in 0..src_w {
                let i = y * stride + x * 3;
                pixels.push(egui::Color32::from_rgb(
                    rgb_data[i],
                    rgb_data[i + 1],
                    rgb_data[i + 2],
                ));
            }
        }
        (src_w, src_h, pixels)
    };

    ColorImage {
        size: [dst_w, dst_h],
        source_size: Vec2::new(src_w as f32, src_h as f32),
        pixels,
    }
}
