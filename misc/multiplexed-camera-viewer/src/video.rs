use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use egui::{ColorImage, Vec2};
use ffmpeg_next as ffmpeg;
use rtp::packet::Packet;
use webrtc_util::marshal::Unmarshal;

pub type SharedFrame = Arc<Mutex<Option<ColorImage>>>;

/// spawns a background thread that listens for RTP/H.264 on the given UDP address.
/// exits when `active` is set to false.
pub fn spawn_receiver(address: &str, frame: SharedFrame, active: Arc<AtomicBool>, ctx: egui::Context) {
    let address = address.to_string();
    thread::spawn(move || {
        ffmpeg::init().expect("Failed to init ffmpeg");
        let socket = match UdpSocket::bind(&address) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to bind UDP socket on {address}: {e}");
                return;
            }
        };
        socket
            .set_read_timeout(Some(Duration::from_millis(100)))
            .ok();

        let codec = ffmpeg::codec::decoder::find(ffmpeg::codec::Id::H264)
            .expect("Failed to find H.264 decoder");
        
        let mut decoder = ffmpeg::codec::Context::new()
            .decoder()
            .open_as(codec)
            .expect("Failed to open H.264 decoder")
            .video()
            .expect("Failed to get video decoder");

        let mut buf = [0u8; 65535];
        let mut nal_buffer: Vec<u8> = Vec::new();
        let mut fu_a_started = false;
        let mut last_seq: Option<u16> = None;
        
        let mut last_frame_time = Instant::now();
        let mut frames_decoded = 0u64;
        
        let mut decoded_frame = ffmpeg::frame::Video::empty();

        while active.load(Ordering::Relaxed) {
            let n = match socket.recv(&mut buf) {
                Ok(n) => n,
                Err(_) => continue,
            };

            let packet = match Packet::unmarshal(&mut &buf[..n]) {
                Ok(p) => p,
                Err(_) => continue,
            };

            if let Some(prev) = last_seq {
                let expected = prev.wrapping_add(1);
                if packet.header.sequence_number != expected {
                    fu_a_started = false;
                    nal_buffer.clear();
                }
            }
            last_seq = Some(packet.header.sequence_number);

            let payload = &packet.payload;
            if payload.is_empty() {
                continue;
            }

            let nal_type = payload[0] & 0x1F;

            // bruh
            match nal_type {
                1..=23 => {
                    let mut nal = Vec::with_capacity(4 + payload.len());
                    nal.extend_from_slice(&[0, 0, 0, 1]);
                    nal.extend_from_slice(payload);
                    if try_decode(
                        &mut decoder,
                        &nal,
                        &frame,
                        &mut last_frame_time,
                        &mut frames_decoded,
                        &mut decoded_frame,
                    ) {
                        ctx.request_repaint();
                    }
                }
                28 => {
                    if payload.len() < 2 {
                        continue;
                    }
                    let fu_header = payload[1];
                    let start = fu_header & 0x80 != 0;
                    let end = fu_header & 0x40 != 0;
                    let nal_type_inner = fu_header & 0x1F;
                    let nri = payload[0] & 0x60;

                    if start {
                        nal_buffer.clear();
                        nal_buffer.extend_from_slice(&[0, 0, 0, 1]);
                        nal_buffer.push(nri | nal_type_inner);
                        nal_buffer.extend_from_slice(&payload[2..]);
                        fu_a_started = true;
                    } else if fu_a_started {
                        nal_buffer.extend_from_slice(&payload[2..]);
                        if end {
                            fu_a_started = false;
                            if try_decode(
                                &mut decoder,
                                &nal_buffer,
                                &frame,
                                &mut last_frame_time,
                                &mut frames_decoded,
                                &mut decoded_frame,
                            ) {
                                ctx.request_repaint();
                            }
                        }
                    }
                }
                24 => {
                    let mut offset = 1;
                    while offset + 2 <= payload.len() {
                        let size =
                            ((payload[offset] as usize) << 8) | (payload[offset + 1] as usize);
                        offset += 2;
                        if offset + size > payload.len() {
                            break;
                        }
                        let mut nal = Vec::with_capacity(4 + size);
                        nal.extend_from_slice(&[0, 0, 0, 1]);
                        nal.extend_from_slice(&payload[offset..offset + size]);
                        if try_decode(
                            &mut decoder,
                            &nal,
                            &frame,
                            &mut last_frame_time,
                            &mut frames_decoded,
                            &mut decoded_frame,
                        ) {
                            ctx.request_repaint();
                        }
                        offset += size;
                    }
                }
                _ => {}
            }
        }        
    });
}

const MAX_TEX_WIDTH: usize = 426*2;
const MAX_TEX_HEIGHT: usize = 240*2;

fn try_decode(
    decoder: &mut ffmpeg::decoder::Video,
    nal: &[u8],
    frame: &SharedFrame,
    last_frame_time: &mut Instant,
    frames_decoded: &mut u64,
    decoded_frame: &mut ffmpeg::frame::Video,
) -> bool {

    let packet = ffmpeg::Packet::copy(nal);
    
    if decoder.send_packet(&packet).is_err() {
        return false;
    }

    if decoder.receive_frame(decoded_frame).is_err() {
        return false;
    }

    let src_w = decoded_frame.width() as usize;
    let src_h = decoded_frame.height() as usize;

    let mut scaler = ffmpeg::software::scaling::Context::get(
        decoded_frame.format(),
        src_w as u32,
        src_h as u32,
        ffmpeg::format::Pixel::RGB24,
        src_w as u32,
        src_h as u32,
        ffmpeg::software::scaling::Flags::BILINEAR,
    ).expect("Failed to create scaler");

    let mut rgb_frame = ffmpeg::frame::Video::empty();
    scaler.run(decoded_frame, &mut rgb_frame).expect("Failed to scale");

    let (dst_w, dst_h, pixels) = if src_w > MAX_TEX_WIDTH || src_h > MAX_TEX_HEIGHT {
        let scale = (MAX_TEX_WIDTH as f32 / src_w as f32)
            .min(MAX_TEX_HEIGHT as f32 / src_h as f32);
        let dst_w = (src_w as f32 * scale) as usize;
        let dst_h = (src_h as f32 * scale) as usize;
        
        let mut downscaled = Vec::with_capacity(dst_w * dst_h);
        let rgb_data = rgb_frame.data(0);
        let stride = rgb_frame.stride(0);
        
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
        let rgb_data = rgb_frame.data(0);
        let stride = rgb_frame.stride(0);
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

    let image = ColorImage {
        size: [dst_w, dst_h],
        source_size: Vec2::new(src_w as f32, src_h as f32),
        pixels,
    };

    if let Ok(mut lock) = frame.lock() {
        *lock = Some(image);
    }
    
    *frames_decoded += 1;

    *last_frame_time = Instant::now();
    return true;
}