//! Times a JPEG through the hardware encoder at a camera's rate, directly and through the encoder child, at full size and
//! scaled: the per-call cost zenoh-gateway's CPU governor sees, and frames in vs out.
//! `cargo run --release --features gstreamer --example transcode_bench -- FILE.jpg [FRAMES] [FPS]`

use anyhow::{Context, Result};
use std::time::{Duration, Instant};
use zenoh_dimos_codecs::encoders::process::{ProcessConfig, ProcessEncoder, serve_if_child};
use zenoh_dimos_codecs::utils::compressed_image::decode_jpeg;
use zenoh_gateway::{DecodedFrame, VideoEncoder, VideoFormat, VideoImage, VideoTarget};

fn hardware() -> Box<dyn VideoEncoder> {
    Box::new(zenoh_dimos_codecs::encoders::gstreamer::GstreamerEncoder::with_element("nvv4l2h264enc").expect("nvv4l2h264enc"))
}

fn run(name: &str, encoder: &mut dyn VideoEncoder, frame: &DecodedFrame, size: (u32, u32), frames: usize, fps: f64) -> Result<()> {
    let interval = Duration::from_secs_f64(1.0 / fps);
    let (mut calls, mut out) = (Vec::new(), 0);
    let start = Instant::now();
    for index in 0..frames {
        let due = start + interval * index as u32;
        if let Some(wait) = due.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
        let mut target = VideoTarget::new(size.0, size.1, 8_000_000, fps);
        target.keyframe = index == 0;
        let began = Instant::now();
        out += encoder.encode(frame, &target)?.is_some() as usize;
        calls.push(began.elapsed().as_secs_f64() * 1000.0);
    }
    let skip = calls.len() / 5;
    let mut steady = calls[skip..].to_vec();
    steady.sort_by(f64::total_cmp);
    let at = |share: f64| steady[((steady.len() - 1) as f64 * share) as usize];
    println!("{name:<28} {}x{}: call p50 {:.1} p90 {:.1} max {:.1} ms; {out}/{frames} out; {:.1} s", size.0, size.1, at(0.5), at(0.9), at(1.0), start.elapsed().as_secs_f64());
    Ok(())
}

fn main() -> Result<()> {
    serve_if_child(&hardware);
    let args: Vec<String> = std::env::args().skip(1).collect();
    let bytes = std::fs::read(args.first().context("a JPEG file")?)?;
    let frames: usize = args.get(1).map(|value| value.parse()).transpose()?.unwrap_or(150);
    let fps: f64 = args.get(2).map(|value| value.parse()).transpose()?.unwrap_or(30.0);
    let decoded = decode_jpeg(&bytes)?;
    let source = (decoded.width(), decoded.height());
    let jpeg = DecodedFrame::Video(VideoImage::jpeg(source.0, source.1, bytes, decode_jpeg)?);
    let i420 = DecodedFrame::Video(decoded.to_i420(source.0, source.1)?);
    let half = ((source.0 / 2) & !15, (source.1 / 2) & !1);
    for (name, frame) in [("jpeg", &jpeg), ("i420", &i420)] {
        for size in [source, half] {
            run(&format!("{name} direct"), &mut *hardware(), frame, size, frames, fps)?;
            let mut child = ProcessEncoder::new(ProcessConfig::new(Duration::from_secs(3)), VideoFormat::H264);
            // (frames before the child says it is ready are skipped)
            std::thread::sleep(Duration::from_secs(3));
            run(&format!("{name} child"), &mut child, frame, size, frames, fps)?;
        }
    }
    Ok(())
}
