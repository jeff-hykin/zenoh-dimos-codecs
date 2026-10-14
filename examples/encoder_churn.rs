//! Rebuilds an encoder's pipeline over and over and prints the process's memory, to tell a leak per pipeline (memory
//! climbs with rebuilds) from one per frame (climbs with frames at a fixed size).
//! `cargo run --release --features gstreamer --example encoder_churn -- BACKEND [ROUNDS] [FRAMES_PER_ROUND] [MODE]`
//! BACKEND: software, gstreamer (the first hardware element that works), or a GStreamer element name.
//! MODE: `rebuild` (default: each round a new size, so a new pipeline), `fresh` (a new encoder each round, same size),
//! or `steady` (one size, one encoder: frames only).

use anyhow::Result;
use zenoh_gateway::{DecodedFrame, VideoEncoder, VideoImage, VideoTarget};

fn rss_mb() -> f64 {
    let status = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    let kb = status.lines().find_map(|line| line.strip_prefix("VmRSS:")).and_then(|value| value.trim().trim_end_matches("kB").trim().parse::<f64>().ok());
    kb.map_or(f64::NAN, |kb| kb / 1024.0)
}

fn encoder(backend: &str) -> Result<Box<dyn VideoEncoder>> {
    Ok(match backend {
        "software" => Box::new(zenoh_gateway::H264Encoder::default()),
        #[cfg(feature = "gstreamer")]
        "gstreamer" => Box::new(zenoh_dimos_codecs::encoders::gstreamer::GstreamerEncoder::hardware()?),
        #[cfg(feature = "gstreamer")]
        element => Box::new(zenoh_dimos_codecs::encoders::gstreamer::GstreamerEncoder::with_element(element)?),
        #[cfg(not(feature = "gstreamer"))]
        other => anyhow::bail!("{other} is not built in"),
    })
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let backend = args.first().map(String::as_str).unwrap_or("gstreamer");
    let rounds: usize = args.get(1).map(|value| value.parse()).transpose()?.unwrap_or(200);
    let frames: usize = args.get(2).map(|value| value.parse()).transpose()?.unwrap_or(10);
    let mode = args.get(3).map(String::as_str).unwrap_or("rebuild");
    // the gateway's sizes move in steps like these as grants change
    let sizes = [(640u32, 480u32), (480, 360), (320, 240), (960, 720)];
    let mut current = encoder(backend)?;
    println!("{backend} {mode}: {rounds} rounds x {frames} frames; rss {:.1} MB", rss_mb());
    for round in 0..rounds {
        let (width, height) = if mode == "rebuild" { sizes[round % sizes.len()] } else { sizes[0] };
        if mode == "fresh" {
            current = encoder(backend)?;
        }
        let image = VideoImage::i420(width, height, vec![(round % 200) as u8 + 20; (width * height * 3 / 2) as usize])?;
        for frame in 0..frames {
            let mut target = VideoTarget::new(width, height, 1_000_000, 30.0);
            target.keyframe = frame == 0;
            let _ = current.encode(&DecodedFrame::Video(image.clone()), &target)?;
        }
        if (round + 1) % 20 == 0 {
            println!("round {:>4}: rss {:.1} MB", round + 1, rss_mb());
        }
    }
    Ok(())
}
