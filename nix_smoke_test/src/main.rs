//! `zenoh-dimos-codecs-example [auto|software|videotoolbox|gstreamer] [frames]`: lists the codecs, starts and stops a
//! zenoh-web server with all of them on 127.0.0.1 (isolated zenoh session: no scouting, no listeners), then selects a
//! video encoder (probing hardware ones by encoding a test frame) and times it on `frames` (default 60) of a moving
//! 1280x720 test pattern.

use anyhow::Result;
use std::time::Instant;
use zenoh_dimos_codecs::encoders::{Backend, select};
use zenoh_web::{DecodedFrame, H264Encoder, Server, VideoEncoder, VideoImage, VideoTarget, zenoh};

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|argument| argument == "-h" || argument == "--help") {
        println!("usage: zenoh-dimos-codecs-example [auto|software|videotoolbox|gstreamer] [frames]");
        return Ok(());
    }
    let backend: Backend = args.first().map(|name| name.parse()).transpose().map_err(anyhow::Error::msg)?.unwrap_or(Backend::Auto);
    let frames: usize = args.get(1).map(|count| count.parse()).transpose()?.unwrap_or(60);

    let codecs = zenoh_dimos_codecs::all();
    println!("codecs: {:?}", codecs.iter().map(|codec| codec.name().to_owned()).collect::<Vec<_>>());
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false").map_err(anyhow::Error::msg)?;
    config.insert_json5("listen/endpoints", "[]").map_err(anyhow::Error::msg)?;
    let mut builder = Server::builder().zenoh_config(config);
    for codec in codecs {
        builder = builder.shared_codec(codec);
    }
    let running = builder.build().await?.bind("127.0.0.1:0").await?;
    println!("serving on http://{}", running.local_addr());
    running.shutdown().await?;

    // this crate enables both encoder features; VideoToolbox only exists on macOS
    println!("encoders compiled in: {:?}", [("videotoolbox", cfg!(target_os = "macos")), ("gstreamer", true)].iter().filter(|(_, present)| *present).map(|(name, _)| name).collect::<Vec<_>>());
    let selected = select(backend)?;
    println!("selected encoder: {}", selected.name);
    let mut encoder: Box<dyn VideoEncoder> = match &selected.factory {
        Some(factory) => factory(),
        None => Box::new(H264Encoder::default()),
    };
    let (width, height, fps) = (1280u32, 720u32, 30.0);
    let (mut total_ms, mut total_bytes) = (0.0, 0usize);
    for index in 0..frames {
        let pixels = (0..height).flat_map(|y| (0..width).flat_map(move |x| {
            let (u, v) = ((x + index as u32 * 4) % 512, y % 512);
            [(u / 2) as u8, (v / 2) as u8, ((u + v) / 4) as u8]
        })).collect();
        let frame = DecodedFrame::Video(VideoImage::rgb8(width, height, pixels)?);
        let started = Instant::now();
        let encoded = encoder.encode(&frame, &VideoTarget::new(width, height, 4_000_000, fps))?;
        total_ms += started.elapsed().as_secs_f64() * 1000.0;
        total_bytes += encoded.map_or(0, |encoded| encoded.data.len());
    }
    println!("{frames} frames {width}x{height}: {:.2} ms/frame, {total_bytes} bytes", total_ms / frames.max(1) as f64);
    println!("ok");
    Ok(())
}
