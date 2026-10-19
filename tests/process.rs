//! encoders::process: a child that hangs is killed and replaced, and the encoder goes on from a keyframe.

use std::sync::Arc;
use std::time::{Duration, Instant};
use zenoh_dimos_codecs::encoders::process::{ProcessConfig, ProcessEncoder, serve_if_child};
use zenoh_gateway::{DecodedFrame, EncodedVideo, H264Encoder, VideoEncoder, VideoFormat, VideoImage, VideoTarget};

/// The built-in H.264 encoder, except that once per `ZDC_TEST_HANG_DIR` the third frame hangs forever (as a wedged
/// hardware encoder), after writing this process's pid to `hung.pid` there.
struct HangsOnce {
    inner: H264Encoder,
    frames: u32,
}

impl VideoEncoder for HangsOnce {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> anyhow::Result<Option<EncodedVideo>> {
        self.frames += 1;
        if self.frames == 3 && let Ok(dir) = std::env::var("ZDC_TEST_HANG_DIR") {
            let marker = std::path::Path::new(&dir).join("hung.pid");
            if std::fs::OpenOptions::new().write(true).create_new(true).open(&marker).is_ok() {
                std::fs::write(&marker, std::process::id().to_string()).unwrap();
                loop {
                    std::thread::sleep(Duration::from_secs(60));
                }
            }
        }
        self.inner.encode(frame, target)
    }
}

/// The child side: this test binary, run with `--exact video_encoder_child`, serves encodes; in a normal run it returns.
#[test]
fn video_encoder_child() {
    serve_if_child(&|| Box::new(HangsOnce { inner: H264Encoder::default(), frames: 0 }));
}

fn frame() -> DecodedFrame {
    DecodedFrame::Video(VideoImage::rgb8(320, 240, [10u8, 200, 30].repeat(320 * 240)).unwrap())
}

/// Encodes until a frame comes out (a starting child answers nothing), at most `within`.
fn next_frame(encoder: &mut ProcessEncoder, within: Duration) -> EncodedVideo {
    let started = Instant::now();
    let target = VideoTarget::new(320, 240, 500_000, 20.0);
    loop {
        assert!(started.elapsed() < within, "no frame within {within:?}");
        if let Ok(Some(encoded)) = encoder.encode(&frame(), &target) {
            return encoded;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn a_hung_child_is_killed_and_replaced_and_encoding_goes_on() {
    let dir = std::env::temp_dir().join(format!("zdc-hang-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let marker = dir.join("hung.pid");
    let _ = std::fs::remove_file(&marker);
    let child_dir = dir.clone();
    let mut config = ProcessConfig::new(Duration::from_millis(500));
    config.command = Arc::new(move || {
        let mut command = std::process::Command::new(std::env::current_exe().unwrap());
        command.args(["--exact", "video_encoder_child", "--nocapture", "--test-threads", "1"]).env("ZDC_TEST_HANG_DIR", &child_dir);
        command
    });
    let mut encoder = ProcessEncoder::new(config, VideoFormat::H264);
    let first = next_frame(&mut encoder, Duration::from_secs(20));
    assert!(first.keyframe && (first.width, first.height) == (320, 240));
    next_frame(&mut encoder, Duration::from_secs(5));
    // the third frame hangs: the encode fails after the deadline, and the child is replaced
    let target = VideoTarget::new(320, 240, 500_000, 20.0);
    let asked = Instant::now();
    let error = encoder.encode(&frame(), &target).expect_err("the hung encode fails").to_string();
    assert!(error.contains("hung"), "{error}");
    assert!(asked.elapsed() < Duration::from_secs(2), "given up at the deadline, not later: {:?}", asked.elapsed());
    assert_eq!(encoder.starts, 2);
    let hung_pid = std::fs::read_to_string(&marker).unwrap();
    // the new child encodes again, from a keyframe
    let resumed = next_frame(&mut encoder, Duration::from_secs(20));
    assert!(resumed.keyframe, "a new child starts at a keyframe");
    let gone = Instant::now();
    while std::process::Command::new("kill").args(["-0", hung_pid.trim()]).status().unwrap().success() {
        assert!(gone.elapsed() < Duration::from_secs(5), "the hung child {hung_pid} was killed");
        std::thread::sleep(Duration::from_millis(50));
    }
    drop(encoder);
    let _ = std::fs::remove_dir_all(&dir);
}
