//! `sensor_msgs/Image` from ROS 2 (rmw_zenoh, CDR) as video: raw pixels (rgb8, bgr8, mono16, ...) or a file in the data field, to a
//! picture for the video encoder.

use super::RawImage;
use crate::utils::{cdr::Cdr, raw_pixels};
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, DecodedFrame};

/// `ros2_image`
pub struct Ros2Image;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<RawImage<'_>> {
    let mut cdr = Cdr::new(payload)?;
    cdr.skip_header()?;
    let height = cdr.u32()?;
    let width = cdr.u32()?;
    let encoding = cdr.string()?;
    let big_endian = cdr.u8()? != 0;
    let step = cdr.u32()?;
    let data = cdr.byte_sequence()?;
    Ok(RawImage { width, height, encoding, big_endian, step, data })
}

impl Codec for Ros2Image {
    fn name(&self) -> &str {
        "ros2_image"
    }

    fn output(&self) -> CodecOutput {
        CodecOutput::Video
    }

    fn decode(&self, sample: &CodecSample<'_>) -> Result<DecodedFrame> {
        Ok(DecodedFrame::Video(raw_pixels::to_video(&parse(sample.payload)?)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("ros2/image_rgb8.cdr");
        let image = parse(&payload).unwrap();
        assert_eq!((image.width, image.height, image.step), (320, 240, 960));
        assert_eq!(image.encoding, "rgb8");
        assert!(!image.big_endian);
        assert_eq!(image.data.len(), 320 * 240 * 3);
        assert_eq!(&image.data[..3], &[255, 0, 0]);
    }

    #[test]
    fn another_type_is_an_error() {
        assert!(parse(&fixture("ros2/compressed_png.cdr")).is_err(), "truncated");
    }
}
