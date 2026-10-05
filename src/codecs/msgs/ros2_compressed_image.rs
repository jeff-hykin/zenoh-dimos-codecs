//! `sensor_msgs/CompressedImage` from ROS 2 (rmw_zenoh, CDR) as video: a JPEG, PNG, WebP or JPEG XL file to a picture for the
//! video encoder.

use super::CompressedImage;
use crate::utils::{cdr::Cdr, compressed_image};
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, DecodedFrame};

/// `ros2_compressed_image`
pub struct Ros2CompressedImage;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<CompressedImage<'_>> {
    let mut cdr = Cdr::new(payload)?;
    cdr.skip_header()?;
    let format = cdr.string()?;
    let data = cdr.byte_sequence()?;
    Ok(CompressedImage { format, data })
}

impl Codec for Ros2CompressedImage {
    fn name(&self) -> &str {
        "ros2_compressed_image"
    }

    fn output(&self) -> CodecOutput {
        CodecOutput::Video
    }

    fn decode(&self, sample: &CodecSample<'_>) -> Result<DecodedFrame> {
        let message = parse(sample.payload)?;
        Ok(DecodedFrame::Video(compressed_image::to_video(message.data, &message.format)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("ros2/compressed_png.cdr");
        let image = parse(&payload).unwrap();
        assert!(image.format.contains("png"), "{}", image.format);
        assert_eq!(image.data, fixture("compressed/pattern.png").as_slice());
    }
}
