//! `sensor_msgs/CompressedImage` from ROS 2 (rmw_zenoh, CDR) as video: a JPEG, PNG, WebP or JPEG XL file to a picture for the
//! video encoder; on the data channel, the file itself or converted (`encodeOptions.format`).

use super::CompressedImage;
use crate::utils::compressed_image::ImageFile;
use crate::utils::{cdr::Cdr, compressed_image};
use anyhow::Result;
use serde_json::{Map, Value};
use zenoh_web::{Channel, DecodedFrame, EncodeOptions, EncodingOutput, EncodingSample, MessageEncoding};

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

impl MessageEncoding for Ros2CompressedImage {
    fn name(&self) -> &str {
        "ros2_compressed_image"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Video
    }

    /// Video channels: no options. Data: `{format}`, `"passthrough"` (default, the file as sent), `"jpeg"` or `"png"`.
    fn output_on(&self, channel: Channel, options: &Map<String, Value>) -> Result<EncodingOutput, String> {
        compressed_image::output_on(self.name(), channel, options, true)
    }

    fn decode(&self, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
        let message = parse(sample.payload)?;
        Ok(match channel {
            Channel::Data => DecodedFrame::data(ImageFile::new(message.data, &message.format)?),
            _ => DecodedFrame::Video(compressed_image::to_video(message.data, &message.format)?),
        })
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        frame.downcast::<ImageFile>()?.encode(options)
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        compressed_image::estimated_bytes(payload_bytes, options, true)
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
