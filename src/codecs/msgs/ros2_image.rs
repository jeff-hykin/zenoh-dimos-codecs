//! `sensor_msgs/Image` from ROS 2 (rmw_zenoh, CDR) as video: raw pixels (rgb8, bgr8, mono16, ...) or a file in the data field, to a
//! picture for the video encoder; on the data channel, a JPEG or PNG of it (`encodeOptions.format`).

use super::RawImage;
use crate::utils::compressed_image;
use crate::utils::{cdr::Cdr, raw_pixels};
use anyhow::Result;
use serde_json::{Map, Value};
use zenoh_gateway::{Channel, DecodedFrame, EncodeOptions, EncodingOutput, EncodingSample, MessageEncoding};

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

impl MessageEncoding for Ros2Image {
    fn name(&self) -> &str {
        "ros2_image"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Video
    }

    /// Video channels: no options. Data: `{format}`, `"jpeg"` (default, at the allocator's quality) or `"png"`.
    fn output_on(&self, channel: Channel, options: &Map<String, Value>) -> Result<EncodingOutput, String> {
        compressed_image::output_on(self.name(), channel, options, false)
    }

    fn decode(&self, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
        let image = parse(sample.payload)?;
        Ok(match channel {
            Channel::Data => DecodedFrame::data(raw_pixels::to_rgb(&image)?),
            _ => DecodedFrame::Video(raw_pixels::to_video(&image)?),
        })
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        compressed_image::encode_rgb(frame.downcast::<raw_pixels::Rgb8>()?, options)
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        // a JPEG of raw pixels is a tenth of them or less
        compressed_image::estimated_bytes(payload_bytes, options, false) * 0.1
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
