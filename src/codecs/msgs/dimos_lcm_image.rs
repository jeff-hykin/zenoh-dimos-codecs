//! `sensor_msgs/Image` from dimos (LCM over zenoh) as video: raw pixels (rgb8, bgr8, mono16, ...) or a file in the data field, to a
//! picture for the video encoder; on the data channel, a JPEG or PNG of it (`encodeOptions.format`).

use super::RawImage;
use crate::utils::compressed_image;
use crate::utils::{lcm::Lcm, raw_pixels};
use anyhow::Result;
use serde_json::{Map, Value};
use zenoh_web::{Channel, DecodedFrame, EncodeOptions, EncodingOutput, EncodingSample, MessageEncoding};

/// The LCM fingerprint of dimos-lcm's type, as `lcm-gen` computes it.
const LCM_IMAGE: [u8; 8] = [0x53, 0x5c, 0xfa, 0xce, 0x1f, 0x4f, 0x57, 0x17];

/// `dimos_lcm_image`
pub struct DimosLcmImage;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<RawImage<'_>> {
    let mut lcm = Lcm::new(payload, LCM_IMAGE, "sensor_msgs.Image")?;
    let data_length = lcm.length()?;
    lcm.skip_header()?;
    let height = lcm.i32()? as u32;
    let width = lcm.i32()? as u32;
    let encoding = lcm.string()?;
    let big_endian = lcm.u8()? != 0;
    let step = lcm.i32()? as u32;
    let data = lcm.take(data_length)?;
    Ok(RawImage { width, height, encoding, big_endian, step, data })
}

impl MessageEncoding for DimosLcmImage {
    fn name(&self) -> &str {
        "dimos_lcm_image"
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
        let payload = fixture("dimos/image_rgb8.bin");
        let image = parse(&payload).unwrap();
        assert_eq!((image.width, image.height, image.step), (320, 240, 960));
        assert_eq!(image.encoding, "rgb8");
        assert!(!image.big_endian);
        assert_eq!(image.data.len(), 320 * 240 * 3);
        assert_eq!(&image.data[..3], &[255, 0, 0]);
    }

    #[test]
    fn another_type_is_an_error() {
        assert!(parse(&fixture("dimos/compressed_png.bin")).is_err(), "fingerprint mismatch");
    }
}
