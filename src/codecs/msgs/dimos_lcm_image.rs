//! `sensor_msgs/Image` from dimos (LCM over zenoh) as video: raw pixels (rgb8, bgr8, mono16, ...) or a file in the data field, to a
//! picture for the video encoder.

use super::RawImage;
use crate::utils::{lcm::Lcm, raw_pixels};
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, DecodedFrame};

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

impl Codec for DimosLcmImage {
    fn name(&self) -> &str {
        "dimos_lcm_image"
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
