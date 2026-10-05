//! `sensor_msgs/CompressedImage` from dimos (LCM over zenoh) as video: a JPEG, PNG, WebP or JPEG XL file to a picture for the
//! video encoder.

use super::CompressedImage;
use crate::utils::{lcm::Lcm, compressed_image};
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, DecodedFrame};

/// The LCM fingerprint of dimos-lcm's type, as `lcm-gen` computes it.
const LCM_COMPRESSED_IMAGE: [u8; 8] = [0xb8, 0xd0, 0x11, 0xc1, 0x04, 0x12, 0xb9, 0xa1];

/// `dimos_lcm_compressed_image`
pub struct DimosLcmCompressedImage;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<CompressedImage<'_>> {
    let mut lcm = Lcm::new(payload, LCM_COMPRESSED_IMAGE, "sensor_msgs.CompressedImage")?;
    let data_length = lcm.length()?;
    lcm.skip_header()?;
    let format = lcm.string()?;
    let data = lcm.take(data_length)?;
    Ok(CompressedImage { format, data })
}

impl Codec for DimosLcmCompressedImage {
    fn name(&self) -> &str {
        "dimos_lcm_compressed_image"
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
        let payload = fixture("dimos/compressed_png.bin");
        let image = parse(&payload).unwrap();
        assert!(image.format.contains("png"), "{}", image.format);
        assert_eq!(image.data, fixture("compressed/pattern.png").as_slice());
    }
}
