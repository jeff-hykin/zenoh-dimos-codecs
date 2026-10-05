//! `sensor_msgs/CompressedImage` depth (16-bit PNG or JPEG XL, also ROS `compressedDepth`) from dimos (LCM over zenoh), lossless on a
//! data channel as zenoh-web fields.

use super::dimos_lcm_compressed_image;
use crate::utils::depth;
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, Compress, DecodedFrame};

/// `dimos_lcm_compressed_depth`
pub struct DimosLcmCompressedDepth;

impl Codec for DimosLcmCompressedDepth {
    fn name(&self) -> &str {
        "dimos_lcm_compressed_depth"
    }

    fn output(&self) -> CodecOutput {
        CodecOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    fn decode(&self, sample: &CodecSample<'_>) -> Result<DecodedFrame> {
        let message = dimos_lcm_compressed_image::parse(sample.payload)?;
        Ok(DecodedFrame::data(depth::from_compressed(message.data, &message.format)?))
    }

    fn encode(&self, frame: &DecodedFrame, quality: f64) -> Result<Vec<u8>> {
        Ok(depth::encode(frame.downcast::<depth::Depth>()?, quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, quality: f64) -> f64 {
        depth::estimated_bytes(payload_bytes, quality)
    }
}
