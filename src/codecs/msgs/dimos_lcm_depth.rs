//! `sensor_msgs/Image` depth (16UC1, 32FC1, mono16) from dimos (LCM over zenoh), lossless on a data channel as zenoh-web fields.

use super::dimos_lcm_image;
use crate::utils::depth;
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, Compress, DecodedFrame};

/// `dimos_lcm_depth`
pub struct DimosLcmDepth;

impl Codec for DimosLcmDepth {
    fn name(&self) -> &str {
        "dimos_lcm_depth"
    }

    fn output(&self) -> CodecOutput {
        CodecOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    fn decode(&self, sample: &CodecSample<'_>) -> Result<DecodedFrame> {
        Ok(DecodedFrame::data(depth::from_raw(&dimos_lcm_image::parse(sample.payload)?)?))
    }

    fn encode(&self, frame: &DecodedFrame, quality: f64) -> Result<Vec<u8>> {
        Ok(depth::encode(frame.downcast::<depth::Depth>()?, quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, quality: f64) -> f64 {
        depth::estimated_bytes(payload_bytes, quality)
    }
}
