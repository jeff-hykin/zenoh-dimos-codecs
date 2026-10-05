//! `sensor_msgs/CompressedImage` depth (16-bit PNG or JPEG XL, also ROS `compressedDepth`) from dimos (LCM over zenoh), lossless on a
//! data channel as zenoh-gateway fields.

use super::dimos_lcm_compressed_image;
use crate::utils::depth;
use anyhow::Result;
use zenoh_gateway::{MessageEncoding, Channel, EncodeOptions, EncodingOutput, EncodingSample, Compress, DecodedFrame};

/// `dimos_lcm_compressed_depth`
pub struct DimosLcmCompressedDepth;

impl MessageEncoding for DimosLcmCompressedDepth {
    fn name(&self) -> &str {
        "dimos_lcm_compressed_depth"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    fn decode(&self, sample: &EncodingSample<'_>, _channel: Channel) -> Result<DecodedFrame> {
        let message = dimos_lcm_compressed_image::parse(sample.payload)?;
        Ok(DecodedFrame::data(depth::from_compressed(message.data, &message.format)?))
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        Ok(depth::encode(frame.downcast::<depth::Depth>()?, options.quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        depth::estimated_bytes(payload_bytes, options.quality)
    }
}
