//! `sensor_msgs/Image` depth (16UC1, 32FC1, mono16) from ROS 2 (rmw_zenoh, CDR), lossless on a data channel as zenoh-web fields.

use super::ros2_image;
use crate::utils::depth;
use anyhow::Result;
use zenoh_web::{Codec, CodecOutput, CodecSample, Compress, DecodedFrame};

/// `ros2_depth`
pub struct Ros2Depth;

impl Codec for Ros2Depth {
    fn name(&self) -> &str {
        "ros2_depth"
    }

    fn output(&self) -> CodecOutput {
        CodecOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    fn decode(&self, sample: &CodecSample<'_>) -> Result<DecodedFrame> {
        Ok(DecodedFrame::data(depth::from_raw(&ros2_image::parse(sample.payload)?)?))
    }

    fn encode(&self, frame: &DecodedFrame, quality: f64) -> Result<Vec<u8>> {
        Ok(depth::encode(frame.downcast::<depth::Depth>()?, quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, quality: f64) -> f64 {
        depth::estimated_bytes(payload_bytes, quality)
    }
}
