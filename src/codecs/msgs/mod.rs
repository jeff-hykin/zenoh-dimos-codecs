//! One codec per message type: each file parses its message (ROS 2: CDR, dimos: LCM) and implements
//! [`Codec`](zenoh_web::Codec). Only the fields a codec needs are kept; headers (stamp, frame_id) are skipped.

pub mod dimos_lcm_compressed_depth;
pub mod dimos_lcm_compressed_image;
pub mod dimos_lcm_depth;
pub mod dimos_lcm_image;
pub mod dimos_lcm_pointcloud2;
pub mod dimos_lcm_raw_audio;
pub mod ros2_compressed_depth;
pub mod ros2_compressed_image;
pub mod ros2_depth;
pub mod ros2_image;
pub mod ros2_pointcloud2;
pub mod ros2_raw_audio;

pub use dimos_lcm_compressed_depth::DimosLcmCompressedDepth;
pub use dimos_lcm_compressed_image::DimosLcmCompressedImage;
pub use dimos_lcm_depth::DimosLcmDepth;
pub use dimos_lcm_image::DimosLcmImage;
pub use dimos_lcm_pointcloud2::DimosLcmPointCloud2;
pub use dimos_lcm_raw_audio::DimosLcmRawAudio;
pub use ros2_compressed_depth::Ros2CompressedDepth;
pub use ros2_compressed_image::Ros2CompressedImage;
pub use ros2_depth::Ros2Depth;
pub use ros2_image::Ros2Image;
pub use ros2_pointcloud2::Ros2PointCloud2;
pub use ros2_raw_audio::Ros2RawAudio;

// The message shapes both protocols share.

/// `sensor_msgs/Image`
#[derive(Debug)]
pub struct RawImage<'a> {
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub big_endian: bool,
    pub step: u32,
    pub data: &'a [u8],
}

/// `sensor_msgs/CompressedImage`
#[derive(Debug)]
pub struct CompressedImage<'a> {
    pub format: String,
    pub data: &'a [u8],
}

#[derive(Debug, Clone, PartialEq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    /// sensor_msgs/PointField datatype: 1 INT8 .. 8 FLOAT64
    pub datatype: u8,
    pub count: u32,
}

/// `sensor_msgs/PointCloud2`
#[derive(Debug)]
pub struct PointCloud<'a> {
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub big_endian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: &'a [u8],
}

/// `foxglove_msgs/RawAudio`
#[derive(Debug)]
pub struct RawAudio<'a> {
    /// interleaved little-endian samples
    pub data: &'a [u8],
    /// `pcm-s16`
    pub format: String,
    pub sample_rate: u32,
    pub channels: u32,
}
