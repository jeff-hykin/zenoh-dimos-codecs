//! The codecs: [`msgs`] has one per message type.

pub mod msgs;

use std::sync::Arc;
use zenoh_web::Codec;

/// Every codec, ready for `zenoh_web::ServerBuilder::shared_codec`.
pub fn all() -> Vec<Arc<dyn Codec>> {
    use msgs::*;
    vec![
        Arc::new(Ros2Image),
        Arc::new(Ros2CompressedImage),
        Arc::new(Ros2Depth),
        Arc::new(Ros2CompressedDepth),
        Arc::new(Ros2PointCloud2),
        Arc::new(Ros2RawAudio),
        Arc::new(DimosLcmImage),
        Arc::new(DimosLcmCompressedImage),
        Arc::new(DimosLcmDepth),
        Arc::new(DimosLcmCompressedDepth),
        Arc::new(DimosLcmPointCloud2),
        Arc::new(DimosLcmRawAudio),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use zenoh_web::{CodecOutput, CodecSample, Compress, DecodedFrame};
    use crate::utils::tests::fixture;

    fn decode_and_encode(name: &str, file: &str, quality: f64) -> Vec<u8> {
        let codec = all().into_iter().find(|codec| codec.name() == name).unwrap();
        let payload = fixture(file);
        let encoding = zenoh_web::zenoh::bytes::Encoding::default();
        let frame = codec.decode(&CodecSample::new("k", &payload, &encoding)).unwrap();
        codec.encode(&frame, quality).unwrap()
    }

    #[test]
    fn outputs_and_round_trips() {
        let names: Vec<_> = all().iter().map(|codec| (codec.name().to_owned(), codec.output())).collect();
        assert_eq!(names.len(), 12);
        assert!(names.contains(&("ros2_image".to_owned(), CodecOutput::Video)));
        assert!(names.contains(&("dimos_lcm_depth".to_owned(), CodecOutput::Fields)));
        assert!(names.contains(&("ros2_raw_audio".to_owned(), CodecOutput::Audio)));
        let mut files: Vec<String> = std::fs::read_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/src/codecs/msgs")).unwrap().map(|entry| entry.unwrap().file_name().into_string().unwrap()).filter(|file| file != "mod.rs").map(|file| file.trim_end_matches(".rs").to_owned()).collect();
        let mut registered: Vec<String> = names.iter().map(|(name, _)| name.clone()).collect();
        files.sort();
        registered.sort();
        assert_eq!(registered, files, "every file in codecs/msgs is registered, under its file name");
        assert!(all().iter().all(|codec| (codec.default_compress() == Compress::Zstd) == (codec.output() == CodecOutput::Fields)), "depth and point clouds stay compressed");
        let fields = |name: &str, file: &str, quality: f64| zenoh_web::fields::parse(&decode_and_encode(name, file, quality)).unwrap();
        let depth = fields("ros2_depth", "ros2/depth_16UC1.cdr", 1.0);
        assert_eq!((depth["version"].values(), depth["encoding"].text()), (vec![2.0], Some("16UC1")));
        assert_eq!(fields("dimos_lcm_depth", "dimos/depth_16UC1.bin", 0.5)["stride"].values(), [2.0], "stride 2 at quality 0.5");
        let cloud = fields("dimos_lcm_pointcloud2", "dimos/pointcloud_xyzi.bin", 1.0);
        assert_eq!((cloud["version"].values(), cloud["count"].values()), (vec![3.0], vec![20000.0]));
        let codec = all().into_iter().find(|codec| codec.name() == "dimos_lcm_image").unwrap();
        let payload = fixture("dimos/image_rgb8.bin");
        let encoding = zenoh_web::zenoh::bytes::Encoding::default();
        let DecodedFrame::Video(image) = codec.decode(&CodecSample::new("k", &payload, &encoding)).unwrap() else { panic!("video expected") };
        assert_eq!((image.width(), image.height()), (320, 240));
    }
}
