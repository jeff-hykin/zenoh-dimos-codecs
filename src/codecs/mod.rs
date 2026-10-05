//! The codecs: [`msgs`] has one per message type.

pub mod msgs;

use std::sync::Arc;
use zenoh_web::MessageEncoding;

/// Every codec, ready for `zenoh_web::ServerBuilder::shared_encoding`.
pub fn all() -> Vec<Arc<dyn MessageEncoding>> {
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
    use zenoh_web::{Channel, Compress, DecodedFrame, EncodeOptions, EncodingOutput, EncodingSample, VideoFormat};
    use crate::utils::tests::fixture;

    fn decode_and_encode(name: &str, file: &str, quality: f64) -> Vec<u8> {
        let codec = all().into_iter().find(|codec| codec.name() == name).unwrap();
        let payload = fixture(file);
        let encoding = zenoh_web::zenoh::bytes::Encoding::default();
        let frame = codec.decode(&EncodingSample::new("k", &payload, &encoding), Channel::Data).unwrap();
        codec.encode(&frame, &EncodeOptions::quality(quality)).unwrap()
    }

    #[test]
    fn outputs_and_round_trips() {
        let names: Vec<_> = all().iter().map(|codec| (codec.name().to_owned(), codec.output())).collect();
        assert_eq!(names.len(), 12);
        assert!(names.contains(&("ros2_image".to_owned(), EncodingOutput::Video)));
        assert!(names.contains(&("dimos_lcm_depth".to_owned(), EncodingOutput::Fields)));
        assert!(names.contains(&("ros2_raw_audio".to_owned(), EncodingOutput::Audio)));
        let mut files: Vec<String> = std::fs::read_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/src/codecs/msgs")).unwrap().map(|entry| entry.unwrap().file_name().into_string().unwrap()).filter(|file| file != "mod.rs").map(|file| file.trim_end_matches(".rs").to_owned()).collect();
        let mut registered: Vec<String> = names.iter().map(|(name, _)| name.clone()).collect();
        files.sort();
        registered.sort();
        assert_eq!(registered, files, "every file in codecs/msgs is registered, under its file name");
        assert!(all().iter().all(|codec| (codec.default_compress() == Compress::Zstd) == (codec.output() == EncodingOutput::Fields)), "depth and point clouds stay compressed");
        let fields = |name: &str, file: &str, quality: f64| zenoh_web::fields::parse(&decode_and_encode(name, file, quality)).unwrap();
        let depth = fields("ros2_depth", "ros2/depth_16UC1.cdr", 1.0);
        assert_eq!((depth["version"].values(), depth["encoding"].text()), (vec![2.0], Some("16UC1")));
        assert_eq!(fields("dimos_lcm_depth", "dimos/depth_16UC1.bin", 0.5)["stride"].values(), [2.0], "stride 2 at quality 0.5");
        let cloud = fields("dimos_lcm_pointcloud2", "dimos/pointcloud_xyzi.bin", 1.0);
        assert_eq!((cloud["version"].values(), cloud["count"].values()), (vec![3.0], vec![20000.0]));
        let codec = all().into_iter().find(|codec| codec.name() == "dimos_lcm_image").unwrap();
        let payload = fixture("dimos/image_rgb8.bin");
        let encoding = zenoh_web::zenoh::bytes::Encoding::default();
        let DecodedFrame::Video(image) = codec.decode(&EncodingSample::new("k", &payload, &encoding), Channel::Video(VideoFormat::H264)).unwrap() else { panic!("video expected") };
        assert_eq!((image.width(), image.height()), (320, 240));
    }

    /// `name`'s data-channel bytes for `file` with `options` at `quality`, after checking it accepts them.
    fn on_data(name: &str, file: &str, options: serde_json::Value, quality: f64) -> Result<Vec<u8>, String> {
        let codec = all().into_iter().find(|codec| codec.name() == name).unwrap();
        let options: serde_json::Map<String, serde_json::Value> = serde_json::from_value(options).unwrap();
        assert_eq!(codec.output_on(Channel::Data, &options)?, EncodingOutput::Data);
        let payload = fixture(file);
        let encoding = zenoh_web::zenoh::bytes::Encoding::default();
        let frame = codec.decode(&EncodingSample::new("k", &payload, &encoding), Channel::Data).map_err(|error| error.to_string())?;
        codec.encode(&frame, &EncodeOptions { quality, options }).map_err(|error| error.to_string())
    }

    #[test]
    fn pictures_on_the_data_channel_pass_through_or_convert() {
        use crate::utils::compressed_image::{FileFormat, sniff_format};
        let png = fixture("compressed/pattern.png");
        assert_eq!(on_data("ros2_compressed_image", "ros2/compressed_png.cdr", serde_json::json!({}), 0.2).unwrap(), png, "the file as sent");
        let jpeg = on_data("dimos_lcm_compressed_image", "dimos/compressed_png.bin", serde_json::json!({"format": "jpeg"}), 0.5).unwrap();
        assert_eq!(sniff_format(&jpeg, "").unwrap(), FileFormat::Jpeg, "png on the wire, jpeg asked for");
        let raw = on_data("ros2_image", "ros2/image_rgb8.cdr", serde_json::json!({}), 1.0).unwrap();
        assert_eq!(sniff_format(&raw, "").unwrap(), FileFormat::Jpeg, "raw pixels go as jpeg by default");
        let raw_png = on_data("dimos_lcm_image", "dimos/image_rgb8.bin", serde_json::json!({"format": "png"}), 1.0).unwrap();
        crate::utils::raw_pixels::tests::assert_pattern(&crate::utils::png::to_rgb(&raw_png).unwrap(), 0.0, "raw → png is lossless");
        assert!(on_data("ros2_image", "ros2/image_rgb8.cdr", serde_json::json!({"format": "passthrough"}), 1.0).is_err());
        let depth = all().into_iter().find(|codec| codec.name() == "ros2_depth").unwrap();
        assert!(depth.output_on(Channel::Video(VideoFormat::H264), &serde_json::Map::new()).is_err(), "depth only goes on data");
        assert!(depth.output_on(Channel::Data, &serde_json::from_value(serde_json::json!({"format": "png"})).unwrap()).is_err(), "and takes no options");
    }
}
