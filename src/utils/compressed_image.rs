//! Compressed images (`sensor_msgs/CompressedImage` data, or a dimos Image carrying a file): the format from the
//! bytes, then that format's decoder.

use crate::utils::raw_pixels::Rgb8;
use crate::utils::{jpeg, jxl, png, webp};
use anyhow::{Result, bail};
use zenoh_web::VideoImage;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileFormat {
    Jpeg,
    Png,
    Webp,
    Jxl,
}

/// The file format, from magic bytes first and the message's format string second.
pub fn sniff_format(data: &[u8], hint: &str) -> Result<FileFormat> {
    if data.starts_with(&[0xff, 0xd8, 0xff]) {
        return Ok(FileFormat::Jpeg);
    }
    if data.starts_with(&[0x89, b'P', b'N', b'G']) {
        return Ok(FileFormat::Png);
    }
    if data.len() >= 12 && &data[0..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        return Ok(FileFormat::Webp);
    }
    if data.starts_with(&[0xff, 0x0a]) || data.starts_with(&[0, 0, 0, 0x0c, b'J', b'X', b'L', b' ']) {
        return Ok(FileFormat::Jxl);
    }
    let hint = hint.to_ascii_lowercase();
    for (name, format) in [("jpeg", FileFormat::Jpeg), ("jpg", FileFormat::Jpeg), ("png", FileFormat::Png), ("webp", FileFormat::Webp), ("jxl", FileFormat::Jxl)] {
        if hint.contains(name) {
            return Ok(format);
        }
    }
    bail!("unrecognized compressed image (format {hint:?}, magic {:02x?})", &data[..data.len().min(8)])
}

/// A compressed image (any [`FileFormat`]) as RGB8.
pub fn to_rgb(data: &[u8], hint: &str) -> Result<Rgb8> {
    match sniff_format(data, hint)? {
        FileFormat::Jpeg => jpeg::to_rgb(data),
        FileFormat::Png => png::to_rgb(data),
        FileFormat::Webp => webp::to_rgb(data),
        FileFormat::Jxl => jxl::to_rgb(data),
    }
}

/// A compressed image for the video path. A YCbCr JPEG with even sides is decoded to I420 without
/// ever becoming RGB: the encoder wants YUV anyway, and the YCbCr -> RGB -> YUV round trip was
/// over half the decode on an ARM core (37 ms of a 1920x1536 frame on a Jetson Orin, against 16).
/// Anything else decodes to RGB.
pub fn to_video(data: &[u8], hint: &str) -> Result<VideoImage> {
    if sniff_format(data, hint)? == FileFormat::Jpeg
        && let Some(image) = jpeg::to_i420(data)?
    {
        return Ok(image);
    }
    to_rgb(data, hint)?.into_video()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codecs::msgs::{CompressedImage, dimos_lcm_compressed_image, ros2_compressed_image};
    use crate::utils::raw_pixels::tests::assert_pattern;
    use crate::utils::tests::fixture;

    type Parse = fn(&[u8]) -> Result<CompressedImage<'_>>;

    #[test]
    fn compressed_formats() {
        for (format, tolerance) in [("jpeg", 8.0), ("png", 0.0), ("webp", 0.0), ("jxl", 8.0)] {
            for (parse, file) in [(ros2_compressed_image::parse as Parse, format!("ros2/compressed_{format}.cdr")), (dimos_lcm_compressed_image::parse, format!("dimos/compressed_{format}.bin"))] {
                let payload = fixture(&file);
                let message = parse(&payload).unwrap();
                assert_pattern(&to_rgb(message.data, &message.format).unwrap(), tolerance, &file);
            }
        }
    }
}
