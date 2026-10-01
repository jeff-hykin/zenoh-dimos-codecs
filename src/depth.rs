//! Lossless depth on a data channel: nearest-neighbor downscale (never interpolated), as zenoh-web
//! fields (zstd-compressed by the bridge by default), see README "Wire formats":
//! `version=2`, `encoding` ("16UC1", "32FC1" or "mono16"), `stride`, `width`, `height`, `sourceWidth`,
//! `sourceHeight`, `data` (width × height u16 or f32, row-major).

use crate::image::{Depth, DepthEncoding, DepthValues};
use zenoh_web::Fields;

pub const VERSION: u8 = 2;

/// Integer downscale factor for a quality: 1 at quality 1, up to 8 at quality 0.
pub fn stride(quality: f64) -> u32 {
    let linear_scale = 1.0 / 8.0 + 7.0 / 8.0 * quality.clamp(0.0, 1.0);
    (1.0 / linear_scale).round().max(1.0) as u32
}

/// Fraction of full-quality pixels sent at `quality` (the allocator's size prior).
pub fn size_factor(quality: f64) -> f64 {
    1.0 / (stride(quality) as f64).powi(2)
}

pub fn encode(depth: &Depth, quality: f64) -> Vec<u8> {
    let stride = stride(quality);
    let width = depth.width.div_ceil(stride);
    let height = depth.height.div_ceil(stride);
    // every stride-th pixel of every stride-th row
    fn sample<T: Copy>(values: &[T], source_width: u32, (width, height, stride): (u32, u32, u32)) -> Vec<T> {
        (0..height).flat_map(|y| (0..width).map(move |x| values[(y * stride) as usize * source_width as usize + (x * stride) as usize])).collect()
    }
    let encoding = match depth.encoding {
        DepthEncoding::U16 => "16UC1",
        DepthEncoding::F32 => "32FC1",
        DepthEncoding::Mono16 => "mono16",
    };
    let fields = Fields::new()
        .scalar("version", VERSION)
        .text("encoding", encoding)
        .scalar("stride", stride)
        .scalar("width", width)
        .scalar("height", height)
        .scalar("sourceWidth", depth.width)
        .scalar("sourceHeight", depth.height);
    match &depth.values {
        DepthValues::U16(values) => fields.array("data", &sample(values, depth.width, (width, height, stride))),
        DepthValues::F32(values) => fields.array("data", &sample(values, depth.width, (width, height, stride))),
    }
    .build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strides() {
        assert_eq!(stride(1.0), 1);
        assert_eq!(stride(0.5), 2);
        assert_eq!(stride(0.0), 8);
    }

    #[test]
    fn full_quality_round_trips_and_downscale_is_nearest() {
        let values: Vec<u16> = (0..12u16).collect();
        let depth = Depth { width: 4, height: 3, encoding: DepthEncoding::U16, values: DepthValues::U16(values.clone()) };
        let full = zenoh_web::fields::parse(&encode(&depth, 1.0)).unwrap();
        assert_eq!((full["version"].values(), full["encoding"].text(), full["stride"].values()), (vec![2.0], Some("16UC1"), vec![1.0]));
        assert_eq!(full["data"].values(), values.iter().map(|&value| value as f64).collect::<Vec<_>>());
        let half = zenoh_web::fields::parse(&encode(&depth, 0.5)).unwrap();
        assert_eq!((half["width"].values(), half["height"].values(), half["sourceWidth"].values()), (vec![2.0], vec![2.0], vec![4.0]));
        assert_eq!(half["data"].values(), [0.0, 2.0, 8.0, 10.0], "every value is a source value, never a blend");
        let floats = Depth { width: 2, height: 1, encoding: DepthEncoding::F32, values: DepthValues::F32(vec![1.5, f32::NAN]) };
        let floats = zenoh_web::fields::parse(&encode(&floats, 1.0)).unwrap();
        assert_eq!((floats["encoding"].text(), floats["data"].dtype), (Some("32FC1"), zenoh_web::fields::Dtype::F32));
    }
}
