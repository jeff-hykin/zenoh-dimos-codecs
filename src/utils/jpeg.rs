//! JPEG decoding (zune-jpeg): to RGB8, or a YCbCr JPEG straight to I420 for the video path; and encoding (jpeg-encoder).

use crate::utils::raw_pixels::{Rgb8, samples_to_rgb};
use anyhow::{Context, Result, ensure};
use zenoh_web::VideoImage;

/// RGB8 as a JPEG at `quality` (0..1 → JPEG quality 20..95).
pub fn encode(image: &Rgb8, quality: f64) -> Result<Vec<u8>> {
    let (width, height) = (u16::try_from(image.width).context("jpeg: wider than 65535")?, u16::try_from(image.height).context("jpeg: taller than 65535")?);
    let mut out = Vec::new();
    let jpeg_quality = (20.0 + 75.0 * quality.clamp(0.0, 1.0)).round() as u8;
    jpeg_encoder::Encoder::new(&mut out, jpeg_quality).encode(&image.pixels, width, height, jpeg_encoder::ColorType::Rgb).map_err(|e| anyhow::anyhow!("jpeg: {e}"))?;
    Ok(out)
}

/// A JPEG as RGB8.
pub fn to_rgb(data: &[u8]) -> Result<Rgb8> {
    use zune_jpeg::zune_core::bytestream::ZCursor;
    use zune_jpeg::zune_core::colorspace::ColorSpace;
    use zune_jpeg::zune_core::options::DecoderOptions;
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::RGB);
    let mut decoder = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(data), options);
    let pixels = decoder.decode().map_err(|e| anyhow::anyhow!("jpeg: {e:?}"))?;
    let info = decoder.info().context("jpeg: no header")?;
    samples_to_rgb(&pixels, 3, info.width as u32, info.height as u32)
}

/// JFIF's full-range YCbCr as BT.601 limited-range I420 (what the H.264 stream is tagged as, and
/// what zenoh-web's RGB conversion produces), chroma from each 2x2 block's mean.
/// `None` for a JPEG that isn't YCbCr or has an odd side.
pub fn to_i420(data: &[u8]) -> Result<Option<VideoImage>> {
    use zune_jpeg::zune_core::bytestream::ZCursor;
    use zune_jpeg::zune_core::colorspace::ColorSpace;
    use zune_jpeg::zune_core::options::DecoderOptions;
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::YCbCr);
    let mut decoder = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(data), options);
    decoder.decode_headers().map_err(|e| anyhow::anyhow!("jpeg: {e:?}"))?;
    let info = decoder.info().context("jpeg: no header")?;
    let (width, height) = (info.width as usize, info.height as usize);
    if decoder.input_colorspace() != Some(ColorSpace::YCbCr) || width == 0 || height == 0 || width % 2 == 1 || height % 2 == 1 {
        return Ok(None);
    }
    let pixels = decoder.decode().map_err(|e| anyhow::anyhow!("jpeg: {e:?}"))?;
    ensure!(pixels.len() >= width * height * 3, "jpeg: decoder returned too few samples");
    let luma_table: [u8; 256] = std::array::from_fn(|value| (16 + (value as u32 * 219 + 127) / 255) as u8);
    let chroma_table: [u8; 1021] = std::array::from_fn(|sum| (16 + (sum as u32 * 224 + 510) / 1020) as u8);
    let mut out = vec![0u8; width * height * 3 / 2];
    let (luma, chroma) = out.split_at_mut(width * height);
    let (u_plane, v_plane) = chroma.split_at_mut(width * height / 4);
    for (luma_value, pixel) in luma.iter_mut().zip(pixels.as_chunks::<3>().0) {
        *luma_value = luma_table[pixel[0] as usize];
    }
    let half_width = width / 2;
    for (block_row, (u_row, v_row)) in u_plane.chunks_exact_mut(half_width).zip(v_plane.chunks_exact_mut(half_width)).enumerate() {
        let top = pixels[block_row * 2 * width * 3..][..width * 3].as_chunks::<6>().0;
        let bottom = pixels[(block_row * 2 + 1) * width * 3..][..width * 3].as_chunks::<6>().0;
        for ((u, v), (a, b)) in u_row.iter_mut().zip(v_row.iter_mut()).zip(top.iter().zip(bottom)) {
            *u = chroma_table[a[1] as usize + a[4] as usize + b[1] as usize + b[4] as usize];
            *v = chroma_table[a[2] as usize + a[5] as usize + b[2] as usize + b[5] as usize];
        }
    }
    Ok(Some(VideoImage::i420(width as u32, height as u32, out)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// BT.601 limited-range I420 with chroma from each 2x2 block's mean, as zenoh-web converts RGB.
    fn rgb_to_i420(rgb: &Rgb8) -> Vec<u8> {
        let width = rgb.width as usize;
        let pixel = |x: usize, y: usize| {
            let at = (y * width + x) * 3;
            [0, 1, 2].map(|channel| rgb.pixels[at + channel] as i32)
        };
        let mut out: Vec<u8> = (0..rgb.pixels.len() / 3).map(|index| {
            let [r, g, b] = pixel(index % width, index / width);
            (((66 * r + 129 * g + 25 * b + 128) >> 8) + 16) as u8
        }).collect();
        for weights in [[-38, -74, 112], [112, -94, -18]] {
            for y in 0..rgb.height as usize / 2 {
                for x in 0..width / 2 {
                    let mean = |channel: usize| (pixel(2 * x, 2 * y)[channel] + pixel(2 * x + 1, 2 * y)[channel] + pixel(2 * x, 2 * y + 1)[channel] + pixel(2 * x + 1, 2 * y + 1)[channel] + 2) >> 2;
                    out.push((((weights[0] * mean(0) + weights[1] * mean(1) + weights[2] * mean(2) + 128) >> 8) + 128).clamp(0, 255) as u8);
                }
            }
        }
        out
    }

    #[test]
    fn jpeg_goes_to_i420_like_rgb_would() {
        let jpeg = include_bytes!("../../test/fixtures/test_image.jpg");
        let direct = to_i420(jpeg).unwrap().unwrap();
        assert_eq!(direct.format(), zenoh_web::PixelFormat::I420, "a YCbCr jpeg skips rgb");
        let rgb = to_rgb(jpeg).unwrap();
        let through_rgb = rgb_to_i420(&rgb);
        // RGB clips out-of-gamut YCbCr, so a few saturated pixels may differ more; on average they agree
        let mut differences: Vec<i32> = direct.data().iter().zip(&through_rgb).map(|(a, b)| (*a as i32 - *b as i32).abs()).collect();
        differences.sort();
        let mean = differences.iter().sum::<i32>() as f64 / differences.len() as f64;
        let p99 = differences[differences.len() * 99 / 100];
        assert!(mean < 1.0 && p99 <= 5, "I420 straight from the jpeg differs from going through RGB: mean {mean:.2}, p99 {p99}");
    }
}
