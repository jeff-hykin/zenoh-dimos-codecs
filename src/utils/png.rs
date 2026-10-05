//! PNG decoding (the png crate).

use crate::utils::raw_pixels::{Rgb8, samples_to_rgb};
use anyhow::{Context, Result, ensure};
use std::io::Cursor;

/// A PNG as RGB8 (16-bit samples keep their top 8 bits).
pub fn to_rgb(data: &[u8]) -> Result<Rgb8> {
    let (info, samples) = decode(data, png::Transformations::EXPAND | png::Transformations::STRIP_16)?;
    samples_to_rgb(&samples, info.color_type.samples(), info.width, info.height)
}

/// A 16-bit grayscale PNG as its u16 values (width, height, values).
pub fn to_u16(data: &[u8]) -> Result<(u32, u32, Vec<u16>)> {
    let (info, samples) = decode(data, png::Transformations::IDENTITY)?;
    ensure!(info.color_type == png::ColorType::Grayscale && info.bit_depth == png::BitDepth::Sixteen,
        "png depth must be 16-bit grayscale, got {:?} {:?}", info.color_type, info.bit_depth);
    Ok((info.width, info.height, samples.as_chunks::<2>().0.iter().map(|&s| u16::from_be_bytes(s)).collect()))
}

fn decode(data: &[u8], transformations: png::Transformations) -> Result<(png::OutputInfo, Vec<u8>)> {
    let mut decoder = png::Decoder::new(Cursor::new(data));
    decoder.set_transformations(transformations);
    let mut reader = decoder.read_info().map_err(|e| anyhow::anyhow!("png: {e}"))?;
    let mut samples = vec![0u8; reader.output_buffer_size().context("png: image too large")?];
    let info = reader.next_frame(&mut samples).map_err(|e| anyhow::anyhow!("png: {e}"))?;
    samples.truncate(info.line_size * info.height as usize);
    Ok((info, samples))
}
