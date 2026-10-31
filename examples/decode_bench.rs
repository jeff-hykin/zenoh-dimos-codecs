//! Times a CompressedImage's trip to the video encoder, stage by stage, on what the gateway runs per frame: the
//! sample's hash (decode sharing), the CDR parse, the JPEG to a picture (`compressed_image::to_video`), then that
//! picture as the encoder's I420 at the output size (`VideoImage::to_i420`).
//! `cargo run --release --example decode_bench -- [WIDTH HEIGHT QUALITY [OUT_W OUT_H] [FILE.jpg]]` (default 1920 1080 95 1280 720)

use anyhow::Result;
use std::hash::{Hash, Hasher};
use std::time::Instant;
use zenoh_dimos_codecs::codecs::msgs::ros2_compressed_image;
use zenoh_dimos_codecs::utils::compressed_image;

/// A camera-like frame: smooth lighting, edges and texture at several scales, and sensor noise (so q95 is not tiny).
fn picture(width: usize, height: usize) -> Vec<u8> {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % 7) as i32 - 3
    };
    let mut rgb = Vec::with_capacity(width * height * 3);
    for y in 0..height {
        for x in 0..width {
            let (fx, fy) = (x as f32 / width as f32, y as f32 / height as f32);
            let light = 0.55 + 0.45 * (fx * 3.1 + fy * 1.7).sin() * (fy * 2.3).cos();
            let tile = ((x / 37 + y / 23) % 5) as f32 / 4.0;
            let stripes = if (x * 7 + y * 3) % 41 < 3 { 0.5 } else { 1.0 };
            let fine = 0.95 + 0.05 * ((x as f32 * 0.9).sin() * (y as f32 * 1.3).cos());
            for (channel, tint) in [0.9f32, 0.7, 0.5].into_iter().enumerate() {
                let base = 255.0 * light * stripes * fine * (0.4 + 0.6 * (tile + tint * (channel as f32 * 0.3 + fx)).fract());
                rgb.push((base as i32 + noise()).clamp(0, 255) as u8);
            }
        }
    }
    rgb
}

/// sensor_msgs/CompressedImage as CDR (little endian), as rmw_zenoh publishes it.
fn cdr_compressed_image(format: &str, data: &[u8]) -> Vec<u8> {
    let mut out = vec![0, 1, 0, 0];
    let mut body = Vec::new();
    let string = |body: &mut Vec<u8>, text: &str| {
        while !body.len().is_multiple_of(4) {
            body.push(0);
        }
        body.extend(((text.len() + 1) as u32).to_le_bytes());
        body.extend(text.as_bytes());
        body.push(0);
    };
    body.extend(1u32.to_le_bytes());
    body.extend(2u32.to_le_bytes());
    string(&mut body, "camera");
    string(&mut body, format);
    while !body.len().is_multiple_of(4) {
        body.push(0);
    }
    body.extend((data.len() as u32).to_le_bytes());
    body.extend(data);
    out.extend(body);
    out
}

/// 0.3.2's conversion (lookup tables), to compare.
fn ycbcr_to_i420_tables(pixels: &[u8], width: usize, height: usize) -> Vec<u8> {
    let luma_table: [u8; 256] = std::array::from_fn(|value| (16 + (value as u32 * 219 + 127) / 255) as u8);
    let chroma_table: [u8; 1021] = std::array::from_fn(|sum| (16 + (sum as u32 * 224 + 510) / 1020) as u8);
    let mut out = vec![0u8; width * height * 3 / 2];
    let (luma, chroma) = out.split_at_mut(width * height);
    let (u_plane, v_plane) = chroma.split_at_mut(width * height / 4);
    for (luma_value, pixel) in luma.iter_mut().zip(pixels.as_chunks::<3>().0) {
        *luma_value = luma_table[pixel[0] as usize];
    }
    for (block_row, (u_row, v_row)) in u_plane.chunks_exact_mut(width / 2).zip(v_plane.chunks_exact_mut(width / 2)).enumerate() {
        let top = pixels[block_row * 2 * width * 3..][..width * 3].as_chunks::<6>().0;
        let bottom = pixels[(block_row * 2 + 1) * width * 3..][..width * 3].as_chunks::<6>().0;
        for ((u, v), (a, b)) in u_row.iter_mut().zip(v_row.iter_mut()).zip(top.iter().zip(bottom)) {
            *u = chroma_table[a[1] as usize + a[4] as usize + b[1] as usize + b[4] as usize];
            *v = chroma_table[a[2] as usize + a[5] as usize + b[2] as usize + b[5] as usize];
        }
    }
    out
}

fn median_ms(rounds: usize, mut run: impl FnMut()) -> f64 {
    run();
    let mut times: Vec<f64> = (0..rounds)
        .map(|_| {
            let started = Instant::now();
            run();
            started.elapsed().as_secs_f64() * 1000.0
        })
        .collect();
    times.sort_by(f64::total_cmp);
    times[rounds / 2]
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let number = |index: usize, default: u32| args.get(index).map_or(Ok(default), |value| value.parse());
    let (width, height, quality) = (number(0, 1920)?, number(1, 1080)?, number(2, 95)?);
    let (out_width, out_height) = (number(3, 1280)?, number(4, 720)?);
    let jpeg = match args.get(5) {
        Some(path) => std::fs::read(path)?,
        None => {
            let rgb = picture(width as usize, height as usize);
            let mut jpeg = Vec::new();
            let mut encoder = jpeg_encoder::Encoder::new(&mut jpeg, quality as u8);
            encoder.set_sampling_factor(jpeg_encoder::SamplingFactor::R_4_2_0);
            encoder.encode(&rgb, width as u16, height as u16, jpeg_encoder::ColorType::Rgb).map_err(|e| anyhow::anyhow!("{e}"))?;
            jpeg
        }
    };
    let payload = cdr_compressed_image("jpeg", &jpeg);
    let image = compressed_image::to_video(&jpeg, "jpeg")?;
    println!("{}x{} q{quality} jpeg: {} KB, decoded as {:?}, out {out_width}x{out_height}", image.width(), image.height(), jpeg.len() / 1024, image.format());
    let rounds = 30;
    let hash = median_ms(rounds, || {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        "camera/image/compressed".hash(&mut hasher);
        payload.hash(&mut hasher);
        std::hint::black_box(hasher.finish());
    });
    let parse = median_ms(rounds, || drop(std::hint::black_box(ros2_compressed_image::parse(&payload).unwrap())));
    let to_video = median_ms(rounds, || drop(std::hint::black_box(compressed_image::to_video(&jpeg, "jpeg").unwrap())));
    let zune_to_video = median_ms(rounds, || drop(std::hint::black_box(zenoh_dimos_codecs::utils::jpeg::zune_to_i420(&jpeg).unwrap())));
    let to_rgb = median_ms(rounds, || drop(std::hint::black_box(compressed_image::to_rgb(&jpeg, "jpeg").unwrap())));
    let zune = |colorspace| {
        use zune_jpeg::zune_core::{bytestream::ZCursor, options::DecoderOptions};
        median_ms(rounds, || drop(std::hint::black_box(zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(&jpeg), DecoderOptions::default().jpeg_set_out_colorspace(colorspace)).decode().unwrap())))
    };
    let (zune_ycbcr, zune_rgb) = (zune(zune_jpeg::zune_core::colorspace::ColorSpace::YCbCr), zune(zune_jpeg::zune_core::colorspace::ColorSpace::RGB));
    let ycbcr = {
        use zune_jpeg::zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions};
        zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(&jpeg), DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::YCbCr)).decode()?
    };
    let (w, h) = (image.width() as usize, image.height() as usize);
    let convert = median_ms(rounds, || drop(std::hint::black_box(zenoh_dimos_codecs::utils::jpeg::ycbcr_to_i420(&ycbcr, w, h))));
    let convert_tables = median_ms(rounds, || drop(std::hint::black_box(ycbcr_to_i420_tables(&ycbcr, w, h))));
    let same_size = median_ms(rounds, || drop(std::hint::black_box(image.to_i420(image.width(), image.height()).unwrap())));
    let scaled = median_ms(rounds, || drop(std::hint::black_box(image.to_i420(out_width, out_height).unwrap())));
    println!("sample hash (decode sharing)  {hash:7.2} ms");
    println!("cdr parse                     {parse:7.2} ms");
    println!("jpeg -> picture (to_video)    {to_video:7.2} ms   (to RGB instead: {to_rgb:.2} ms)");
    println!("  zune-jpeg's to_video        {zune_to_video:7.2} ms   (the decoder without the turbojpeg feature)");
    println!("  of which zune-jpeg alone    {zune_ycbcr:7.2} ms   (zune-jpeg to RGB: {zune_rgb:.2} ms)");
    println!("  of which YCbCr -> I420      {convert:7.2} ms   (with lookup tables, as 0.3.2: {convert_tables:.2} ms)");
    println!("picture -> I420 same size     {same_size:7.2} ms");
    println!("picture -> I420 {out_width}x{out_height}     {scaled:7.2} ms");
    println!("decode stage total            {:7.2} ms   (+ {scaled:.2} ms in the encode stage)", hash + parse + to_video);
    Ok(())
}
