//! zenoh-dimos-codecs: [zenoh-web](https://github.com/jeff-hykin/zenoh-web) codecs for ROS 2
//! (rmw_zenoh, CDR) and dimos (LCM) sensor messages, implemented through zenoh-web's public
//! [`Codec`] trait like any external codec.
//!
//! ```no_run
//! # async fn run() -> anyhow::Result<()> {
//! let mut builder = zenoh_web::Server::builder();
//! for codec in zenoh_dimos_codecs::all() {
//!     builder = builder.shared_codec(codec);
//! }
//! let server = builder.build().await?;
//! # Ok(())
//! # }
//! ```
//!
//! One codec per message type, in [`codecs::msgs`] (e.g. [`codecs::msgs::Ros2CompressedImage`]); the input type
//! decides the output:
//! - `*_image`, `*_compressed_image`: color/mono images, H.264 on a WebRTC video track
//! - `*_depth`, `*_compressed_depth`: lossless depth (u16/f32) as zenoh-web fields
//! - `*_pointcloud2`: thinned (every Nth point) + int16 quantized points as zenoh-web fields
//! - `*_raw_audio`: `pcm-s16` audio blocks, Opus on a WebRTC audio track
//!
//! Depth and point clouds are zstd-compressed by default ([`Codec::default_compress`]); zenoh-web's
//! client decodes them into `msg.decoded` with no codec code in the page.

pub mod codecs;
/// Hardware H.264 encoders (features `videotoolbox`, `gstreamer`) with a fallback to software, for
/// [`ServerBuilder::video_encoder`](zenoh_web::ServerBuilder::video_encoder).
pub mod encoders;
/// What the codecs share: wire readers (CDR, LCM), image decoders (JPEG, PNG, WebP, JPEG XL, raw pixels), and the
/// depth and point-cloud encodings.
pub mod utils;

pub use codecs::all;
