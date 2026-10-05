# zenoh-dimos-codecs

[zenoh-web](https://github.com/jeff-hykin/zenoh-web) codecs for ROS 2 (rmw_zenoh) and dimos sensor
messages: images as H.264 video, lossless depth and quantized point clouds, plus hardware H.264 encoders
(see [Hardware encoders](#hardware-encoders)). A Rust crate of `zenoh_web::Codec`s and encoders: depth and point clouds are zenoh-web **fields**, which
zenoh-web's own browser client decodes, so pages need no code from here. The `zenoh-web` command
([zenoh-web-cli](https://github.com/jeff-hykin/zenoh-web-cli)) has them all.

```toml
[dependencies]
zenoh-dimos-codecs = { git = "https://github.com/jeff-hykin/zenoh-dimos-codecs", rev = "<commit>" }
```

```rust
let mut builder = zenoh_web::Server::builder();
for codec in zenoh_dimos_codecs::all() {
    builder = builder.shared_codec(codec);
}
```

```js
import { connect } from "https://esm.sh/gh/jeff-hykin/zenoh-web@<commit>/client/zenoh_web.ts"
const z = await connect("http://robot.local:7448")
z.codecs   // [{ name: "dimos-depth", output: "fields" }, { name: "dimos-image", output: "video" }, ...]
z.subscribe("dimos/camera/depth/sensor_msgs.Image", { codec: "dimos-depth" }, (msg) => draw(msg.decoded))
```

## Codecs

Named `<protocol>-<input type>`; the input type decides the output:

| codec | input message | output |
|---|---|---|
| `ros2-image`, `dimos-image` | `sensor_msgs/Image`: rgb8, bgr8, rgba8, bgra8, mono8, mono16/16UC1 (top 8 bits), or `jpeg`/`png` data in an Image (dimos' jpeg-encoded Image) | H.264 video track (`sub.mediaStream`) |
| `ros2-compressed-image`, `dimos-compressed-image` | `sensor_msgs/CompressedImage`: jpeg, png, webp, jxl (magic bytes first, `format` string second) | H.264 video track |
| `ros2-depth`, `dimos-depth` | `sensor_msgs/Image`: 16UC1, 32FC1, mono16 | lossless depth: `msg.decoded` is a depth image (below) |
| `ros2-compressed-depth`, `dimos-compressed-depth` | `sensor_msgs/CompressedImage`: 16-bit gray png or jxl, ROS `compressedDepth` png (12-byte header skipped; its quantized 32FC1 form is refused) | lossless depth |
| `ros2-pointcloud2`, `dimos-pointcloud2` | `sensor_msgs/PointCloud2`, any field layout | quantized points: `msg.decoded` is a point cloud (below) |

Inputs:
- ROS 2 over rmw_zenoh: key `<domain>/<topic>/<pkg>::msg::dds_::<Type>_/RIHS01_<hash>`, payload CDR with
  the 4-byte encapsulation header (little or big endian honored).
- dimos over zenoh: key `<topic>/<msg_name>` (e.g. `dimos/camera/color/sensor_msgs.Image`), payload in
  the dimos message format (big endian) with its 8-byte type fingerprint, which the codec checks (a
  wrong type is an error, counted in zenoh-web's `codecErrors` / `lastCodecError` stats).
- mono16 is ambiguous (IR intensity or depth-like); the subscriber decides: `*-image` shows its top
  8 bits as gray video, `*-depth` delivers it losslessly with encoding `mono16`.
- Decoders are pure Rust (zune-jpeg, png, image-webp, jxl-oxide). A YCbCr JPEG with
  even sides decodes straight to I420 (full range → BT.601 limited), never through RGB; everything
  else decodes to RGB8.

Depth stays lossless: quality only lowers resolution, by an integer stride `round(1 / (1/8 + 7/8 q))`
(1 at q = 1, 2 at 0.5, 8 at 0), nearest neighbor (every value is a source value, never a blend).
`data` is a `Uint16Array` (16UC1, mono16) or `Float32Array` (32FC1).

Point clouds: points with a non-finite x, y or z are skipped; fields are read by name (`x`, `y`, `z`,
optional `intensity`) at their offsets with any PointField datatype, honoring `point_step`, `row_step`
and `is_bigendian`. Quality q thins the cloud to 1 point in every `round(1 / q)` (at most 16 at q = 0):
points 0, N, 2N, ... in message order, never moved or merged, with no assumption about units or
spacing (`keepEvery`). Coordinates are int16 around a per-message origin: `x = originX + qx × scale`.
Error per axis of a sent point is at most `scale / 2` (`maxError`), where `scale = (largest
bounding-box extent / 2) / 32767` in the cloud's own units, plus f32 rounding. Intensity is scaled to
u8 over the message's min..max (`intensityMin`, `intensityScale`). `positions` is a `Float32Array`.

## Hardware encoders

`zenoh_dimos_codecs::encoders`: hardware H.264 encoders as zenoh-web `VideoEncoder`s for
`ServerBuilder::video_encoder` (formerly the zenoh-web-encoders crate). [zenoh-web-cli](https://github.com/jeff-hykin/zenoh-web-cli)
and [zenoh-web-relay](https://github.com/jeff-hykin/zenoh-web-relay) use them (`--video-encoder auto|software|videotoolbox|gstreamer`).

| feature | backend | needs |
|---|---|---|
| `videotoolbox` | macOS VideoToolbox (the media engine): constrained baseline, low-latency rate control | macOS (does nothing elsewhere) |
| `gstreamer` | the first GStreamer hardware encoder that works: `nvv4l2h264enc` (Jetson), `nvh264enc` (NVENC), `vah264enc` / `vaapih264enc` (VAAPI) | GStreamer 1.x at runtime only: it is loaded with `dlopen`, so building needs nothing and a machine without it falls back to software |

```toml
zenoh-dimos-codecs = { git = "https://github.com/jeff-hykin/zenoh-dimos-codecs", rev = "<commit>", features = ["videotoolbox", "gstreamer"] }
```

```rust
let selected = zenoh_dimos_codecs::encoders::select(zenoh_dimos_codecs::encoders::Backend::Auto)?;
if let Some(factory) = selected.factory {
    builder = builder.video_encoder(factory); // else zenoh-web's software H.264
}
```

- `select` probes each backend by encoding a test frame; `Auto` tries VideoToolbox, then GStreamer, then picks
  software. A named backend that doesn't work is an error.
- Each encoder is wrapped in `Fallback`: after its first error it hands over to openh264 for good (a new software
  encoder starts with a keyframe).
- All of them encode at the bitrate zenoh-web grants (changed in place, no keyframe), restart on a new picture size,
  give keyframes on request, and signal zenoh-web's colors (BT.601 matrix, BT.709 primaries and transfer).
- `examples/encode_file.rs` encodes raw RGB frames (or a moving test pattern) with any backend to an `.h264` file, to
  try an encoder on a machine and measure it offline.

Measured on the zenoh-web bench scene (720p60, offline, decoded by ffmpeg): VideoToolbox 3.95 Mbit/s → 29.34 dB,
8.2 → 29.68, 15.4 → 29.90; openh264 4.0 → 29.37, 8.3 → 29.68, 16.2 → 29.88. Same quality per bit, but on the media
engine instead of a core per stream.

On a Jetson AGX Orin (JetPack 6, GStreamer 1.20; under load from the robot's own stack): `nvv4l2h264enc` took 6-14 ms
per 720p frame (three frames in flight) against openh264's 34 ms, hit its bitrate, and scored 30.27 dB on the bench
scene at 16.6 Mbit/s against openh264's 29.57.

## Wire formats

Both are zenoh-web fields messages (zenoh-web SPEC "Fields"), zstd-compressed by default
(`Codec::default_compress`; a subscription's `compress: "none"` turns it off). The client decodes them
into `msg.decoded`:

- depth (`version` 2): `{ version, encoding: "16UC1" | "32FC1" | "mono16", stride, width, height,
  sourceWidth, sourceHeight, data }`; `data` is width × height values, row-major.
- point cloud (`version` 3): `{ version, count, sourceCount, origin (Float32Array of 3), scale, keepEvery,
  maxError, positions (Float32Array, x, y, z per point, sent as int16 scaled by origin and scale),
  intensity (Uint8Array, only if the cloud has intensity), intensityMin, intensityScale }`.

## Nix / cross compiling

`nix build .#zenoh-dimos-codecs-example` (native) and `.#zenoh-dimos-codecs-example-aarch64-linux` /
`-x86_64-linux` build `nix_smoke_test/` with both encoder features: `zenoh-dimos-codecs-example [auto|software|videotoolbox|gstreamer] [frames]`
lists the codecs, starts and stops a loopback server with them, then selects an encoder and times it on a 720p test
pattern. Built with zenoh-web's `lib.crossRust`: crate2nix, one derivation per crate shared with the other zenoh-web
flakes, Linux cross compiled with zig (glibc 2.35). GStreamer is opened at runtime, so the Linux builds need no
GStreamer and the feature is always compiled in. Pass `--max-jobs auto`. After changing `nix_smoke_test/Cargo.lock`,
`nix run github:jeff-hykin/zenoh-web#crate2nix -- generate` in `nix_smoke_test/`. To build your own crate that uses this one,
see zenoh-web's README "Nix / cross compiling".

## Tests

```sh
cargo test && cargo clippy --all-targets   # parsers, decoders, encoders against test/fixtures
```

`test/fixtures/` holds one payload per message (made by dimos's own encoder and by rosbags;
`generate.py` rebuilds them, `manifest.json` describes them). zenoh-web-cli's `test/codecs.js` sends
every fixture through every codec into headless Chrome.
