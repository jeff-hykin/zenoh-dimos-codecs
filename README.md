# zenoh-dimos-codecs

[zenoh-web](https://github.com/jeff-hykin/zenoh-web) codecs for ROS 2 (rmw_zenoh) and dimos sensor
messages: images as H.264 video (or JPEG files), lossless depth and quantized point clouds. A Rust
crate of `zenoh_web::Codec`s plus their browser decoders (`client/dimos_codecs.ts`). The
`zenoh-web` command ([zenoh-web-cli](https://github.com/jeff-hykin/zenoh-web-cli)) has them all.

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
import { connect, registerCodec } from "https://esm.sh/gh/jeff-hykin/zenoh-web@<commit>/client/zenoh_web.ts"
import { registerDimosCodecs } from "https://esm.sh/gh/jeff-hykin/zenoh-dimos-codecs@<commit>/client/dimos_codecs.ts"
registerDimosCodecs(registerCodec)   // depth and point cloud messages get msg.decoded
const z = await connect("http://robot.local:7448")
z.subscribe("dimos/camera/depth/sensor_msgs.Image", { codec: "dimos-depth" }, (msg) => draw(msg.decoded))
```

Also exported: `CODECS` (the names), `codecOutput(codec)` (`"video"`, `"depth"` or `"pointcloud"`),
`decodeDepth(bytes)`, `decodePointCloud(bytes)`, and the `DepthImage` / `PointCloud` types.

## Codecs

Named `<protocol>-<input type>`; the input type decides the output:

| codec | input message | output |
|---|---|---|
| `ros2-image`, `dimos-image` | `sensor_msgs/Image`: rgb8, bgr8, rgba8, bgra8, mono8, mono16/16UC1 (top 8 bits), or `jpeg`/`png` data in an Image (dimos' jpeg-encoded Image) | H.264 video track (`sub.mediaStream`) |
| `ros2-compressed-image`, `dimos-compressed-image` | `sensor_msgs/CompressedImage`: jpeg, png, webp, jxl (magic bytes first, `format` string second) | H.264 video track |
| `ros2-depth`, `dimos-depth` | `sensor_msgs/Image`: 16UC1, 32FC1, mono16 | lossless depth: `msg.decoded` is a `DepthImage` |
| `ros2-compressed-depth`, `dimos-compressed-depth` | `sensor_msgs/CompressedImage`: 16-bit gray png or jxl, ROS `compressedDepth` png (12-byte header skipped; its quantized 32FC1 form is refused) | lossless depth |
| `ros2-pointcloud2`, `dimos-pointcloud2` | `sensor_msgs/PointCloud2`, any field layout | quantized points: `msg.decoded` is a `PointCloud` |

The image codecs also send JPEG files (zenoh-web's `imageTransport: "jpeg"`); a JPEG source passes
through untouched at full quality.

Inputs:
- ROS 2 over rmw_zenoh: key `<domain>/<topic>/<pkg>::msg::dds_::<Type>_/RIHS01_<hash>`, payload CDR with
  the 4-byte encapsulation header (little or big endian honored).
- dimos over zenoh: key `<topic>/<msg_name>` (e.g. `dimos/camera/color/sensor_msgs.Image`), payload in
  the dimos message format (big endian) with its 8-byte type fingerprint, which the codec checks (a
  wrong type is an error, counted in zenoh-web's `codecErrors` / `lastCodecError` stats).
- mono16 is ambiguous (IR intensity or depth-like); the subscriber decides: `*-image` shows its top
  8 bits as gray video, `*-depth` delivers it losslessly with encoding `mono16`.
- Decoders are pure Rust (zune-jpeg, png, image-webp, jxl-oxide), compression zstd. A YCbCr JPEG with
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

## Wire formats

Little endian; the browser decodes zstd with vendored fzstd (`client/vendor/`), since
`DecompressionStream("zstd")` isn't in every browser yet.

- depth: `u8 version=1 | u8 encoding (1 16UC1, 2 32FC1, 3 mono16) | u16 stride | u32 width |
  u32 height | u32 sourceWidth | u32 sourceHeight | zstd(width × height values)`
- point cloud: `u8 version=2 | u8 flags (bit0 intensity) | u16 0 | u32 pointCount | u32 sourcePointCount |
  f32 originX | f32 originY | f32 originZ | f32 scale | u32 keepEvery | f32 intensityMin |
  f32 intensityScale | zstd(i16 x, y, z per point, then u8 intensity per point if flagged)`

## Tests

```sh
cargo test && cargo clippy --all-targets   # parsers, decoders, encoders against test/fixtures
deno task check                            # type-check the browser decoders
```

`test/fixtures/` holds one payload per message (made by dimos's own encoder and by rosbags;
`generate.py` rebuilds them, `manifest.json` describes them). zenoh-web-cli's `test/codecs.js` sends
every fixture through every codec into headless Chrome.
