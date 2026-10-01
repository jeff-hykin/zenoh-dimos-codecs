/// <reference no-default-lib="true" />
/// <reference lib="dom" />
/// <reference lib="esnext" />
// Browser decoders for zenoh-dimos-codecs' data codecs (depth, point clouds), see README.md

import { decompress as zstdDecompress } from "./vendor/fzstd.ts"

/** The codecs, picked with the subscribe option `codec`. */
export const CODECS = Object.freeze([
    "ros2-image",
    "ros2-compressed-image",
    "ros2-depth",
    "ros2-compressed-depth",
    "ros2-pointcloud2",
    "dimos-image",
    "dimos-compressed-image",
    "dimos-depth",
    "dimos-compressed-depth",
    "dimos-pointcloud2",
] as const)

export type CodecName = typeof CODECS[number]
/** What a codec delivers: a video track, lossless depth or a point cloud. */
export type CodecOutput = "video" | "depth" | "pointcloud"

/** The output of a codec. */
export function codecOutput(codec: CodecName): CodecOutput {
    if (codec.endsWith("pointcloud2")) {
        return "pointcloud"
    }
    return codec.endsWith("depth") ? "depth" : "video"
}

/** Lossless depth, downscaled by `stride` (nearest neighbor) at lower quality. */
export interface DepthImage {
    width: number
    height: number
    sourceWidth: number
    sourceHeight: number
    stride: number
    encoding: "16UC1" | "32FC1" | "mono16"
    /** row-major; Uint16Array for 16UC1/mono16, Float32Array for 32FC1 */
    data: Uint16Array | Float32Array
}

export interface PointCloud {
    count: number
    /** finite points in the source message, before thinning */
    sourceCount: number
    /** x, y, z per point */
    positions: Float32Array
    /** 0..255 per point; the source value is intensityMin + value * intensityScale */
    intensity: Uint8Array | null
    intensityMin: number
    intensityScale: number
    origin: [number, number, number]
    /** cloud units per quantization step */
    scale: number
    /** 1 point kept in every `keepEvery` source points (1 = all) */
    keepEvery: number
    /** largest per-axis error of a sent point against its source point: scale / 2 */
    maxError: number
}

// browsers are little endian, so typed arrays can view the decoded little-endian values directly
const depthEncodings = { 1: "16UC1", 2: "32FC1", 3: "mono16" } as const

/** Depth codec payload: 20-byte header + zstd(values), see README "Wire formats". */
export function decodeDepth(bytes: Uint8Array): DepthImage {
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
    if (bytes[0] !== 1) {
        throw new Error(`zenoh-dimos-codecs: unknown depth format version ${bytes[0]}`)
    }
    const encoding = depthEncodings[bytes[1] as 1 | 2 | 3]
    if (!encoding) {
        throw new Error(`zenoh-dimos-codecs: unknown depth encoding ${bytes[1]}`)
    }
    const width = view.getUint32(4, true)
    const height = view.getUint32(8, true)
    const values = zstdDecompress(bytes.subarray(20)) as Uint8Array
    const aligned = values.byteOffset % 4 === 0 ? values : values.slice()
    const count = width * height
    const data = encoding === "32FC1" ? new Float32Array(aligned.buffer, aligned.byteOffset, count) : new Uint16Array(aligned.buffer, aligned.byteOffset, count)
    return { width, height, sourceWidth: view.getUint32(12, true), sourceHeight: view.getUint32(16, true), stride: view.getUint16(2, true), encoding, data }
}

/** Point cloud codec payload: 40-byte header + zstd(int16 xyz, u8 intensity), see README "Wire formats". */
export function decodePointCloud(bytes: Uint8Array): PointCloud {
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
    if (bytes[0] !== 2) {
        throw new Error(`zenoh-dimos-codecs: unknown point cloud format version ${bytes[0]}`)
    }
    const hasIntensity = (bytes[1] & 1) === 1
    const count = view.getUint32(4, true)
    const origin: [number, number, number] = [view.getFloat32(12, true), view.getFloat32(16, true), view.getFloat32(20, true)]
    const scale = view.getFloat32(24, true)
    const keepEvery = view.getUint32(28, true)
    const body = zstdDecompress(bytes.subarray(40)) as Uint8Array
    const quantized = new DataView(body.buffer, body.byteOffset, body.byteLength)
    const positions = new Float32Array(count * 3)
    for (let index = 0; index < count * 3; index++) {
        positions[index] = origin[index % 3] + quantized.getInt16(index * 2, true) * scale
    }
    return {
        count,
        sourceCount: view.getUint32(8, true),
        positions,
        intensity: hasIntensity ? body.slice(count * 6, count * 7) : null,
        intensityMin: view.getFloat32(32, true),
        intensityScale: view.getFloat32(36, true),
        origin,
        scale,
        keepEvery,
        maxError: scale / 2,
    }
}

/**
 * Registers the depth and point cloud decoders with zenoh-web's client, so `msg.decoded` is a
 * `DepthImage` or a `PointCloud`: `registerDimosCodecs(registerCodec)`.
 */
export function registerDimosCodecs(registerCodec: (name: string, decoder: (bytes: Uint8Array) => unknown) => void): void {
    for (const name of CODECS) {
        const output = codecOutput(name)
        if (output !== "video") {
            registerCodec(name, output === "depth" ? decodeDepth : decodePointCloud)
        }
    }
}
