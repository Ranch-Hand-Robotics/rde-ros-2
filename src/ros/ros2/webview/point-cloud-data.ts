// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

export const MAX_POINT_CLOUD_BYTES = 32 * 1024 * 1024;
export const MAX_RENDERED_POINTS = 200000;

export interface DecodedPointCloud {
  /** Interleaved x, y, z, sensor range, r, g, b, alpha (eight floats). */
  vertices: Float32Array;
  count: number;
  totalPoints: number;
  sampledPoints: number;
  skippedPoints: number;
  hasColor: boolean;
  center: [number, number, number];
  radius: number;
  minDepth: number;
  maxDepth: number;
  frameId: string;
}

interface PointField {
  offset: number;
  datatype: number;
  count: number;
}

const FIELD_BYTES = [0, 1, 1, 2, 2, 4, 4, 4, 8];
const NEUTRAL_COLOR = 0.5;
const UINT32_MAX = 0xffffffff;

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function integer(value: unknown, name: string, min: number, max: number): number {
  if (typeof value !== "number" || !Number.isSafeInteger(value) || value < min || value > max) {
    throw new Error(`Invalid PointCloud2 ${name}: expected an integer in ${min}..${max}`);
  }
  return value;
}

function base64Digit(code: number): number {
  if (code >= 65 && code <= 90) { return code - 65; }
  if (code >= 97 && code <= 122) { return code - 71; }
  if (code >= 48 && code <= 57) { return code + 4; }
  if (code === 43) { return 62; }
  if (code === 47) { return 63; }
  throw new Error("Invalid PointCloud2 base64 data");
}

function bytes(data: unknown): Uint8Array {
  if (typeof data === "string") {
    // Only normalized, padded standard base64 is accepted. Check both sizes
    // before decoding; avoid a large intermediate binary string from atob().
    if (data.length > Math.ceil(MAX_POINT_CLOUD_BYTES / 3) * 4 || data.length % 4 !== 0) {
      throw new Error("Invalid or oversized PointCloud2 base64 data");
    }
    const padding = data.endsWith("==") ? 2 : data.endsWith("=") ? 1 : 0;
    const length = integer(data.length / 4 * 3 - padding, "data length", 0, MAX_POINT_CLOUD_BYTES);
    const end = data.length - padding;
    for (let i = 0; i < end; i++) {
      base64Digit(data.charCodeAt(i));
    }
    if (padding && (base64Digit(data.charCodeAt(end - 1)) & (padding === 2 ? 15 : 3)) !== 0) {
      throw new Error("Invalid PointCloud2 base64 padding bits");
    }
    const result = new Uint8Array(length);
    for (let i = 0, output = 0; i < data.length; i += 4) {
      const a = base64Digit(data.charCodeAt(i));
      const b = base64Digit(data.charCodeAt(i + 1));
      const c = i + 2 < end ? base64Digit(data.charCodeAt(i + 2)) : 0;
      const d = i + 3 < end ? base64Digit(data.charCodeAt(i + 3)) : 0;
      result[output++] = (a << 2) | (b >> 4);
      if (output < length) { result[output++] = (b << 4) | (c >> 2); }
      if (output < length) { result[output++] = (c << 6) | d; }
    }
    return result;
  }
  if (data instanceof Uint8Array) {
    integer(data.byteLength, "data length", 0, MAX_POINT_CLOUD_BYTES);
    return data;
  }
  // Also accepts the JSON representation of a Node buffer, without a Node API.
  if (record(data) && Array.isArray(data.data)) { data = data.data; }
  if (!Array.isArray(data)) { throw new Error("Invalid PointCloud2 byte data"); }
  integer(data.length, "data length", 0, MAX_POINT_CLOUD_BYTES);
  for (let i = 0; i < data.length; i++) {
    integer(data[i], "data byte", 0, 255);
  }
  return new Uint8Array(data);
}

function fields(value: unknown, pointStep: number): Map<string, PointField> {
  // Bound descriptor processing independently of the binary payload.
  if (!Array.isArray(value) || value.length > 1024) {
    throw new Error("Invalid PointCloud2 fields (maximum 1024)");
  }
  const result = new Map<string, PointField>();
  for (const item of value) {
    if (!record(item) || typeof item.name !== "string" || !item.name || result.has(item.name)) {
      throw new Error("Invalid or duplicate PointCloud2 field name");
    }
    const datatype = integer(item.datatype, "field datatype", 1, 8);
    const count = integer(item.count, "field count", 1, UINT32_MAX);
    const offset = integer(item.offset, "field offset", 0, pointStep);
    if (count * FIELD_BYTES[datatype] > pointStep - offset) {
      throw new Error(`PointCloud2 field ${item.name} exceeds point_step`);
    }
    result.set(item.name, { offset, datatype, count });
  }
  return result;
}

function numeric(view: DataView, base: number, field: PointField, littleEndian: boolean): number {
  // For array-valued fields, use the first component after validating the full span.
  const offset = base + field.offset;
  switch (field.datatype) {
    case 1: return view.getInt8(offset);
    case 2: return view.getUint8(offset);
    case 3: return view.getInt16(offset, littleEndian);
    case 4: return view.getUint16(offset, littleEndian);
    case 5: return view.getInt32(offset, littleEndian);
    case 6: return view.getUint32(offset, littleEndian);
    case 7: return view.getFloat32(offset, littleEndian);
    case 8: return view.getFloat64(offset, littleEndian);
    default: throw new Error("Invalid PointCloud2 field datatype");
  }
}

function channel(view: DataView, base: number, field: PointField, littleEndian: boolean): number {
  const value = numeric(view, base, field, littleEndian);
  if (!Number.isFinite(value)) { return NEUTRAL_COLOR; }
  return Math.max(0, Math.min(1, field.datatype >= 7 ? value : value / 255));
}

/**
 * Decode a PointCloud2 record, not a topic-message envelope. Malformed inputs
 * throw; nonfinite/overflowing positions or ranges are skipped, even if is_dense.
 * Colors are normalized to 0..1; rgb takes precedence over rgba, then r/g/b.
 * maxPoints must be a positive integer. Sampling includes both ends when > 1,
 * and does not replace invalid attempts. No transforms or frame lookup occur.
 */
export function decodePointCloud(message: unknown, maxPoints = MAX_RENDERED_POINTS): DecodedPointCloud {
  integer(maxPoints, "maxPoints", 1, MAX_RENDERED_POINTS);
  if (!record(message)) { throw new Error("Expected a PointCloud2 record"); }
  const width = integer(message.width, "width", 0, UINT32_MAX);
  const height = integer(message.height, "height", 0, UINT32_MAX);
  const pointStep = integer(message.point_step, "point_step", 0, MAX_POINT_CLOUD_BYTES);
  const rowStep = integer(message.row_step, "row_step", 0, MAX_POINT_CLOUD_BYTES);
  const totalPoints = integer(width * height, "total points", 0, Number.MAX_SAFE_INTEGER);
  integer(rowStep * height, "layout size", 0, MAX_POINT_CLOUD_BYTES);
  // Some cameras retain the original row_step after compacting a one-row cloud.
  // Require every point and inter-row stride, but not unused final row padding.
  const requiredBytes = totalPoints > 0 ? (height - 1) * rowStep + width * pointStep : 0;
  if ((totalPoints > 0 && pointStep === 0) || width * pointStep > rowStep) {
    throw new Error("Invalid PointCloud2 point_step or row_step");
  }
  if (message.is_bigendian !== true && message.is_bigendian !== false &&
      message.is_bigendian !== 0 && message.is_bigendian !== 1) {
    throw new Error("Invalid PointCloud2 is_bigendian");
  }
  const littleEndian = message.is_bigendian === false || message.is_bigendian === 0;
  const layout = fields(message.fields, pointStep);
  const xField = layout.get("x");
  const yField = layout.get("y");
  const zField = layout.get("z");
  if (totalPoints > 0 && (!xField || !yField || !zField)) {
    throw new Error("PointCloud2 requires x, y and z fields");
  }
  const packed = layout.get("rgb") || layout.get("rgba");
  if (packed && packed.datatype !== 6 && packed.datatype !== 7) {
    throw new Error("PointCloud2 packed color must be UINT32 or FLOAT32");
  }
  const red = layout.get("r");
  const green = layout.get("g");
  const blue = layout.get("b");
  const hasColor = !!packed || !!(red && green && blue);
  const data = bytes(message.data);
  if (data.byteLength < requiredBytes) {
    throw new Error(`Truncated PointCloud2 data: received ${data.byteLength} bytes, need ${requiredBytes} for point records`);
  }
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const sampledPoints = Math.min(totalPoints, maxPoints);
  const vertices = new Float32Array(sampledPoints * 8);
  const result: DecodedPointCloud = {
    vertices, count: 0, totalPoints, sampledPoints, skippedPoints: 0, hasColor,
    center: [0, 0, 0], radius: 1, minDepth: 0, maxDepth: 1,
    frameId: record(message.header) && typeof message.header.frame_id === "string" ? message.header.frame_id : ""
  };
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  let minDepth = Infinity, maxDepth = -Infinity;
  for (let sample = 0; sample < sampledPoints; sample++) {
    const index = sampledPoints <= 1 ? 0 : Math.floor(sample * (totalPoints - 1) / (sampledPoints - 1));
    const base = Math.floor(index / width) * rowStep + (index % width) * pointStep;
    const x = Math.fround(numeric(view, base, xField!, littleEndian));
    const y = Math.fround(numeric(view, base, yField!, littleEndian));
    const z = Math.fround(numeric(view, base, zField!, littleEndian));
    const depth = Math.fround(Math.hypot(x, y, z));
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(depth)) {
      result.skippedPoints++;
      continue;
    }
    const output = result.count++ * 8;
    vertices[output] = x;
    vertices[output + 1] = y;
    vertices[output + 2] = z;
    vertices[output + 3] = depth;
    if (packed) {
      // Reading the bits directly preserves FLOAT32 colors whose bit pattern is NaN.
      const color = view.getUint32(base + packed.offset, littleEndian);
      vertices[output + 4] = ((color >>> 16) & 255) / 255;
      vertices[output + 5] = ((color >>> 8) & 255) / 255;
      vertices[output + 6] = (color & 255) / 255;
    } else if (red && green && blue) {
      vertices[output + 4] = channel(view, base, red, littleEndian);
      vertices[output + 5] = channel(view, base, green, littleEndian);
      vertices[output + 6] = channel(view, base, blue, littleEndian);
    } else {
      vertices[output + 4] = vertices[output + 5] = vertices[output + 6] = NEUTRAL_COLOR;
    }
    vertices[output + 7] = 1;
    minX = Math.min(minX, x); maxX = Math.max(maxX, x);
    minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
    minDepth = Math.min(minDepth, depth); maxDepth = Math.max(maxDepth, depth);
  }
  // A view avoids another allocation; only count * 8 floats are exposed.
  result.vertices = vertices.subarray(0, result.count * 8);
  if (result.count > 0) {
    result.center = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2];
    let radius = 0;
    for (let i = 0; i < result.vertices.length; i += 8) {
      radius = Math.max(radius, Math.hypot(
        vertices[i] - result.center[0], vertices[i + 1] - result.center[1], vertices[i + 2] - result.center[2]
      ));
    }
    result.radius = radius || 1;
    result.minDepth = minDepth;
    result.maxDepth = maxDepth;
  }
  return result;
}