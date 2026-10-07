// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import {
  decodePointCloud,
  MAX_POINT_CLOUD_BYTES,
  MAX_RENDERED_POINTS
} from "../../src/ros/ros2/webview/point-cloud-data";

interface TestField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

interface TestCloud {
  width: number;
  height: number;
  point_step: number;
  row_step: number;
  is_bigendian: boolean | number;
  fields: TestField[];
  data: unknown;
  header?: unknown;
  is_dense?: boolean;
}

const sizes = [0, 1, 1, 2, 2, 4, 4, 4, 8];

function write(view: DataView, offset: number, datatype: number, value: number, littleEndian: boolean): void {
  switch (datatype) {
    case 1: view.setInt8(offset, value); break;
    case 2: view.setUint8(offset, value); break;
    case 3: view.setInt16(offset, value, littleEndian); break;
    case 4: view.setUint16(offset, value, littleEndian); break;
    case 5: view.setInt32(offset, value, littleEndian); break;
    case 6: view.setUint32(offset, value, littleEndian); break;
    case 7: view.setFloat32(offset, value, littleEndian); break;
    case 8: view.setFloat64(offset, value, littleEndian); break;
    default: throw new Error("Invalid fixture datatype");
  }
}

function fixture(points: number[][] = [[3, 4, 0]], datatype = 7, bigEndian = false): TestCloud {
  const size = sizes[datatype];
  const data = new Uint8Array(points.length * size * 3);
  const view = new DataView(data.buffer);
  points.forEach((point, index) => point.forEach((value, axis) => {
    write(view, (index * 3 + axis) * size, datatype, value, !bigEndian);
  }));
  return {
    width: points.length, height: 1, point_step: size * 3, row_step: data.length,
    is_bigendian: bigEndian, data,
    fields: ["x", "y", "z"].map((name, axis) => ({ name, offset: axis * size, datatype, count: 1 }))
  };
}

function colored(name: string, datatype: number, bits: number, bigEndian: boolean): TestCloud {
  const message = fixture([[3, 4, 0]], 7, bigEndian);
  const data = new Uint8Array(16);
  data.set(message.data as Uint8Array);
  new DataView(data.buffer).setUint32(12, bits, !bigEndian);
  return {
    ...message, data, point_step: 16, row_step: 16,
    fields: [...message.fields, { name, offset: 12, datatype, count: 1 }]
  };
}

function floats(actual: Float32Array, expected: number[]): void {
  assert.deepStrictEqual(Array.from(actual), expected.map(Math.fround));
}

describe("Browser PointCloud2 decoder", () => {
  it("exports the exact byte and point limits", () => {
    assert.strictEqual(MAX_POINT_CLOUD_BYTES, 32 * 1024 * 1024);
    assert.strictEqual(MAX_RENDERED_POINTS, 200000);
  });

  it("decodes base64, stride-eight vertices and single-point sensor range", () => {
    const message = fixture();
    message.data = Buffer.from(message.data as Uint8Array).toString("base64");
    message.header = { frame_id: "lidar_frame" };
    const result = decodePointCloud(message);
    floats(result.vertices, [3, 4, 0, 5, 0.5, 0.5, 0.5, 1]);
    assert.strictEqual(result.count, 1);
    assert.strictEqual(result.totalPoints, 1);
    assert.strictEqual(result.sampledPoints, 1);
    assert.strictEqual(result.skippedPoints, 0);
    assert.strictEqual(result.hasColor, false);
    assert.deepStrictEqual(result.center, [3, 4, 0]);
    assert.strictEqual(result.radius, 1);
    assert.strictEqual(result.minDepth, 5);
    assert.strictEqual(result.maxDepth, 5);
    assert.strictEqual(result.frameId, "lidar_frame");
  });

  const numericCases = [
    { datatype: 1, point: [-128, 127, -1] },
    { datatype: 2, point: [255, 128, 1] },
    { datatype: 3, point: [-32768, 32767, -1234] },
    { datatype: 4, point: [65535, 32768, 1234] },
    { datatype: 5, point: [-2147483648, 2147483647, -123456] },
    { datatype: 6, point: [4294967295, 2147483648, 123456] },
    { datatype: 7, point: [-1.5, 2.25, 0.125] },
    { datatype: 8, point: [-1.123456789, 2.25, 0.125] }
  ];
  for (const bigEndian of [false, true]) {
    for (const { datatype, point } of numericCases) {
      it(`reads PointField datatype ${datatype}, big endian ${bigEndian}`, () => {
        const result = decodePointCloud(fixture([point], datatype, bigEndian));
        const rounded = point.map(Math.fround);
        floats(result.vertices, [...rounded, Math.hypot(...rounded), 0.5, 0.5, 0.5, 1]);
        assert.deepStrictEqual(result.center, rounded);
      });
    }

    for (const bits of [0x00123456, 0x7fc12345]) {
      it(`reinterprets FLOAT32 rgb bits ${bits.toString(16)}, big endian ${bigEndian}`, () => {
        const message = colored("rgb", 7, bits, bigEndian);
        if (bits === 0x7fc12345) {
          assert.ok(Number.isNaN(new DataView((message.data as Uint8Array).buffer).getFloat32(12, !bigEndian)));
        }
        const result = decodePointCloud(message);
        assert.strictEqual(result.count, 1);
        assert.strictEqual(result.hasColor, true);
        floats(result.vertices.subarray(4), [((bits >>> 16) & 255) / 255, ((bits >>> 8) & 255) / 255, (bits & 255) / 255, 1]);
      });
    }

    it(`reads UINT32 rgba and ignores alpha, big endian ${bigEndian}`, () => {
      const result = decodePointCloud(colored("rgba", 6, 0x001234ff, bigEndian));
      assert.strictEqual(result.hasColor, true);
      floats(result.vertices.subarray(4), [0x12 / 255, 0x34 / 255, 1, 1]);
    });

    it(`reads UINT32 rgb and FLOAT32 rgba, big endian ${bigEndian}`, () => {
      for (const [name, datatype] of [["rgb", 6], ["rgba", 7]] as [string, number][]) {
        floats(decodePointCloud(colored(name, datatype, 0xffabcdef, bigEndian)).vertices.subarray(4),
          [0xab / 255, 0xcd / 255, 0xef / 255, 1]);
      }
    });

    it(`handles reordered fields, unaligned offsets, point padding and row padding, big endian ${bigEndian}`, () => {
      const data = new Uint8Array(88);
      data.fill(0xff);
      const view = new DataView(data.buffer);
      const points = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]];
      points.forEach(([x, y, z], index) => {
        const base = Math.floor(index / 2) * 44 + (index % 2) * 19;
        view.setFloat32(base + 1, z, !bigEndian);
        view.setInt16(base + 7, x, !bigEndian);
        view.setFloat64(base + 9, y, !bigEndian);
      });
      const result = decodePointCloud({
        width: 2, height: 2, point_step: 19, row_step: 44, data, is_bigendian: bigEndian ? 1 : 0,
        fields: [
          { name: "z", offset: 1, datatype: 7, count: 1 },
          { name: "unused", offset: 17, datatype: 2, count: 2 },
          { name: "x", offset: 7, datatype: 3, count: 1 },
          { name: "y", offset: 9, datatype: 8, count: 1 }
        ]
      });
      assert.strictEqual(result.count, 4);
      points.forEach((point, index) => floats(result.vertices.subarray(index * 8, index * 8 + 3), point));
      assert.deepStrictEqual(result.center, [5.5, 6.5, 7.5]);
      assert.strictEqual(result.radius, Math.hypot(4.5, 4.5, 4.5));
      assert.strictEqual(result.minDepth, Math.fround(Math.hypot(1, 2, 3)));
      assert.strictEqual(result.maxDepth, Math.fround(Math.hypot(10, 11, 12)));
    });
  }

  it("accepts Uint8Array subviews, byte arrays and buffer JSON without modifying input", () => {
    const message = fixture();
    const raw = message.data as Uint8Array;
    const storage = new Uint8Array(raw.length + 10);
    storage.fill(99);
    storage.set(raw, 5);
    const before = storage.slice();
    for (const data of [storage.subarray(5, 5 + raw.length), Array.from(raw), { type: "Buffer", data: Array.from(raw) }, { data: Array.from(raw) }]) {
      floats(decodePointCloud({ ...message, data }).vertices, [3, 4, 0, 5, 0.5, 0.5, 0.5, 1]);
    }
    assert.deepStrictEqual(storage, before);
  });

  it("accepts base64 with zero, one or two padding characters", () => {
    for (const length of [3, 4, 5]) {
      const message = fixture([[3, 4, 12]], 2);
      const data = new Uint8Array(length);
      data.set(message.data as Uint8Array, length - 3);
      const fields = message.fields.map(field => ({ ...field, offset: field.offset + length - 3 }));
      const result = decodePointCloud({ ...message, fields, point_step: length, row_step: length,
        data: Buffer.from(data).toString("base64") });
      assert.strictEqual(result.count, 1);
      floats(result.vertices.subarray(0, 4), [3, 4, 12, 13]);
    }
  });

  it("uses the first component of array fields and validates their full span", () => {
    const message = fixture();
    message.fields[0].count = 2;
    floats(decodePointCloud(message).vertices.subarray(0, 3), [3, 4, 0]);
    message.fields[0].count = 4;
    assert.throws(() => decodePointCloud(message), /exceeds point_step/);
  });

  it("filters nonfinite and overflowing float32 positions and ranges regardless of is_dense", () => {
    const message = fixture([
      [3, 4, 0], [NaN, 0, 0], [0, Infinity, 0], [0, 0, -Infinity],
      [1e100, 0, 0], [0, -1e100, 0], [3e38, 3e38, 0], [-3, -4, 0]
    ], 8);
    message.is_dense = true;
    const result = decodePointCloud(message);
    assert.strictEqual(result.count, 2);
    assert.strictEqual(result.totalPoints, 8);
    assert.strictEqual(result.sampledPoints, 8);
    assert.strictEqual(result.skippedPoints, 6);
    floats(result.vertices, [3, 4, 0, 5, 0.5, 0.5, 0.5, 1, -3, -4, 0, 5, 0.5, 0.5, 0.5, 1]);
    assert.deepStrictEqual(result.center, [0, 0, 0]);
    assert.strictEqual(result.radius, 5);
    assert.strictEqual(result.minDepth, 5);
    assert.strictEqual(result.maxDepth, 5);
  });

  it("uses an enclosing radius rather than distance to the sensor", () => {
    const result = decodePointCloud(fixture([[10, 0, 0], [14, 0, 0], [12, 1, 0]]));
    assert.deepStrictEqual(result.center, [12, 0.5, 0]);
    assert.strictEqual(result.radius, Math.hypot(2, 0.5));
    assert.strictEqual(result.minDepth, 10);
    assert.strictEqual(result.maxDepth, 14);
  });

  for (const bigEndian of [false, true]) {
    for (const datatype of [1, 2, 3, 4, 5, 6, 7, 8]) {
      it(`normalizes separate channels of datatype ${datatype}, big endian ${bigEndian}`, () => {
        const message = fixture([[3, 4, 0]], 7, bigEndian);
        const size = sizes[datatype];
        const data = new Uint8Array(12 + size * 3);
        data.set(message.data as Uint8Array);
        const view = new DataView(data.buffer);
        const values = datatype >= 7 ? [0.25, 0.5, 1] : datatype === 1 ? [0, 64, 127] : [0, 128, 255];
        const channels = ["r", "g", "b"].map((name, i) => {
          write(view, 12 + size * i, datatype, values[i], !bigEndian);
          return { name, offset: 12 + size * i, datatype, count: 1 };
        });
        const result = decodePointCloud({ ...message, data, point_step: data.length, row_step: data.length,
          fields: [...message.fields, ...channels] });
        assert.strictEqual(result.hasColor, true);
        floats(result.vertices.subarray(4), [...values.map(value => datatype >= 7 ? value : value / 255), 1]);
      });
    }
  }

  it("clamps out-of-range channels and uses neutral for nonfinite channels", () => {
    const message = fixture();
    const data = new Uint8Array(24);
    data.set(message.data as Uint8Array);
    const view = new DataView(data.buffer);
    view.setFloat32(12, -2, true);
    view.setFloat32(16, 2, true);
    view.setFloat32(20, NaN, true);
    const fields = [...message.fields, ...["r", "g", "b"].map((name, i) => ({ name, offset: 12 + i * 4, datatype: 7, count: 1 }))];
    const result = decodePointCloud({ ...message, fields, data, point_step: 24, row_step: 24 });
    floats(result.vertices.subarray(4), [0, 1, 0.5, 1]);
    assert.strictEqual(result.count, 1);
    assert.strictEqual(result.hasColor, true);
  });

  it("uses neutral color for incomplete channels and prefers packed rgb over rgba and channels", () => {
    const message = fixture();
    const incomplete = [...message.fields, { name: "r", offset: 0, datatype: 7, count: 1 }];
    const neutral = decodePointCloud({ ...message, fields: incomplete });
    assert.strictEqual(neutral.hasColor, false);
    floats(neutral.vertices.subarray(4), [0.5, 0.5, 0.5, 1]);
    const packed = colored("rgb", 6, 0x00123456, false);
    packed.fields.push({ name: "rgba", offset: 0, datatype: 6, count: 1 });
    packed.fields.push(...["r", "g", "b"].map(name => ({ name, offset: 0, datatype: 7, count: 1 })));
    floats(decodePointCloud(packed).vertices.subarray(4), [0x12 / 255, 0x34 / 255, 0x56 / 255, 1]);
  });

  it("samples across padded rows with endpoint coverage, not a prefix", () => {
    const data = new Uint8Array(34);
    for (let i = 0; i < 10; i++) { data[Math.floor(i / 5) * 17 + i % 5 * 3] = i; }
    const result = decodePointCloud({ ...fixture([], 2), data, width: 5, height: 2, row_step: 17 }, 4);
    assert.strictEqual(result.totalPoints, 10);
    assert.strictEqual(result.sampledPoints, 4);
    assert.strictEqual(result.skippedPoints, 0);
    assert.deepStrictEqual([0, 1, 2, 3].map(i => result.vertices[i * 8]), [0, 3, 6, 9]);
    assert.deepStrictEqual(result.center, [4.5, 0, 0]);
    assert.strictEqual(result.radius, 4.5);
  });

  it("counts invalid sampled attempts without backfilling and bounds only rendered samples", () => {
    const points = Array.from({ length: 10 }, (_, i) => [i, 0, 0]);
    points[3][0] = NaN;
    points[8][0] = 10000;
    const result = decodePointCloud(fixture(points), 4);
    assert.strictEqual(result.count, 3);
    assert.strictEqual(result.sampledPoints, 4);
    assert.strictEqual(result.skippedPoints, 1);
    assert.strictEqual(result.totalPoints, 10);
    assert.deepStrictEqual([0, 1, 2].map(i => result.vertices[i * 8]), [0, 6, 9]);
    assert.deepStrictEqual(result.center, [4.5, 0, 0]);
    assert.strictEqual(result.radius, 4.5);
    assert.strictEqual(result.minDepth, 0);
    assert.strictEqual(result.maxDepth, 9);
  });

  it("enforces the default output cap without allocating one vertex per input point", () => {
    const total = MAX_RENDERED_POINTS + 5;
    const data = new Uint8Array(total * 3);
    data[(total - 1) * 3] = 99;
    const result = decodePointCloud({ ...fixture([], 2), width: total, row_step: data.length, data });
    assert.strictEqual(result.count, MAX_RENDERED_POINTS);
    assert.strictEqual(result.sampledPoints, MAX_RENDERED_POINTS);
    assert.strictEqual(result.totalPoints, total);
    assert.strictEqual(result.vertices.length, MAX_RENDERED_POINTS * 8);
    assert.strictEqual(result.vertices.buffer.byteLength, MAX_RENDERED_POINTS * 8 * 4);
    assert.strictEqual(result.vertices[(result.count - 1) * 8], 99);
  });

  it("supports a one-point cap and caps attempts at the input size", () => {
    const message = fixture([[1, 2, 3], [4, 5, 6]]);
    const one = decodePointCloud(message, 1);
    assert.strictEqual(one.sampledPoints, 1);
    floats(one.vertices.subarray(0, 3), [1, 2, 3]);
    assert.strictEqual(decodePointCloud(message, 100).sampledPoints, 2);
  });

  for (const maxPoints of [0, -1, 1.5, NaN, Infinity, MAX_RENDERED_POINTS + 1, "10", null]) {
    it(`rejects invalid maxPoints ${String(maxPoints)}`, () => {
      assert.throws(() => decodePointCloud(fixture(), maxPoints as number), /maxPoints/);
    });
  }

  it("returns safe defaults for empty and all-invalid clouds", () => {
    for (const message of [
      fixture([]),
      { ...fixture([]), height: 0, point_step: 0, fields: [], data: "" },
      { ...fixture([]), width: 3, height: 0, row_step: 36 },
      fixture([[NaN, 0, 0], [0, Infinity, 0]])
    ]) {
      const result = decodePointCloud(message);
      assert.strictEqual(result.count, 0);
      assert.strictEqual(result.vertices.length, 0);
      assert.deepStrictEqual(result.center, [0, 0, 0]);
      assert.strictEqual(result.radius, 1);
      assert.strictEqual(result.minDepth, 0);
      assert.strictEqual(result.maxDepth, 1);
      assert.strictEqual(result.sampledPoints, result.skippedPoints);
      assert.strictEqual(result.totalPoints, message.width * message.height);
    }
  });

  it("handles the origin as a valid point with zero sensor range", () => {
    const result = decodePointCloud(fixture([[0, 0, 0]]));
    assert.strictEqual(result.count, 1);
    assert.strictEqual(result.minDepth, 0);
    assert.strictEqual(result.maxDepth, 0);
    assert.strictEqual(result.radius, 1);
  });

  it("only copies a plain string header.frame_id, without coercion", () => {
    for (const header of [undefined, null, "map", [], { frame_id: 123 }, { frame_id: { data: "map" } }, { frame_id: new String("map") }]) {
      assert.strictEqual(decodePointCloud({ ...fixture(), header }).frameId, "");
    }
    assert.strictEqual(decodePointCloud({ ...fixture(), header: { frame_id: "<map>" } }).frameId, "<map>");
  });

  it("rejects envelopes and nonrecords", () => {
    for (const message of [null, undefined, 42, "cloud", [], { data: fixture(), timestamp: 1 }]) {
      assert.throws(() => decodePointCloud(message), /PointCloud2/);
    }
  });

  const badLayouts: Record<string, unknown>[] = [
    { width: -1 }, { width: 1.5 }, { width: "1" }, { width: 0x100000000 },
    { height: -1 }, { height: NaN }, { height: Infinity },
    { width: 0xffffffff, height: 0xffffffff },
    { point_step: 0 }, { point_step: -1 }, { point_step: 11 }, { point_step: 12.5 },
    { row_step: 11 }, { row_step: -1 }, { row_step: 12.5 },
    { row_step: MAX_POINT_CLOUD_BYTES, height: 2 },
    { point_step: MAX_POINT_CLOUD_BYTES + 1 }, { row_step: MAX_POINT_CLOUD_BYTES + 1 },
    { is_bigendian: "false" }, { is_bigendian: 2 }, { is_bigendian: null }, { is_bigendian: undefined }
  ];
  badLayouts.forEach((layout, i) => {
    it(`rejects malformed or excessive layout ${i}`, () => {
      assert.throws(() => decodePointCloud({ ...fixture(), ...layout }), /PointCloud2/);
    });
  });

  it("validates all fields, including unused descriptors", () => {
    const message = fixture();
    const invalid = [
      { name: "" }, { name: 1 }, { name: "x" },
      { datatype: 0 }, { datatype: 9 }, { datatype: 1.5 }, { datatype: "7" },
      { count: 0 }, { count: -1 }, { count: 0.5 }, { count: NaN }, { count: undefined },
      { count: Number.MAX_SAFE_INTEGER }, { count: 2 },
      { offset: -1 }, { offset: 0.5 }, { offset: Infinity }, { offset: 12 }, { offset: 13 }
    ];
    for (const change of invalid) {
      const descriptor = { name: "unused", offset: 8, datatype: 7, count: 1, ...change };
      assert.throws(() => decodePointCloud({ ...message, fields: [...message.fields, descriptor] }), /PointCloud2/);
    }
    for (const descriptor of [null, {}, undefined]) {
      assert.throws(() => decodePointCloud({ ...message, fields: [...message.fields, descriptor] }), /PointCloud2/);
    }
    for (const fields of [undefined, null, {}, [], message.fields.slice(0, 2), new Array(1025)]) {
      assert.throws(() => decodePointCloud({ ...message, fields }), /PointCloud2/);
    }
    for (const datatype of [1, 2, 3, 4, 5, 8]) {
      assert.throws(() => decodePointCloud({ ...message,
        fields: [...message.fields, { name: "rgb", offset: 0, datatype, count: 1 }] }), /packed color/);
    }
  });

  it("accepts a compact camera cloud with a stale larger row_step", () => {
    const message = fixture();
    message.width = 349825;
    message.point_step = 20;
    message.row_step = 8140800;
    message.data = new Uint8Array(6996500);
    const result = decodePointCloud(message);
    assert.strictEqual(result.totalPoints, 349825);
    assert.strictEqual(result.count, MAX_RENDERED_POINTS);
  });

  it("allows omitted final row padding without changing organized row offsets", () => {
    const data = new Uint8Array(28);
    const view = new DataView(data.buffer);
    [1, 2, 3].forEach((value, axis) => view.setFloat32(axis * 4, value, true));
    [4, 5, 6].forEach((value, axis) => view.setFloat32(16 + axis * 4, value, true));
    const message = { ...fixture(), height: 2, row_step: 16, data };
    const result = decodePointCloud(message);
    assert.deepStrictEqual(Array.from(result.vertices.slice(0, 3)), [1, 2, 3]);
    assert.deepStrictEqual(Array.from(result.vertices.slice(8, 11)), [4, 5, 6]);
    assert.throws(() => decodePointCloud({ ...message, data: data.subarray(0, 27) }, 1), /Truncated/);
    assert.throws(() => decodePointCloud({ ...message, data: data.subarray(0, 24) }), /Truncated/);
  });

  it("rejects truncated point records even in unsampled rows", () => {
    const message = fixture([[1, 2, 3], [4, 5, 6]]);
    const data = message.data as Uint8Array;
    for (const short of [data.subarray(0, 23), Array.from(data.subarray(0, 1)), Buffer.from(data.subarray(0, 12)).toString("base64")]) {
      assert.throws(() => decodePointCloud({ ...message, data: short }, 1), /Truncated/);
    }
    assert.strictEqual(decodePointCloud({ ...fixture(), row_step: 13 }).count, 1);
    assert.throws(() => decodePointCloud({ ...fixture(), height: 2 }, 1), /Truncated/);
  });

  it("rejects malformed bytes rather than silently coercing them", () => {
    for (const bad of [-1, 256, 1.5, NaN, Infinity, "1", null, undefined, true]) {
      const data: unknown[] = new Array(12).fill(0);
      data[11] = bad;
      for (const representation of [data, { data }]) {
        assert.throws(() => decodePointCloud({ ...fixture(), data: representation }), /data byte/);
      }
    }
    for (const data of [new Array(12), null, undefined, new Uint16Array(12), new ArrayBuffer(12), { data: "AAAA" }, {}]) {
      assert.throws(() => decodePointCloud({ ...fixture(), data }), /PointCloud2/);
    }
  });

  it("rejects invalid base64 alphabet, padding, lengths and noncanonical pad bits", () => {
    for (const data of ["A", "AAA", "AAAAA", "====", "A===", "AA=A", "AA!A", "AA-A", "AA_A", "AAAA\n", " AA=", "AB==", "AAB=", "AAéA", "data:;base64,AAAA"]) {
      assert.throws(() => decodePointCloud({ ...fixture([]), data }), /base64|data length/);
    }
  });

  it("rejects oversized typed data and array lengths before reading or allocating bytes", () => {
    assert.throws(() => decodePointCloud({ ...fixture(), data: new Uint8Array(MAX_POINT_CLOUD_BYTES + 1) }), /data length/);
    // Simulate an enormous array without allocating hundreds of MiB in the test.
    const data = new Proxy([], {
      get: (_target, key) => {
        if (key === "length") { return MAX_POINT_CLOUD_BYTES + 1; }
        throw new Error("Oversized array elements must not be accessed");
      }
    });
    assert.throws(() => decodePointCloud({ ...fixture(), data }), /data length/);
    assert.throws(() => decodePointCloud({ ...fixture(), data: { data } }), /data length/);
  });

  it("checks encoded and decoded base64 limits before allocation or alphabet scanning", () => {
    const maxEncodedLength = Math.ceil(MAX_POINT_CLOUD_BYTES / 3) * 4;
    // Invalid alphabet proves size validation occurs before character validation.
    assert.throws(() => decodePointCloud({ ...fixture(), data: "!".repeat(maxEncodedLength + 4) }), /oversized/);
    // This encoded size could fit with padding, but without it decodes to 32 MiB + 1.
    assert.throws(() => decodePointCloud({ ...fixture(), data: "!".repeat(maxEncodedLength) }), /data length/);
  });
});