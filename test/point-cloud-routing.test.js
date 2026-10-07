const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const { readFileSync } = require('node:fs');
const Module = require('node:module');
const path = require('node:path');
const { PassThrough } = require('node:stream');
const { test } = require('node:test');
const { decodePointCloud } = require('../out/src/ros/ros2/webview/point-cloud-data');

function load(filename, mocks) {
  const resolved = require.resolve(filename);
  const loaded = new Module(resolved, module);
  loaded.filename = resolved;
  loaded.paths = Module._nodeModulePaths(path.dirname(resolved));
  loaded.require = name => Object.hasOwn(mocks, name) ? mocks[name] : Module.prototype.require.call(loaded, name);
  loaded._compile(readFileSync(resolved, 'utf8'), resolved);
  return loaded.exports;
}

const cloudType = 'sensor_msgs/msg/PointCloud2';
const tick = () => new Promise(resolve => setImmediate(resolve));

function harness() {
  const calls = [], sent = [], logs = [];
  let receive;
  const extension = { extPath: process.cwd(), env: { ROS_VERSION: '2' },
    resolvedEnv: async () => extension.env, outputChannel: { appendLine: text => logs.push(text) } };
  const childProcess = { execFile() {}, spawn(executable, args, options) {
    const child = new EventEmitter();
    child.stdout = new PassThrough(); child.stderr = new PassThrough(); child.stdin = new PassThrough();
    child.kill = () => { child.killed = true; return true; };
    calls.push({ executable, args, options, child });
    return child;
  } };
  const transport = load('../out/src/ros/ros2/image-subscription', {
    '../../extension': extension, '../python': { resolveRosPython: async () => 'ros-python' },
    child_process: childProcess,
  });
  const monitor = load('../out/src/ros/ros2/topic-monitor', {
    '../../extension': extension, './image-subscription': transport, child_process: childProcess,
  });
  const panel = { reveal() {}, dispose() {}, onDidDispose() {}, webview: {
    cspSource: 'test:', html: '', asWebviewUri: uri => uri,
    postMessage: message => { sent.push(message); return Promise.resolve(true); },
    onDidReceiveMessage: callback => { receive = callback; },
  } };
  const webview = load('../out/src/ros/ros2/topic-webview', {
    './topic-monitor': monitor, vscode: { ViewColumn: { Beside: 2 },
      Uri: { joinPath: (...parts) => parts.join('/') },
      window: { createWebviewPanel: () => panel } },
  });
  return { ...monitor, ...webview, calls, sent, logs, receive: message => receive(message) };
}

function frame(bigendian) {
  const data = Buffer.alloc(32, 0xff);
  for (const [offset, value] of [[0, 3], [4, 4], [8, 0], [16, 0], [20, 0], [24, 5]]) {
    if (bigendian) { data.writeFloatBE(value, offset); }
    else { data.writeFloatLE(value, offset); }
  }
  return {
    header: { frame_id: 'lidar', stamp: { sec: 1, nanosec: 2 } },
    width: 1, height: 2, point_step: 12, row_step: 16, is_bigendian: bigendian, is_dense: true,
    fields: ['x', 'y', 'z'].map((name, i) => ({ name, offset: i * 4, datatype: 7, count: 1 })),
    data,
  };
}

function encode({ data, ...metadata }) {
  const json = Buffer.from(JSON.stringify(metadata));
  const header = Buffer.alloc(12);
  header.write('RDEB'); header.writeUInt32LE(json.length, 4); header.writeUInt32LE(data.length, 8);
  return Buffer.concat([header, json, data]);
}

test('PointCloud2 pipe reaches the browser decoder as binary with endian and row padding intact', async () => {
  const h = harness();
  const manager = new h.TopicWebviewManager({ extensionUri: 'test:', subscriptions: [] });
  try {
    manager.openTopicMonitor('/points', cloudType);
    await tick();
    assert.equal(h.calls.length, 1);
    assert.equal(h.calls[0].executable, 'ros-python');
    assert.equal(h.calls[0].args.at(-1), '1');
    for (const bigendian of [false, true]) {
      const cloud = frame(bigendian), wire = encode(cloud);
      for (let i = 0; i < wire.length; i += 7) {
        h.calls[0].child.stdout.write(wire.subarray(i, i + 7));
      }
      const delivered = h.sent.at(-1).message.data;
      assert.equal(delivered.data.constructor, Uint8Array);
      assert.deepEqual(Buffer.from(delivered.data), cloud.data);
      assert.deepEqual({ ...delivered, data: cloud.data }, cloud);
      const decoded = decodePointCloud(structuredClone(delivered));
      assert.equal(decoded.count, 2);
      assert.equal(decoded.frameId, 'lidar');
      assert.deepEqual(Array.from(decoded.vertices), [
        3, 4, 0, 5, 0.5, 0.5, 0.5, 1, 0, 0, 5, 5, 0.5, 0.5, 0.5, 1,
      ]);
    }
    h.receive({ command: 'getHistory' });
    assert.equal(h.sent.at(-1).messages.length, 1);
    assert.ok(h.sent.at(-1).messages[0].data.data instanceof Uint8Array);
    h.receive({ command: 'setRefreshRate', rateHz: 0.2 });
    assert.deepEqual(JSON.parse(h.calls[0].child.stdin.read().toString()), { rateHz: 0.2 });
    h.receive({ command: 'pause' });
    assert.ok(h.calls[0].child.killed);
    h.receive({ command: 'resume' });
    await tick();
    assert.equal(h.calls[1].args.at(-1), '0.2');
  } finally { manager.dispose(); }
  assert.ok(h.calls.every(call => call.child.killed));
});

test('binary webview preparation reuses dedicated storage and isolates buffer slices', () => {
  const h = harness();
  for (const raw of [Buffer.alloc(32 * 1024 * 1024, 0xa5), new Uint8Array([1, 2, 3]),
    Buffer.from([99, 1, 2, 3, 88]).subarray(1, 4), Buffer.alloc(0)]) {
    const { data } = h.prepareTopicMessage({ timestamp: 1, data: { data: raw } }, cloudType);
    assert.equal(data.data.constructor, Uint8Array);
    assert.deepEqual(Buffer.from(data.data), raw instanceof Buffer ? raw : Buffer.from(raw));
    assert.equal(data.data.buffer.byteLength, raw.byteLength);
    if (raw.byteLength === raw.buffer.byteLength) { assert.equal(data.data.buffer, raw.buffer); }
  }
  const oversized = h.prepareTopicMessage({
    timestamp: 1, data: { data: Buffer.alloc(32 * 1024 * 1024 + 1) },
  }, cloudType);
  assert.match(oversized.data.previewError, /32 MiB/);
});

test('routing switches cleanly between binary sensors and ordinary YAML and disposes all processes', async () => {
  const h = harness(), manager = new h.TopicEchoManager(), received = [];
  try {
    for (const type of [cloudType, 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'std_msgs/msg/String']) {
      manager.startEcho('/same', type === 'std_msgs/msg/String' ? m => received.push(m) : () => {}, type);
      await tick();
      assert.ok(manager.isEchoing('/same'));
      assert.ok(h.calls.slice(0, -1).every(call => call.child.killed));
      assert.equal(h.calls.at(-1).executable, type === 'std_msgs/msg/String' ? 'ros2' : 'ros-python');
    }
    const yaml = h.calls.at(-1);
    assert.deepEqual(yaml.args, ['topic', 'echo', '/same', '--full-length']);
    yaml.child.stdout.write("data: hello\n---\n");
    assert.deepEqual(received.map(message => message.data), [{ data: 'hello' }]);
    manager.startEcho('/cloud', () => {}, cloudType);
    manager.startEcho('/image', () => {}, 'sensor_msgs/msg/Image');
    await tick();
  } finally { manager.dispose(); }
  assert.ok(h.calls.every(call => call.child.killed));
  for (const topic of ['/same', '/cloud', '/image']) { assert.equal(manager.isEchoing(topic), false); }
});
