const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const Module = require('node:module');
const path = require('node:path');
const { PassThrough } = require('node:stream');
const { test } = require('node:test');

function load(filename, mocks) {
  const resolved = require.resolve(filename);
  const original = Module._load;
  delete require.cache[resolved];
  Module._load = function(name, parent, main) {
    if (parent?.filename === resolved && Object.hasOwn(mocks, name)) { return mocks[name]; }
    return original.call(this, name, parent, main);
  };
  try { return require(resolved); }
  finally { Module._load = original; delete require.cache[resolved]; }
}

const tick = () => new Promise(resolve => setImmediate(resolve));
const cloudType = 'sensor_msgs/msg/PointCloud2';
const frame = {
  header: { frame_id: 'lidar 🚀', stamp: { sec: -12, nanosec: 345 } },
  width: 2, height: 2, point_step: 32, row_step: 72, is_bigendian: true, is_dense: false,
  fields: [{ name: 'normal', offset: 8, datatype: 7, count: 3 },
    { name: 'intensity', offset: 0, datatype: 4, count: 1 },
    { name: 'label', offset: 24, datatype: 1, count: 4 }],
  data: Buffer.from(Array.from({ length: 144 }, (_, i) => i)),
};

function header(metadataLength, payloadLength, magic = 'RDEB') {
  const result = Buffer.alloc(12);
  result.write(magic, 0, 4, 'ascii');
  result.writeUInt32LE(metadataLength, 4); result.writeUInt32LE(payloadLength, 8);
  return result;
}

function binaryFrame({ data, ...metadata }) {
  const json = Buffer.from(JSON.stringify(metadata), 'utf8');
  return Buffer.concat([header(json.length, data.length), json, data]);
}

function metadataFrame(text) {
  const json = Buffer.from(text, 'utf8');
  return Buffer.concat([header(json.length, 0), json]);
}

function harness() {
  const calls = [], logs = [], children = [];
  const env = { CONDA_PREFIX: 'C:\\pixi\\lyrical', ROS_VERSION: '2', CUSTOM: 'retained' };
  const extension = { extPath: path.resolve('extension path & tools'),
    resolvedEnv: async () => env, outputChannel: { appendLine: s => logs.push(s) } };
  const python = { resolveRosPython: async e => {
    assert.equal(e, env); return 'C:\\pixi\\lyrical\\python.exe';
  } };
  const child_process = { spawn: (executable, args, options) => {
    const child = new EventEmitter();
    child.stdout = new PassThrough(); child.stderr = new PassThrough(); child.stdin = new PassThrough();
    child.controls = []; child.stdin.on('data', data => child.controls.push(data.toString()));
    child.kill = () => { child.killed = true; return true; };
    calls.push({ executable, args, options }); children.push(child);
    return child;
  } };
  const transport = load('../out/src/ros/ros2/image-subscription', {
    '../../extension': extension, '../python': python, child_process,
  });
  return { ...transport, calls, logs, children, env, extension, python, child_process };
}

test('PointCloud2 starts direct ROS Python with exact arguments and default 1 Hz', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager(), received = [];
  const topic = '/points & literal';
  manager.start(topic, cloudType, message => received.push(message));
  assert.equal(manager.has(topic), true);
  await tick();
  assert.deepEqual(h.calls, [{ executable: 'C:\\pixi\\lyrical\\python.exe', args: [
    '-u', path.join(h.extension.extPath, 'assets/scripts/image_subscriber.py'),
    '--topic', topic, '--type', cloudType, '--rate', '1',
  ], options: { env: h.env, stdio: ['pipe', 'pipe', 'pipe'], windowsHide: true } }]);
  const before = Date.now();
  assert.equal(h.children[0].stdout.readableEncoding, null);
  assert.equal(h.children[0].stderr.readableEncoding, 'utf8');
  h.children[0].stdout.write(binaryFrame(frame));
  assert.deepEqual(received[0].data, frame);
  assert.ok(received[0].timestamp >= before && received[0].timestamp <= Date.now());
  assert.match(h.logs[0], /PointCloud2 subscription.*1 Hz preview/);
  manager.dispose();
});

test('cloud default, finite clamps and invalid-rate fallback apply to start and controls', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager();
  const cases = [[undefined, 1], [NaN, 1], [Infinity, 1], [-Infinity, 1],
    [-10, 0.2], [0, 0.2], [0.2, 0.2], [0.5, 0.5], [5, 5], [30, 5]];
  for (const [rate, expected] of cases) {
    manager.start('/points', cloudType, () => {}, rate); await tick();
    assert.equal(h.calls.at(-1).args.at(-1), String(expected));
  }
  const child = h.children.at(-1), count = h.calls.length;
  for (const [rate] of cases) { manager.setRate('/points', rate); }
  assert.deepEqual(child.controls.map(line => JSON.parse(line).rateHz), cases.map(([, expected]) => expected));
  assert.ok(child.controls.every(line => line.endsWith('\n')));
  assert.equal(h.calls.length, count, 'Controls must not restart DDS discovery');
  manager.setRate('/absent', 5);
  child.stdin.destroy(); manager.setRate('/points', 2);
  assert.equal(child.controls.length, cases.length, 'No write to a closed control pipe');
  manager.dispose();
});

test('cloud replacement, stop/restart, pending rate and stale events use shared lifecycle', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager(), received = [];
  manager.start('/points', cloudType, message => received.push(message)); await tick();
  const old = h.children[0];
  manager.start('/points', cloudType, message => received.push(message));
  manager.setRate('/points', 0.2); await tick();
  assert.ok(old.killed); assert.ok(old.stdin.writableEnded);
  assert.equal(h.calls[1].args.at(-1), '0.2');
  const logCount = h.logs.length;
  old.stdout.write(binaryFrame(frame)); old.stderr.write('stale');
  old.emit('error', new Error('stale')); old.emit('close', 1);
  old.stdin.emit('error', new Error('stale'));
  assert.equal(h.logs.length, logCount);
  assert.equal(manager.has('/points'), true);
  assert.deepEqual(received, []);
  h.children[1].stdout.write(binaryFrame(frame));
  assert.equal(received.length, 1);
  manager.stop('/points');
  assert.ok(h.children[1].killed); assert.equal(manager.has('/points'), false);
  manager.start('/points', cloudType, () => {}); await tick();
  assert.equal(h.calls[2].args.at(-1), '1');
  manager.start('/other', cloudType, () => {}); await tick();
  manager.dispose();
  assert.ok(h.children.every(child => child.killed));
  assert.equal(manager.has('/other'), false);
});

test('cloud stop/dispose cancels pending environment and Python resolution', async () => {
  for (const phase of ['environment', 'python']) {
    for (const action of ['stop', 'dispose']) {
      const h = harness(), manager = new h.PointCloudSubscriptionManager();
      let resolve;
      const pending = new Promise(r => { resolve = r; });
      if (phase === 'environment') { h.extension.resolvedEnv = () => pending; }
      else { h.python.resolveRosPython = () => pending; }
      manager.start('/points', cloudType, () => assert.fail('Cancelled')); await tick();
      manager[action]('/points');
      resolve(phase === 'environment' ? h.env : 'python'); await tick();
      assert.equal(h.calls.length, 0);
      assert.equal(manager.has('/points'), false);
    }
  }
});

test('cloud pending replacement ignores a rejected stale activation', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager();
  let reject;
  const pending = new Promise((_, r) => { reject = r; });
  h.extension.resolvedEnv = () => pending;
  manager.start('/points', cloudType, () => {});
  h.extension.resolvedEnv = async () => h.env;
  manager.start('/points', cloudType, () => {}, 2); await tick();
  const logCount = h.logs.length;
  reject(new Error('old activation')); await tick();
  assert.equal(h.logs.length, logCount);
  assert.equal(h.calls.length, 1);
  assert.equal(manager.has('/points'), true);
  manager.dispose();
});

test('cloud binary frames deliver large chunked payloads, fields, endian and padding unchanged', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager(), received = [];
  manager.start('/points', cloudType, message => received.push(message)); await tick();
  const raw = Buffer.alloc(16 * 1024 * 1024, 0xa5);
  const large = { ...frame, width: 262143, height: 2, row_step: raw.length / 2, data: raw };
  const encoded = Buffer.concat([binaryFrame(large), binaryFrame(frame)]);
  // Split the header and UTF-8 metadata, then chunk the raw body.
  for (let i = 0; i < 128; i++) { h.children[0].stdout.write(encoded.subarray(i, i + 1)); }
  for (let i = 128; i < encoded.length; i += 65521) {
    h.children[0].stdout.write(encoded.subarray(i, i + 65521));
  }
  assert.deepEqual(received.map(message => message.data), [large, frame]);
  assert.ok(Buffer.isBuffer(received[0].data.data));
  assert.deepEqual(received[0].data.data, raw);
  manager.dispose();
});

test('cloud malformed/oversized frames and activation/pipe/process errors stop without payload logs', async () => {
  for (const failure of ['activation', 'python', 'spawn', 'error', 'close', 'stdin', 'json', 'schema',
    'oversize', 'metadata-size', 'magic', 'fields', 'wrong-type']) {
    const h = harness(), manager = new h.PointCloudSubscriptionManager();
    if (failure === 'activation') { h.extension.resolvedEnv = async () => { throw new Error('bad ROS'); }; }
    if (failure === 'python') { h.python.resolveRosPython = async () => { throw new Error('bad Python'); }; }
    if (failure === 'spawn') { h.child_process.spawn = () => { throw new Error('spawn failed'); }; }
    manager.start('/points', cloudType, () => assert.fail('Invalid frame')); await tick();
    const child = h.children[0];
    if (failure === 'error') { child.emit('error', new Error('failed')); }
    if (failure === 'close') { child.emit('close', 1); }
    if (failure === 'stdin') { child.stdin.emit('error', new Error('EPIPE')); }
    if (failure === 'json') { child.stdout.write(metadataFrame('private-cloud-payload')); }
    if (failure === 'schema') { child.stdout.write(metadataFrame('{"data":[]}')); }
    if (failure === 'oversize') { child.stdout.write(header(2, 32 * 1024 * 1024 + 1)); }
    if (failure === 'metadata-size') { child.stdout.write(header(65537, 0)); }
    if (failure === 'magic') { child.stdout.write(header(2, 0, 'NOPE')); }
    if (failure === 'fields') { child.stdout.write(binaryFrame({ ...frame, fields: [{ name: 'x', offset: -1 }] })); }
    if (failure === 'wrong-type') { child.stdout.write(binaryFrame({ header: frame.header, format: 'jpeg', data: frame.data })); }
    assert.equal(manager.has('/points'), false, failure);
    assert.ok(h.logs.length > 0);
    assert.ok(!h.logs.join('\n').includes('private-cloud-payload'));
    if (child && failure !== 'close') { assert.ok(child.killed, failure); }
  }
});

test('shared transport preserves image exports, rates, controls and exact image diagnostics', async () => {
  const h = harness(), images = new h.ImageSubscriptionManager(), clouds = new h.PointCloudSubscriptionManager();
  assert.equal(typeof h.ImageFrameDecoder, 'function');
  const image = { header: frame.header, width: 1, height: 1, step: 3, encoding: 'rgb8', is_bigendian: 0, data: Buffer.from([0, 1, 2]) };
  const received = [];
  images.start('/shared', 'sensor_msgs/msg/Image', message => received.push(message));
  clouds.start('/shared', cloudType, () => {}); await tick();
  assert.equal(h.calls[0].args.at(-1), '5');
  assert.equal(h.calls[1].args.at(-1), '1');
  assert.equal(h.logs[0], 'Image subscription /shared: C:\\pixi\\lyrical\\python.exe (5 Hz preview)');
  for (const rate of [-10, Infinity, 100]) { images.setRate('/shared', rate); }
  assert.deepEqual(h.children[0].controls.map(line => JSON.parse(line).rateHz), [1, 5, 30]);
  h.children[0].stdout.write(binaryFrame(image));
  assert.deepEqual(received[0].data, image);
  h.children[0].stderr.write('diagnostic\n');
  assert.equal(h.logs.at(-1), 'Image subscriber /shared: diagnostic');
  h.children[0].stdout.write(header(2, 0, 'NOPE'));
  assert.equal(h.logs.at(-1), 'Image subscription /shared: Invalid or oversized image frame received; subscription stopped.');
  assert.equal(images.has('/shared'), false);
  assert.equal(clouds.has('/shared'), true, 'Each manager owns its sessions');
  images.start('/compressed', 'sensor_msgs/msg/CompressedImage', () => {}, 30); await tick();
  assert.equal(h.calls.at(-1).args.at(-1), '30');
  h.children.at(-1).emit('close', 0);
  assert.equal(h.logs.at(-1), 'Image subscription /compressed exited (0).');
  images.dispose(); clouds.dispose();
});

test('cloud one-byte chunks preserve Unicode and empty payloads without confusing frame boundaries', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager(), received = [];
  manager.start('/points', cloudType, message => received.push(message)); await tick();
  const empty = { ...frame, width: 0, height: 0, row_step: 0, data: Buffer.alloc(0) };
  const encoded = Buffer.concat([binaryFrame(empty), binaryFrame(frame), binaryFrame(empty)]);
  for (let i = 0; i < encoded.length; i++) { h.children[0].stdout.write(encoded.subarray(i, i + 1)); }
  assert.deepEqual(received.map(message => message.data), [empty, frame, empty]);
  manager.dispose();
});

test('cloud stop discards incomplete frames and a resumed stream starts with a fresh decoder', async () => {
  const h = harness(), manager = new h.PointCloudSubscriptionManager(), received = [];
  const encoded = binaryFrame(frame);
  manager.start('/points', cloudType, message => received.push(message)); await tick();
  h.children[0].stdout.write(encoded.subarray(0, -1));
  assert.deepEqual(received, []);
  manager.stop('/points');
  manager.start('/points', cloudType, message => received.push(message)); await tick();
  h.children[0].stdout.write(encoded.subarray(-1));
  h.children[1].stdout.write(encoded);
  assert.deepEqual(received.map(message => message.data), [frame]);
  manager.dispose();
});