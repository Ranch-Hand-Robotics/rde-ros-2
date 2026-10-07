const assert = require('node:assert/strict');
const { execFile } = require('node:child_process');
const { EventEmitter } = require('node:events');
const Module = require('node:module');
const path = require('node:path');
const { PassThrough } = require('node:stream');
const { test } = require('node:test');
const vm = require('node:vm');

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
const imageType = 'sensor_msgs/msg/Image';
const compressedType = 'sensor_msgs/msg/CompressedImage';
const frame = { width: 2, height: 1, step: 6, encoding: 'rgb8', is_bigendian: 0,
  header: { frame_id: 'camera', stamp: { sec: 1, nanosec: 2 } }, data: Buffer.from([10, 20, 30, 40, 50, 60]) };
const webFrame = { ...frame, data: frame.data.toString('base64') };

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

function metadataFrame(text, payloadLength = 0) {
  const json = Buffer.isBuffer(text) ? text : Buffer.from(text, 'utf8');
  return Buffer.concat([header(json.length, payloadLength), json]);
}

function harness() {
  const calls = [], logs = [], children = [];
  const env = { CONDA_PREFIX: 'C:\\pixi\\lyrical', ROS_VERSION: '2', CUSTOM: 'retained' };
  const extension = { extPath: path.resolve('extension path & tools'), env,
    resolvedEnv: async () => env, outputChannel: { appendLine: s => logs.push(s) } };
  const python = { resolveRosPython: async e => { assert.equal(e, env); return 'C:\\pixi\\lyrical\\python.exe'; } };
  const child_process = { execFile, spawn: (executable, args, options) => {
    const child = new EventEmitter();
    child.stdout = new PassThrough(); child.stderr = new PassThrough(); child.stdin = new PassThrough();
    child.controls = []; child.stdin.on('data', data => child.controls.push(data.toString()));
    child.kill = () => { child.killed = true; return true; };
    calls.push({ executable, args, options }); children.push(child);
    return child;
  } };
  const image = load('../out/src/ros/ros2/image-subscription', { '../../extension': extension, '../python': python, child_process });
  return { ...image, calls, logs, children, env, extension, python, child_process };
}

test('decoder preserves multi-MB frames split into arbitrary chunks and multiple documents', () => {
  const { ImageFrameDecoder } = harness();
  const large = { ...frame, data: Buffer.alloc(1280 * 720 * 3, 42) };
  const text = Buffer.concat([binaryFrame(large), binaryFrame(frame)]);
  const decoder = new ImageFrameDecoder(), results = [];
  for (let i = 0; i < text.length; i += 65521) { decoder.push(text.subarray(i, i + 65521), d => results.push(d)); }
  assert.deepEqual(results, [large, frame]);
  text.fill(0);
  assert.deepEqual(results, [large, frame], 'Decoded frames own their payload, not the input chunks');
});

test('decoder waits for the last payload byte and accepts empty payloads and back-to-back frames', () => {
  const { ImageFrameDecoder } = harness();
  const decoder = new ImageFrameDecoder(), frames = [];
  const encoded = binaryFrame(frame);
  decoder.push(encoded.subarray(0, -1), d => frames.push(d));
  assert.equal(frames.length, 0);
  decoder.push(encoded.subarray(-1), d => frames.push(d));
  assert.equal(frames.length, 1);
  const empty = { ...frame, data: Buffer.alloc(0) };
  decoder.push(Buffer.concat([binaryFrame(empty), binaryFrame(empty), encoded]), d => frames.push(d));
  decoder.push(Buffer.alloc(0), () => assert.fail('Empty input'));
  assert.deepEqual(frames, [frame, empty, empty, frame]);
});

test('decoder rejects declared oversize lengths and bad magic before allocating any frame buffers', () => {
  const { ImageFrameDecoder } = harness();
  const inputs = [header(65537, 0), header(1, 32 * 1024 * 1024 + 1),
    header(0xffffffff, 0xffffffff), header(0, 0), header(2, 0, 'NOPE')];
  for (const input of inputs) {
    const decoder = new ImageFrameDecoder();
    const allocate = Buffer.allocUnsafe;
    let allocations = 0;
    Buffer.allocUnsafe = size => { allocations++; return allocate(size); };
    try { assert.throws(() => decoder.push(input, () => assert.fail('Invalid')), /transport limit/); }
    finally { Buffer.allocUnsafe = allocate; }
    assert.equal(allocations, 0);
    assert.throws(() => decoder.push(binaryFrame(frame), () => assert.fail('No recovery')));
  }
});

test('decoder rejects malformed JSON, UTF-8 and metadata before payload allocation without leaking data', () => {
  const { ImageFrameDecoder } = harness();
  const invalid = ['private-camera-payload', 'null', '[]', '{}', '42', '{"data":[]}',
    JSON.stringify({ ...webFrame }), JSON.stringify({ ...frame, data: undefined, width: -1 }),
    JSON.stringify({ ...frame, data: undefined, header: { frame_id: 42 } }),
    JSON.stringify({ ...frame, data: undefined, step: '6' }), Buffer.from([0xff, 0xfe])];
  for (const text of invalid) {
    const input = metadataFrame(text, 32 * 1024 * 1024), decoder = new ImageFrameDecoder();
    const allocate = Buffer.allocUnsafe, allocations = [];
    Buffer.allocUnsafe = size => { allocations.push(size); return allocate(size); };
    try {
      assert.throws(() => decoder.push(input, () => assert.fail('Invalid')), error => {
        assert.ok(!error.message.includes('private-camera-payload')); return true;
      });
    } finally { Buffer.allocUnsafe = allocate; }
    assert.deepEqual(allocations, [input.length - 12], 'Only bounded metadata is allocated');
  }
  assert.throws(() => new ImageFrameDecoder().push('not a Buffer', () => {}));
  const compressed = { header: frame.header, format: 'jpeg', data: Buffer.alloc(0) };
  assert.throws(() => new ImageFrameDecoder(imageType).push(binaryFrame(compressed), () => {}));
});

test('decoder accepts exact 64 KiB metadata and 32 MiB payload limits', () => {
  const { ImageFrameDecoder } = harness();
  const { data, ...metadata } = frame;
  const text = JSON.stringify(metadata).padEnd(65536, ' ');
  const raw = Buffer.alloc(32 * 1024 * 1024, 0xff), results = [];
  const decoder = new ImageFrameDecoder();
  decoder.push(metadataFrame(text, raw.length), d => results.push(d));
  assert.deepEqual(results, []);
  decoder.push(raw, d => results.push(d));
  assert.deepEqual(results, [{ ...metadata, data: raw }]);
});

test('one-byte chunks preserve Unicode metadata and allocate each frame buffer only once without concat', () => {
  const { ImageFrameDecoder } = harness();
  const value = { ...frame, header: { ...frame.header, frame_id: 'camera 🚀 雪' } };
  const input = binaryFrame(value), decoder = new ImageFrameDecoder(), results = [];
  const allocate = Buffer.allocUnsafe, concat = Buffer.concat, allocations = [];
  Buffer.allocUnsafe = size => { allocations.push(size); return allocate(size); };
  Buffer.concat = () => assert.fail('Decoder must not concatenate growing buffers');
  try {
    for (let i = 0; i < input.length; i++) { decoder.push(input.subarray(i, i + 1), d => results.push(d)); }
  } finally { Buffer.allocUnsafe = allocate; Buffer.concat = concat; }
  assert.deepEqual(results, [value]);
  assert.deepEqual(allocations, [input.readUInt32LE(4), value.data.length]);
});

for (const type of [imageType, compressedType]) {
  test(`${type}: starts explicit ROS Python and delivers raw bytes without a shell or YAML`, async () => {
    const h = harness(), manager = new h.ImageSubscriptionManager(), received = [];
    manager.start('/camera/image & literal', type, m => received.push(m), 7);
    assert.equal(manager.has('/camera/image & literal'), true);
    await tick();
    assert.equal(h.calls[0].executable, 'C:\\pixi\\lyrical\\python.exe');
    assert.deepEqual(h.calls[0].args, ['-u', path.join(h.extension.extPath, 'assets/scripts/image_subscriber.py'),
      '--topic', '/camera/image & literal', '--type', type, '--rate', '7']);
    assert.equal(h.calls[0].options.env, h.env);
    assert.equal(h.calls[0].options.shell, undefined);
    const data = type === imageType ? frame : { header: frame.header, format: 'jpeg', data: Buffer.from([0, 1, 2, 255]) };
    assert.equal(h.children[0].stdout.readableEncoding, null);
    assert.equal(h.children[0].stderr.readableEncoding, 'utf8');
    h.children[0].stdout.write(binaryFrame(data));
    assert.deepEqual(received[0].data, data);
    manager.setRate('/camera/image & literal', 12);
    assert.deepEqual(JSON.parse(h.children[0].controls[0]), { rateHz: 12 });
    assert.equal(h.calls.length, 1, 'Rate change does not restart DDS discovery');
    manager.stop('/camera/image & literal');
    assert.ok(h.children[0].killed);
    h.children[0].stdout.write(binaryFrame(data));
    assert.equal(received.length, 1, 'No late delivery after stop');
  });
}

test('stop and dispose invalidate pending ROS activation and interpreter resolution', async () => {
  for (const phase of ['environment', 'python']) {
    const h = harness(), manager = new h.ImageSubscriptionManager();
    let resolve;
    const pending = new Promise(r => { resolve = r; });
    if (phase === 'environment') { h.extension.resolvedEnv = () => pending; }
    else { h.python.resolveRosPython = () => pending; }
    manager.start('/image', imageType, () => assert.fail('Cancelled'));
    await tick(); manager.dispose(); resolve(phase === 'environment' ? h.env : 'python'); await tick();
    assert.equal(h.calls.length, 0);
    assert.equal(manager.has('/image'), false);
  }
});

test('resume and replacement ignore old process events and use latest pending rate', async () => {
  const h = harness(), manager = new h.ImageSubscriptionManager(), received = [];
  manager.start('/image', imageType, m => received.push(m)); await tick();
  const old = h.children[0];
  old.stdout.write(binaryFrame(frame).subarray(0, 20));
  manager.start('/image', imageType, m => received.push(m));
  manager.setRate('/image', 30); await tick();
  assert.ok(old.killed);
  const logs = h.logs.length;
  old.stdout.write(binaryFrame(frame)); old.stderr.write('stale'); old.stdin.emit('error', new Error('stale'));
  old.emit('close', 1); old.emit('error', new Error('stale'));
  assert.equal(h.logs.length, logs); assert.deepEqual(received, []);
  assert.equal(manager.has('/image'), true);
  assert.equal(h.calls[1].args.at(-1), '30');
  h.children[1].stdout.write(binaryFrame(frame));
  assert.deepEqual(received.map(m => m.data), [frame]);
  manager.dispose(); assert.ok(h.children[1].killed);
});

test('activation failures and stream/process errors stop cleanly with diagnostics', async () => {
  for (const failure of ['activation', 'spawn', 'error', 'close', 'stdin', 'json']) {
    const h = harness(), manager = new h.ImageSubscriptionManager();
    if (failure === 'activation') { h.extension.resolvedEnv = async () => { throw new Error('bad ROS'); }; }
    if (failure === 'spawn') { h.child_process.spawn = () => { throw new Error('spawn failed'); }; }
    manager.start('/image', imageType, () => {}); await tick();
    const child = h.children[0];
    if (failure === 'error') { child.emit('error', new Error('missing Python')); }
    if (failure === 'close') { child.emit('close', 1); }
    if (failure === 'stdin') { child.stdin.emit('error', new Error('EPIPE')); }
    if (failure === 'json') { child.stdout.write('private-camera-payload\n'); }
    assert.equal(manager.has('/image'), false, failure);
    assert.ok(h.logs.length > 0);
    assert.ok(!h.logs.join('\n').includes('private-camera-payload'));
  }
});

test('TopicEchoManager routes image types to direct subscriptions and keeps ordinary topics on YAML', () => {
  const h = harness(), imageCalls = [];
  const images = { start: (...args) => imageCalls.push(args), stop() {}, dispose() {}, has: () => true, setRate() {} };
  const { TopicEchoManager } = load('../out/src/ros/ros2/topic-monitor', { '../../extension': h.extension,
    child_process: h.child_process, './image-subscription': {
      ImageSubscriptionManager: function() { return images; },
      PointCloudSubscriptionManager: function() { return { start() {}, stop() {}, dispose() {}, has: () => false, setRate() {} }; },
    } });
  const manager = new TopicEchoManager(), handler = () => {};
  manager.startEcho('/image', handler, imageType, 8);
  assert.deepEqual(imageCalls[0], ['/image', imageType, handler, 8]);
  assert.equal(h.calls.length, 0);
  manager.startEcho('/chatter', handler, 'std_msgs/msg/String');
  assert.equal(h.calls[0].executable, 'ros2');
  assert.deepEqual(h.calls[0].args, ['topic', 'echo', '/chatter', '--full-length']);
  manager.dispose();
});

function webviewHarness() {
  const starts = [], stops = [], rates = [], sent = [];
  let receive;
  const panel = { reveal() {}, dispose() {}, onDidDispose() {}, webview: { cspSource: 'test:', html: '',
    postMessage: m => { sent.push(m); return Promise.resolve(true); }, onDidReceiveMessage: fn => { receive = fn; } } };
  const module = load('../out/src/ros/ros2/topic-webview', {
    vscode: { ViewColumn: { Beside: 2 }, window: { createWebviewPanel: () => panel } },
    './topic-monitor': { TopicEchoManager: class {
      startEcho(...args) { starts.push(args); } stopEcho(t) { stops.push(t); }
      setImageRefreshRate(...args) { rates.push(args); }
      setPointCloudRefreshRate(...args) { rates.push(args); } dispose() {}
    } },
  });
  return { ...module, starts, stops, rates, sent, receive: m => receive(m) };
}

test('webview routes raw images, changes source rate, pauses/resumes and preserves history', () => {
  const h = webviewHarness(), manager = new h.TopicWebviewManager({ subscriptions: [] });
  manager.openTopicMonitor('/image', imageType);
  assert.equal(h.starts[0][2], imageType); assert.equal(h.starts[0][3], 5);
  h.starts[0][1]({ timestamp: 1, data: frame });
  assert.deepEqual(h.sent[0].message.data, webFrame);
  h.receive({ command: 'setRefreshRate', rateHz: 12 });
  assert.deepEqual(h.rates[0], ['/image', 12]);
  h.receive({ command: 'pause' });
  const count = h.sent.length;
  h.starts[0][1]({ timestamp: 2, data: frame });
  assert.equal(h.sent.length, count);
  h.receive({ command: 'resume' });
  assert.equal(h.starts[1][3], 12);
  h.receive({ command: 'getHistory' });
  assert.deepEqual(h.sent.at(-1).messages, [{ timestamp: 1, data: webFrame }]);
  manager.dispose(); assert.ok(h.stops.length >= 2);
});

test('received base64 RGB bytes reach the existing canvas renderer', () => {
  const h = webviewHarness();
  const html = h.createTopicMonitorHtml('test:', '/image', imageType);
  const functions = html.slice(html.indexOf('    function imageBytes('), html.indexOf('    let latestImage;'));
  const context = { Uint8Array, atob: value => Buffer.from(value, 'base64').toString('binary') };
  vm.createContext(context); vm.runInContext(functions, context);
  let pixels;
  const canvas = { getContext: () => ({ createImageData: (w, h) => ({ data: new Uint8ClampedArray(w * h * 4) }),
    putImageData: image => { pixels = image.data; } }) };
  assert.equal(context.drawRawImage(canvas, { data: webFrame }), undefined);
  assert.deepEqual(Array.from(pixels), [10,20,30,255,40,50,60,255]);
});

test('webview pause/resume updates the icon for clicks and watcher state changes', () => {
  const html = webviewHarness().createTopicMonitorHtml('test:', '/points', 'sensor_msgs/msg/PointCloud2');
  const attributes = {}, icon = {}, sent = [];
  let click;
  const context = {
    isPaused: false,
    hero: { classList: { toggle() {} } },
    pauseLabel: {}, statusText: {},
    pauseIcon: { setAttribute: (key, value) => { icon[key] = value; } },
    pauseButton: {
      setAttribute: (key, value) => { attributes[key] = value; },
      addEventListener: (name, callback) => { assert.equal(name, 'click'); click = callback; },
    },
    vscode: { postMessage: message => sent.push(message.command) },
  };
  vm.createContext(context);
  vm.runInContext(html.slice(html.indexOf('    function setPaused('),
    html.indexOf('    clearButton.addEventListener(')), context);
  context.setPaused(false);
  const pausePath = icon.d;
  assert.ok(pausePath);
  click();
  const playPath = icon.d;
  assert.notEqual(playPath, pausePath);
  assert.equal(context.pauseLabel.textContent, 'Resume');
  assert.equal(attributes['aria-pressed'], 'true');
  click();
  assert.equal(icon.d, pausePath);
  assert.equal(context.pauseLabel.textContent, 'Pause');
  assert.equal(attributes['aria-pressed'], 'false');
  assert.deepEqual(sent, ['pause', 'resume']);
  context.setPaused(true);
  assert.equal(icon.d, playPath);
  context.setPaused(false);
  assert.equal(icon.d, pausePath);
  assert.match(html, /id="pauseIcon"/);
});

test('UTF-8 chunk boundaries and invalid refresh rates are handled safely', async () => {
  const h = harness(), manager = new h.ImageSubscriptionManager(), received = [];
  manager.start('/image', imageType, m => received.push(m), NaN);
  await tick();
  assert.equal(h.calls[0].args.at(-1), '5');
  const data = { ...frame, header: { ...frame.header, frame_id: 'camera 🚀' } };
  const encoded = binaryFrame(data);
  for (const byte of encoded) { h.children[0].stdout.write(Buffer.from([byte])); }
  assert.deepEqual(received[0].data, data);
  for (const rate of [-10, Infinity, 100]) { manager.setRate('/image', rate); }
  assert.deepEqual(h.children[0].controls.map(s => JSON.parse(s).rateHz), [1, 5, 30]);
  manager.dispose();
});

test('global watcher pause/resume stops image subscriptions and preserves preview rate', () => {
  const h = webviewHarness(), manager = new h.TopicWebviewManager({ subscriptions: [] });
  manager.setMonitoringEnabled(true);
  manager.openTopicMonitor('/image', compressedType);
  manager.setMonitoringEnabled(false);
  assert.equal(h.stops.at(-1), '/image');
  h.receive({ command: 'setRefreshRate', rateHz: 9 });
  assert.equal(h.starts.length, 1, 'Changing rate while paused must not resume');
  manager.setMonitoringEnabled(true);
  assert.equal(h.starts.length, 2);
  assert.equal(h.starts[1][2], compressedType);
  assert.equal(h.starts[1][3], 9);
  manager.closeAll();
  assert.equal(h.stops.at(-1), '/image');
});