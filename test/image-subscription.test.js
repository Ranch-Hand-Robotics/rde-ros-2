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
  header: { frame_id: 'camera', stamp: { sec: 1, nanosec: 2 } }, data: 'ChQeKDI8' };

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
  const large = { ...frame, data: Buffer.alloc(1280 * 720 * 3, 42).toString('base64') };
  const text = JSON.stringify(large) + '\r\n' + JSON.stringify(frame) + '\n';
  const decoder = new ImageFrameDecoder(), results = [];
  for (let i = 0; i < text.length; i += 65521) { decoder.push(text.slice(i, i + 65521), d => results.push(d)); }
  assert.deepEqual(results, [large, frame]);
});

test('decoder waits for newline and rejects malformed or oversized frames', () => {
  const { ImageFrameDecoder } = harness();
  const decoder = new ImageFrameDecoder(), frames = [];
  decoder.push(JSON.stringify(frame), d => frames.push(d));
  assert.equal(frames.length, 0);
  decoder.push('\n', d => frames.push(d));
  assert.equal(frames.length, 1);
  for (const input of ['bad json\n', 'null\n', '{"data":[]}\n']) {
    assert.throws(() => new ImageFrameDecoder().push(input, () => {}));
  }
  assert.throws(() => new ImageFrameDecoder().push('a'.repeat(64 * 1024 * 1024 + 1), () => {}), /transport limit/);
});

for (const type of [imageType, compressedType]) {
  test(`${type}: starts explicit ROS Python and delivers base64 without a shell or YAML`, async () => {
    const h = harness(), manager = new h.ImageSubscriptionManager(), received = [];
    manager.start('/camera/image & literal', type, m => received.push(m), 7);
    assert.equal(manager.has('/camera/image & literal'), true);
    await tick();
    assert.equal(h.calls[0].executable, 'C:\\pixi\\lyrical\\python.exe');
    assert.deepEqual(h.calls[0].args, ['-u', path.join(h.extension.extPath, 'assets/scripts/image_subscriber.py'),
      '--topic', '/camera/image & literal', '--type', type, '--rate', '7']);
    assert.equal(h.calls[0].options.env, h.env);
    assert.equal(h.calls[0].options.shell, undefined);
    const data = type === imageType ? frame : { format: 'jpeg', data: 'AAEC/w==' };
    h.children[0].stdout.write(JSON.stringify(data) + '\n');
    assert.deepEqual(received[0].data, data);
    manager.setRate('/camera/image & literal', 12);
    assert.deepEqual(JSON.parse(h.children[0].controls[0]), { rateHz: 12 });
    assert.equal(h.calls.length, 1, 'Rate change does not restart DDS discovery');
    manager.stop('/camera/image & literal');
    assert.ok(h.children[0].killed);
    h.children[0].stdout.write(JSON.stringify(data) + '\n');
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
  const h = harness(), manager = new h.ImageSubscriptionManager();
  manager.start('/image', imageType, () => {}); await tick();
  const old = h.children[0];
  manager.start('/image', imageType, () => {});
  manager.setRate('/image', 30); await tick();
  assert.ok(old.killed);
  old.emit('close', 1); old.emit('error', new Error('stale'));
  assert.equal(manager.has('/image'), true);
  assert.equal(h.calls[1].args.at(-1), '30');
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

test('TopicEchoManager routes only image types to direct subscriptions', () => {
  const h = harness(), imageCalls = [];
  const images = { start: (...args) => imageCalls.push(args), stop() {}, dispose() {}, has: () => true, setRate() {} };
  const { TopicEchoManager } = load('../out/src/ros/ros2/topic-monitor', { '../../extension': h.extension,
    child_process: h.child_process, './image-subscription': { ImageSubscriptionManager: function() { return images; } } });
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
      setImageRefreshRate(...args) { rates.push(args); } dispose() {}
    } },
  });
  return { ...module, starts, stops, rates, sent, receive: m => receive(m) };
}

test('webview routes raw images, changes source rate, pauses/resumes and preserves history', () => {
  const h = webviewHarness(), manager = new h.TopicWebviewManager({ subscriptions: [] });
  manager.openTopicMonitor('/image', imageType);
  assert.equal(h.starts[0][2], imageType); assert.equal(h.starts[0][3], 5);
  h.starts[0][1]({ timestamp: 1, data: frame });
  assert.deepEqual(h.sent[0].message.data, frame);
  h.receive({ command: 'setRefreshRate', rateHz: 12 });
  assert.deepEqual(h.rates[0], ['/image', 12]);
  h.receive({ command: 'pause' });
  const count = h.sent.length;
  h.starts[0][1]({ timestamp: 2, data: frame });
  assert.equal(h.sent.length, count);
  h.receive({ command: 'resume' });
  assert.equal(h.starts[1][3], 12);
  h.receive({ command: 'getHistory' });
  assert.deepEqual(h.sent.at(-1).messages, [{ timestamp: 1, data: frame }]);
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
  assert.equal(context.drawRawImage(canvas, { data: frame }), undefined);
  assert.deepEqual(Array.from(pixels), [10,20,30,255,40,50,60,255]);
});

test('UTF-8 chunk boundaries and invalid refresh rates are handled safely', async () => {
  const h = harness(), manager = new h.ImageSubscriptionManager(), received = [];
  manager.start('/image', imageType, m => received.push(m), NaN);
  await tick();
  assert.equal(h.calls[0].args.at(-1), '5');
  const data = { ...frame, header: { ...frame.header, frame_id: 'camera 🚀' } };
  const encoded = Buffer.from(JSON.stringify(data) + '\n');
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