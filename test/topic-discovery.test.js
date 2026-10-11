const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const Module = require('node:module');
const path = require('node:path');
const { test } = require('node:test');
const { promisify } = require('node:util');

function harness() {
  const h = { calls: [], logs: [], resolutions: [],
    run: async () => ({ stdout: '/camera [sensor_msgs/msg/Image]\r\n/ros2cli_probe [std_msgs/msg/String]\r\n', stderr: '' }) };
  h.env = { ROS_VERSION: '2', ROS_DISTRO: 'lyrical', CONDA_PREFIX: "C:\\ROS user's & tools",
    Path: 'ROS tools', AMENT_PREFIX_PATH: 'workspace install', PYTHONPATH: 'workspace modules' };
  h.python = path.win32.join(h.env.CONDA_PREFIX, 'python.exe');
  h.extension = { env: h.env, outputChannel: { appendLine: text => h.logs.push(text) } };
  h.resolve = async env => { h.resolutions.push(env); return h.python; };
  const execFile = () => assert.fail('Use promisified execFile');
  execFile[promisify.custom] = (executable, args, options) => {
    h.calls.push({ executable, args, options });
    return h.run();
  };
  const mocks = {
    '../../extension': h.extension,
    '../python': { resolveRosPython: env => h.resolve(env) },
    './image-subscription': {},
    child_process: { execFile, spawn: () => assert.fail('Discovery must be bounded') },
    os: { EOL: '\r\n' },
  };
  const filename = require.resolve('../out/src/ros/ros2/topic-monitor');
  const loaded = new Module(filename, module);
  loaded.filename = filename;
  loaded.paths = Module._nodeModulePaths(path.dirname(filename));
  loaded.require = name => Object.hasOwn(mocks, name) ? mocks[name] : Module.prototype.require.call(loaded, name);
  loaded._compile(readFileSync(filename, 'utf8'), filename);
  return { ...h, monitor: loaded.exports, state: h };
}

test('topic discovery uses selected ROS Python, a hidden direct child, and a bounded query', async () => {
  const h = harness();
  assert.deepEqual(await h.monitor.listTopics(), [{ name: '/camera', type: 'sensor_msgs/msg/Image' }]);
  assert.deepEqual(h.resolutions, [h.env]);
  const { executable, args, options } = h.calls[0];
  assert.equal(executable, h.python);
  assert.deepEqual(args.slice(0, 2), ['-u', '-c']);
  assert.match(args[2], /from ros2cli.cli import main/);
  assert.deepEqual(args.slice(3), ['topic', 'list', '-t']);
  assert.equal(options.env, h.env);
  assert.equal(options.windowsHide, true);
  assert.equal(options.shell, undefined);
  assert.equal(options.timeout, 10000);
  assert.equal(options.maxBuffer, 1024 * 1024);
});

for (const eol of ['\n', '\r\n']) {
  test(`topic info accepts ${JSON.stringify(eol)} output on Windows`, async () => {
    const h = harness();
    h.state.run = async () => ({ stdout: ['Type: std_msgs/msg/String', 'Publisher count: 2',
      'Subscription count: 1', 'Reliability: RELIABLE', 'Durability: VOLATILE'].join(eol), stderr: '' });
    const info = await h.monitor.getTopicInfo('/chatter');
    assert.equal(info.type, 'std_msgs/msg/String');
    assert.equal(info.publisherCount, 2);
    assert.equal(info.subscriberCount, 1);
    assert.equal(info.qos.reliability, 'RELIABLE');
    assert.deepEqual(h.calls[0].args.slice(3), ['topic', 'info', '/chatter', '-v']);
    assert.equal(h.calls[0].options.timeout, 10000);
    assert.equal(h.calls[0].options.windowsHide, true);
  });
}

test('missing ROS does not fall back to the extension-host environment', async () => {
  const h = harness();
  for (const env of [undefined, {}, { ROS_VERSION: '1' }]) {
    h.extension.env = env;
    await assert.rejects(h.monitor.listTopics(), /ROS 2 environment/);
    assert.equal(await h.monitor.getTopicInfo('/chatter'), null);
  }
  assert.deepEqual(h.calls, []);
  assert.deepEqual(h.resolutions, []);
});

test('interpreter resolution failures do not execute a host ROS launcher', async () => {
  const h = harness();
  h.state.resolve = async () => { throw new Error('selected ROS Python is unavailable'); };
  await assert.rejects(h.monitor.listTopics(), /selected ROS Python/);
  assert.deepEqual(h.calls, []);
});

test('failed discovery is logged and a later query can recover', async () => {
  const h = harness();
  h.state.run = async () => { throw new Error('query timed out'); };
  await assert.rejects(h.monitor.listTopics(), /query timed out/);
  assert.match(h.logs.join('\n'), /Error listing topics.*query timed out/);
  h.state.run = async () => ({ stdout: '', stderr: '' });
  assert.deepEqual(await h.monitor.listTopics(), []);
  assert.equal(h.calls.length, 2);
});
