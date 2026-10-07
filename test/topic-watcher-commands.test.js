const assert = require("node:assert/strict");
const { readFileSync } = require("node:fs");
const Module = require("node:module");
const path = require("node:path");
const { test } = require("node:test");

// Private compiled modules: no global require hooks, process mutation, ROS,
// VS Code host, real timers, or subprocesses. Run after npm run test-compile.
function loadWithMocks(filename, mocks, globals, stubLocalImports = false) {
  const resolved = require.resolve(filename);
  const loaded = new Module(resolved, module);
  loaded.filename = resolved;
  loaded.paths = Module._nodeModulePaths(path.dirname(resolved));
  loaded.globals = globals;
  loaded.require = request => {
    if (Object.hasOwn(mocks, request)) { return mocks[request]; }
    if (stubLocalImports && request.startsWith(".")) { return {}; }
    return Module.prototype.require.call(loaded, request);
  };
  loaded._compile("const { process, setInterval, clearInterval } = module.globals;\n" +
    readFileSync(resolved, "utf8"), resolved);
  return loaded.exports;
}

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

class EventEmitter {
  listeners = new Set();
  event = listener => {
    this.listeners.add(listener);
    return { dispose: () => this.listeners.delete(listener) };
  };
  fire(value) { for (const listener of [...this.listeners]) { listener(value); } }
  dispose() { this.listeners.clear(); }
}

const flush = () => new Promise(resolve => setImmediate(resolve));
const readyEnv = { ROS_VERSION: "2", ROS_DISTRO: "fixture", PATH: "fixture ROS" };
const topic = { name: "/camera", type: "sensor_msgs/msg/Image" };
const watcherContext = "ros2.topicWatcherEnabled";

async function harness(t) {
  const h = {
    commands: new Map(), contexts: new Map(), errors: [], logs: [],
    sources: [], lists: [], starts: [], monitoring: false, activations: 0,
    source: async () => readyEnv, list: async () => [],
  };
  const disposable = () => ({ dispose() {} });
  const visibility = new EventEmitter();
  const timers = new Set();
  const globals = {
    process: { platform: process.platform, env: {} },
    setInterval: callback => { timers.add(callback); return callback; },
    clearInterval: callback => timers.delete(callback),
  };
  const setup = path.join(__dirname, "fixture-setup");
  const config = { get: (key, fallback) => ({ rosSetupScript: setup, showROS2WelcomeOnStartup: false })[key] ?? fallback };
  const context = { subscriptions: [], extensionPath: path.dirname(__dirname) };
  const view = { visible: false, dispose() {}, onDidChangeCheckboxState: disposable,
    onDidChangeVisibility: visibility.event };
  const vscode = {
    EventEmitter,
    TreeItem: class { constructor(label) { this.label = label; } },
    ThemeIcon: class {},
    TreeItemCollapsibleState: { None: 0 },
    TreeItemCheckboxState: { Unchecked: 0, Checked: 1 },
    commands: {
      registerCommand: (name, callback) => { h.commands.set(name, callback); return disposable(); },
      executeCommand: async (name, key, value) => {
        assert.equal(name, "setContext");
        h.contexts.set(key, value);
      },
    },
    workspace: {
      onDidChangeWorkspaceFolders: disposable, onDidCreateFiles: disposable,
      onDidDeleteFiles: disposable, onDidChangeConfiguration: disposable,
    },
    window: {
      createTreeView: name => name.endsWith(".topicTree") ? view : disposable(),
      onDidChangeWindowState: disposable,
      showErrorMessage: message => { h.errors.push(message); },
      showInformationMessage: disposable, setStatusBarMessage: disposable,
    },
    languages: { registerDocumentFormattingEditProvider: disposable },
    tasks: { registerTaskProvider: disposable },
  };
  const items = loadWithMocks("../out/src/ros/topic-tree/topic-tree-item", { vscode }, globals);
  const provider = loadWithMocks("../out/src/ros/topic-tree/topic-tree-provider", {
    vscode, "./topic-tree-item": items,
    "../ros2/topic-monitor": {
      listTopics: () => { h.lists.push(h.extension.env); return h.list(); },
      getTopicInfo: async () => null,
    },
  }, globals);
  // Use the real one-shot event helper, including listener disposal.
  const debugUtils = loadWithMocks("../out/src/debugger/utils", { vscode }, globals, true);
  h.extension = loadWithMocks("../out/src/extension", {
    vscode,
    fs: { promises: { access: async filename => assert.equal(filename, setup) } },
    "./debugger/utils": debugUtils,
    "./build-tool/colcon": { COLCON_TASK_TYPE: "colcon", ColconProvider: class {} },
    "./telemetry-helper": { getReporter: () => ({ sendTelemetryActivate() {} }) },
    "./cpp-formatter": { CppFormatter: class {} },
    "./ros/ros-msg-providers": { registerRosMessageProviders: () => [] },
    "./ros/launch-link-provider": { registerLaunchLinkProvider: disposable },
    "./test-provider/ros-test-provider": { RosTestProvider: class {} },
    "./ros/launch-tree/launch-tree-provider": { LaunchTreeDataProvider: class {} },
    "./ros/ros-distributions-provider": { RosDistributionsProvider: class {} },
    "./ros/topic-tree/topic-tree-provider": provider,
    "./ros/ros2/topic-webview": { TopicWebviewManager: class {
      topics = new Set();
      openTopicMonitor(name) { this.topics.add(name); }
      closeTopicMonitor(name) { this.topics.delete(name); }
      setMonitoringEnabled(enabled) {
        if (enabled === h.monitoring) { return; }
        h.monitoring = enabled;
        if (enabled) {
          for (const name of this.topics) { h.starts.push({ name, env: h.extension.env }); }
        }
      }
    } },
    "./mcp": { registerMcpCommands() {} },
    "./vscode-utils": {
      createOutputChannel: () => ({ appendLine: text => h.logs.push(text), dispose() {} }),
      workspaceContainsPackageXml: async () => false,
      isLldbExtensionInstalled: () => false, isCppToolsExtensionInstalled: () => false, isCursorEditor: () => false,
      getExtensionConfiguration: () => config, getRosSetupScript: () => setup,
    },
    "./ros/utils": {
      sourceSetupFile: () => { h.sources.push(setup); return h.source(); },
      getSetupScriptExtension: () => process.platform === "win32" ? ".bat" : ".bash",
    },
    "./build-tool/build-tool": { determineBuildTool: async () => { h.activations++; return false; } },
    "./ros/ros": { selectROSApi() {}, rosApi: { setContext() {}, activateCoreMonitor: disposable } },
    "./build-tool/ros-shell": { registerRosShellTaskProvider: () => [] },
    "./debugger/manager": { registerRosDebugManager() {} },
    "./ros/installer/install-ros": { promptInstallRosIfNeeded: async () => {} },
  }, globals, true);
  t.after(() => {
    for (const subscription of context.subscriptions) { subscription.dispose?.(); }
    assert.equal(timers.size, 0);
  });
  await h.extension.activate(context);
  await h.extension.activateEnvironment(context);
  await flush();
  h.setVisible = visible => { view.visible = visible; visibility.fire({ visible }); };
  h.setVisible(true);
  await flush();
  assert.deepEqual(h.errors, [], "Activation and visibility setup must succeed");
  h.sources.length = 0;
  h.lists.length = 0;
  h.provider = h.extension.topicTreeProvider;
  await h.provider.subscribe(topic);
  h.run = name => {
    const callback = h.commands.get(`ROS2.topicTree.${name}`);
    assert.equal(typeof callback, "function", "Exercise activate()'s actual registration");
    return callback();
  };
  h.assertState = enabled => {
    assert.equal(h.provider.isWatcherEnabled(), enabled, "Provider state");
    assert.equal(h.contexts.get(watcherContext), enabled, "Play/Pause context must match provider");
    assert.equal(h.monitoring, enabled && view.visible, "Monitor subscription gate");
  };
  return h;
}

test("Play starts selected subscriptions with ready env before a slow graph query completes", async t => {
  const h = await harness(t);
  const source = deferred();
  const list = deferred();
  h.source = () => source.promise;
  h.list = () => list.promise;
  t.after(() => { source.resolve(readyEnv); list.resolve([]); });
  const play = h.run("startWatcher");
  await flush();
  h.assertState(false);
  assert.equal(h.lists.length, 0, "No query before fresh sourcing completes");
  source.resolve(readyEnv);
  await flush();
  h.assertState(true);
  assert.deepEqual(h.starts, [{ name: topic.name, env: readyEnv }]);
  assert.deepEqual(h.lists, [readyEnv], "setWatcherEnabled/refreshTopics coalesce into ONE query");
  assert.equal(typeof play?.then, "function", "Play must return its completion promise");
  let completed = false;
  play.then(() => { completed = true; });
  await flush();
  assert.equal(completed, false, "Completion still includes the graph refresh");
  list.resolve([topic]);
  await play;
  assert.equal((await h.provider.getChildren())[0].label, topic.name);
  h.assertState(true);
});

test("Pause wins over Play that is still sourcing", async t => {
  const h = await harness(t);
  const source = deferred();
  h.source = () => source.promise;
  t.after(() => source.resolve(readyEnv));
  const play = h.run("startWatcher");
  await flush();
  const pause = h.run("pauseWatcher");
  await pause;
  source.resolve(readyEnv);
  await play;
  await flush();
  h.assertState(false);
  assert.deepEqual(h.starts, []);
  assert.deepEqual(h.lists, [], "Cancelled Play must not start a query");
  assert.equal(typeof pause?.then, "function", "Pause must return its completion promise");
});

test("Pause during a graph query is not overwritten by late Play completion", async t => {
  const h = await harness(t);
  const list = deferred();
  h.list = () => list.promise;
  t.after(() => list.resolve([]));
  const play = h.run("startWatcher");
  await flush();
  await h.run("pauseWatcher");
  h.assertState(false);
  list.resolve([topic]);
  await play;
  await flush();
  h.assertState(false);
  assert.equal(h.starts.length, 1, "Subscriptions start promptly, and are not restarted after Pause");
});

test("Refresh is one-shot while paused; Play joins its pending query without delaying subscriptions", async t => {
  const h = await harness(t);
  const list = deferred();
  h.list = () => list.promise;
  t.after(() => list.resolve([]));
  const refresh = h.run("refresh");
  await flush();
  h.assertState(false);
  assert.deepEqual(h.starts, []);
  assert.equal(typeof refresh?.then, "function");
  let completed = false;
  refresh.then(() => { completed = true; });
  await flush();
  assert.equal(completed, false);
  const play = h.run("startWatcher");
  await flush();
  h.assertState(true);
  assert.equal(h.starts.length, 1);
  assert.equal(h.lists.length, 1, "An existing request is coalesced, not duplicated");
  assert.equal(h.sources.length, 2, "Refresh and Play use the same fresh sourcing path");
  list.resolve([topic]);
  await Promise.all([refresh, play]);
});

for (const command of ["refresh", "startWatcher"]) {
  test(`${command}: silent sourcing releases an existing resolvedEnv wait without reactivating providers`, async t => {
    const h = await harness(t);
    const activations = h.activations;
    let environmentChanges = 0;
    h.extension.onDidChangeEnv(() => { environmentChanges++; });
    h.extension.env = undefined;
    let observed;
    const waiting = h.extension.resolvedEnv().then(env => { observed = env; });
    const completion = h.run(command);
    await flush();
    assert.equal(h.extension.env, readyEnv, "Sourcing published the ready environment");
    assert.equal(observed, readyEnv, "An already-waiting subscriber must also be released");
    await Promise.all([waiting, completion]);
    assert.equal(environmentChanges, 0, "Do not force full environment reactivation for topic commands");
    assert.equal(h.activations, activations);
  });
}

test("Play/Pause/Play respects the latest request when sourcing finishes out of order", async t => {
  const h = await harness(t);
  const oldSource = deferred();
  const newSource = deferred();
  let calls = 0;
  h.source = () => ++calls === 1 ? oldSource.promise : newSource.promise;
  t.after(() => { oldSource.resolve(readyEnv); newSource.resolve(readyEnv); });
  const oldPlay = h.run("startWatcher");
  await flush();
  await h.run("pauseWatcher");
  const newPlay = h.run("startWatcher");
  await flush();
  newSource.resolve(readyEnv);
  await newPlay;
  h.assertState(true);
  const queries = h.lists.length;
  oldSource.resolve(readyEnv);
  await oldPlay;
  await flush();
  h.assertState(true);
  assert.equal(h.lists.length, queries, "Stale Play must not refresh or restart monitoring");
  assert.equal(h.starts.length, 1);
});

test("hiding the tree during Play's query stops monitoring and late completion cannot restart it", async t => {
  const h = await harness(t);
  const list = deferred();
  h.list = () => list.promise;
  t.after(() => list.resolve([]));
  const play = h.run("startWatcher");
  await flush();
  h.setVisible(false);
  assert.equal(h.monitoring, false);
  list.resolve([]);
  await play;
  h.assertState(true); // Watcher intent remains enabled; visibility gates subscriptions.
  assert.equal(h.starts.length, 1);
});

test("missing ROS environment does not enable the watcher or start subscribers", async t => {
  const h = await harness(t);
  h.source = async () => undefined;
  await h.run("startWatcher");
  await flush();
  h.assertState(false);
  assert.deepEqual(h.starts, []);
  assert.deepEqual(h.lists, []);
  assert.match(h.errors.join("\n"), /ROS 2 environment/);
});

test("Stop All returns completion and closes the selected monitors", async t => {
  const h = await harness(t);
  const completion = h.run("pauseAll");
  assert.equal(typeof completion?.then, "function");
  await completion;
  assert.deepEqual(h.provider.getSubscribedTopics(), []);
});