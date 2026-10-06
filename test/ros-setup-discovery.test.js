const assert = require("node:assert/strict");
const childProcess = require("node:child_process");
const fs = require("node:fs/promises");
const Module = require("node:module");
const os = require("node:os");
const path = require("node:path");
const { test } = require("node:test");
const { promisify } = require("node:util");
const { sourceWindowsEnvironment } = require("../out/src/ros/windows-env");

class EventEmitter {
  event = () => ({ dispose() {} });
  fire() {}
  dispose() {}
}

function loadWithMocks(filename, mocks, stubOtherLocalImports = false) {
  const resolved = require.resolve(filename);
  const original = Module._load;
  delete require.cache[resolved];
  Module._load = function(request, parent, isMain) {
    if (parent?.filename === resolved) {
      if (Object.hasOwn(mocks, request)) { return mocks[request]; }
      if (stubOtherLocalImports && request.startsWith(".")) { return {}; }
    }
    return original.call(this, request, parent, isMain);
  };
  try {
    return require(resolved);
  } finally {
    Module._load = original;
    delete require.cache[resolved];
  }
}

function discovery(files = [], platform = "win32", root = "D:\\cached ROS") {
  const platformPath = platform === "win32" ? path.win32 : path.posix;
  const normalize = filename => platform === "win32"
    ? platformPath.normalize(filename).toLowerCase() : platformPath.normalize(filename);
  const existing = new Set(files.map(normalize));
  const promises = {
    access: async filename => {
      if (!existing.has(normalize(filename))) { throw new Error("ENOENT"); }
    },
    readdir: async directory => {
      const prefix = normalize(directory) + platformPath.sep;
      const directories = new Set([...existing].filter(file => file.startsWith(prefix))
        .map(file => file.slice(prefix.length).split(platformPath.sep))
        .filter(parts => parts.length > 1).map(parts => parts[0]));
      return [...directories].map(name => ({ name, isDirectory: () => true }));
    },
  };
  const provider = loadWithMocks("../out/src/ros/ros-distributions-provider", {
    vscode: { TreeItem: class {}, EventEmitter },
    fs: { promises },
    os: { platform: () => platform, homedir: () => "/Users/test" },
    path: platformPath,
    "./installer/pixi-location": { getPixiInstallRoot: () => root },
  });
  return { ...provider, promises };
}

const cachedSetup = "D:\\cached ROS\\lyrical\\.pixi\\envs\\lyrical\\Library\\local_setup.bat";
const globalSetup = "C:\\opt\\ros\\lyrical\\x64\\setup.bat";

test("cached machine root wins over default Pixi root and global installs for the same distro", async () => {
  const loaded = discovery([globalSetup, "c:\\pixi_ws\\lyrical\\install\\setup.bat", cachedSetup]);
  const distros = await loaded.detectInstalledDistros();
  assert.equal(distros.length, 3);
  for (const [configured, inherited] of [["lyrical", "jazzy"], ["", "lyrical"], ["", ""], ["lyrical (pixi)", ""]]) {
    assert.equal(loaded.selectInstalledDistro(distros, configured, inherited).setupScript, cachedSetup);
  }
});

test("Windows discovery supports each known Pixi layout without inventing scripts", async () => {
  for (const relative of [
    "ros2-windows/local_setup.bat", "lyrical/install/setup.bat", "lyrical/local_setup.bat",
    "lyrical/.pixi/envs/lyrical/Library/local_setup.bat", "lyrical/.pixi/envs/lyrical/Library/setup.bat",
  ]) {
    const setup = path.win32.join("D:\\cached ROS", relative);
    const loaded = discovery([setup]);
    const distros = await loaded.detectInstalledDistros();
    assert.equal(distros.length, 1, relative);
    assert.equal(loaded.selectInstalledDistro(distros).setupScript, setup);
  }
});

test("Pixi layout priority favors install, then local, then environment local/setup", async () => {
  const scripts = ["install/setup.bat", "local_setup.bat", ".pixi/envs/lyrical/Library/local_setup.bat",
    ".pixi/envs/lyrical/Library/setup.bat"].map(relative => path.win32.join("D:\\cached ROS\\lyrical", relative));
  for (let index = 0; index < scripts.length; index++) {
    const loaded = discovery(scripts.slice(index));
    assert.deepEqual(await loaded.detectInstalledDistros(), [{ name: "lyrical (pixi)", setupScript: scripts[index] }]);
  }
});

test("Windows global fallback uses only existing x64/root local_setup.bat or setup.bat", async () => {
  const scripts = ["x64/local_setup.bat", "x64/setup.bat", "local_setup.bat", "setup.bat"]
    .map(relative => path.win32.join("C:\\opt\\ros\\lyrical", relative));
  for (let index = 0; index < scripts.length; index++) {
    const loaded = discovery(scripts.slice(index));
    assert.deepEqual(await loaded.detectInstalledDistros(), [{ name: "lyrical", setupScript: scripts[index] }]);
  }
});

test("multiple distinct distros remain ambiguous and missing requested distros do not fall back", async () => {
  const loaded = discovery([cachedSetup, "c:\\pixi_ws\\jazzy\\install\\setup.bat"]);
  const distros = await loaded.detectInstalledDistros();
  assert.equal(loaded.selectInstalledDistro(distros), undefined);
  assert.equal(loaded.selectInstalledDistro(distros, "missing", "lyrical"), undefined);
  assert.equal(loaded.selectInstalledDistro(distros, "", "missing"), undefined);
  assert.equal(loaded.selectInstalledDistro([], "lyrical"), undefined);
});

test("legacy ros2-windows does not pretend to be a requested named distro", async () => {
  const loaded = discovery(["c:\\pixi_ws\\ros2-windows\\local_setup.bat"], "win32", "c:\\pixi_ws");
  const distros = await loaded.detectInstalledDistros();
  assert.equal(distros.length, 1, "Overlapping configured/default roots are deduplicated");
  assert.equal(loaded.selectInstalledDistro(distros, "lyrical"), undefined);
});

test("Linux retains standard bash/sh discovery and macOS prefers its cached Pixi setup", async () => {
  for (const extension of ["bash", "sh"]) {
    const script = `/opt/ros/lyrical/setup.${extension}`;
    const loaded = discovery([script], "linux", "");
    assert.deepEqual(await loaded.detectInstalledDistros(), [{ name: "lyrical", setupScript: script }]);
  }
  const cached = "/Users/test/cached/lyrical/setup.bash";
  const loaded = discovery([cached, "/opt/ros/lyrical/setup.bash"], "darwin", "/Users/test/cached");
  assert.equal(loaded.selectInstalledDistro(await loaded.detectInstalledDistros(), "lyrical").setupScript, cached);
});

// Exercise the real exported activation entry point, but never activate ROS,
// run build tools, launch installers, or access the host's installations.
function activationHarness({ files = [], configured = "", explicit = "", failExplicit = false,
  workspace, source, probe, buildToolDetected = false, overlayText } = {}) {
  const loaded = discovery(files);
  loaded.promises.readFile = async filename => {
    await loaded.promises.access(filename).catch(() => { throw Object.assign(new Error("ENOENT"), { code: "ENOENT" }); });
    return overlayText ?? ':: generated from colcon_core/shell/template/prefix_chain.bat.em\n' +
      'call:_colcon_prefix_chain_bat_call_script "%%~dp0local_setup.bat"\n';
  };
  const buildEnvironment = loadWithMocks("../out/src/ros/build-environment", { fs: { promises: loaded.promises } });
  const sourced = [];
  const sourceCalls = [];
  const logs = [];
  const statuses = [];
  const h = { sourced, sourceCalls, logs, statuses, discoveries: 0, panels: [], errors: [],
    probes: [], registrations: [], activeProviders: new Set(), events: [] };
  const config = { get: (key, fallback) => ({ distro: configured, rosSetupScript: explicit })[key] ?? fallback };
  const disposable = () => ({ dispose() {} });
  const context = { subscriptions: [], extensionPath: "C:\\fixture extension" };
  const vscode = {
    EventEmitter,
    workspace: { rootPath: workspace, workspaceFolders: workspace ? [{ uri: { fsPath: workspace } }] : undefined,
      onDidChangeWorkspaceFolders: disposable, onDidCreateFiles: disposable, onDidDeleteFiles: disposable,
      onDidChangeConfiguration: disposable },
    commands: { executeCommand: async () => {}, registerCommand: disposable },
    languages: { registerDocumentFormattingEditProvider: disposable },
    TaskGroup: { Build: "build", Test: "test" },
    window: {
      createTreeView: () => ({ visible: false, dispose() {}, onDidChangeCheckboxState: disposable, onDidChangeVisibility: disposable }),
      setStatusBarMessage: async message => { statuses.push(message); },
      showErrorMessage: async message => { h.errors.push(message); },
    },
    tasks: {
      registerTaskProvider: (type, provider) => {
        h.events.push("register");
        const registration = { type, provider, disposed: false, dispose() {
          this.disposed = true;
          h.activeProviders.delete(this);
        } };
        h.registrations.push(registration);
        h.activeProviders.add(registration);
        return registration;
      },
      onDidEndTask: () => ({ dispose() {} }),
    },
  };
  const colcon = loadWithMocks("../out/src/build-tool/colcon", {
    vscode,
    "./ros-shell": { make: (name, definition) => ({ name, definition }) },
    "../vscode-utils": { workspaceContainsPackageXml: () => assert.fail("Tasks must not require package.xml") },
    "./colcon-utils": {
      getColconIgnoreConfig: () => ({}),
      getPackages: () => assert.fail("Activation task discovery must not run colcon list"),
      getNonIgnoredPackages: () => assert.fail("Activation task discovery must not run colcon list"),
    },
  });
  const execFile = () => assert.fail("CLI checks must use promisified execFile");
  execFile[promisify.custom] = async (command, args, options) => {
    h.probes.push({ command, args, options });
    return probe ? probe(command, args, options) : { stdout: "fixture help", stderr: "" };
  };
  const extension = loadWithMocks("../out/src/extension", {
    vscode,
    child_process: { execFile, spawn: () => assert.fail("Never spawn a build during ROS preparation") },
    fs: { promises: loaded.promises },
    "./build-tool/colcon": colcon,
    "./telemetry-helper": { getReporter: () => ({ sendTelemetryActivate() {} }) },
    "./cpp-formatter": { CppFormatter: class {} },
    "./ros/ros-msg-providers": { registerRosMessageProviders: () => [] },
    "./ros/launch-link-provider": { registerLaunchLinkProvider: disposable },
    "./test-provider/ros-test-provider": { RosTestProvider: class {} },
    "./ros/launch-tree/launch-tree-provider": { LaunchTreeDataProvider: class {} },
    "./ros/ros2/topic-webview": { TopicWebviewManager: class {} },
    "./ros/topic-tree/topic-tree-provider": { TopicTreeDataProvider: class { setViewVisible() {} } },
    "./mcp": { registerMcpCommands() {} },
    "./ros/build-environment": buildEnvironment,
    "./vscode-utils": {
      createOutputChannel: () => ({ appendLine: text => logs.push(text) }),
      workspaceContainsPackageXml: async () => false,
      isLldbExtensionInstalled: () => false, isCppToolsExtensionInstalled: () => false, isCursorEditor: () => false,
      getExtensionConfiguration: () => config,
      getRosSetupScript: () => explicit || "c:\\pixi_ws\\ros2-windows\\local_setup.bat",
      showOutputPanel: channel => { h.panels.push(channel); },
    },
    "./ros/ros-distributions-provider": {
      RosDistributionsProvider: class {},
      selectInstalledDistro: loaded.selectInstalledDistro,
      detectInstalledDistros: async () => { h.discoveries++; return loaded.detectInstalledDistros(); },
    },
    "./ros/utils": {
      getDistros: () => assert.fail("Must not use /opt/ros-only discovery"),
      getSetupScriptExtension: () => ".bat",
      sourceSetupFile: async (script, baseEnv, failOnMissingSetup) => {
        h.events.push("source");
        sourced.push(script);
        sourceCalls.push({ script, baseEnv, failOnMissingSetup });
        assert.ok(files.includes(script), `Must source a discovered or explicit script: ${script}`);
        if (failExplicit && script === explicit) { throw new Error("Explicit setup failed"); }
        return source ? source(script, baseEnv, failOnMissingSetup) : { ...baseEnv, ROS_DISTRO: "lyrical", ROS_VERSION: "2" };
      },
    },
    "./build-tool/build-tool": {
      determineBuildTool: async () => buildToolDetected,
      BuildTool: { registerTaskProvider: () => assert.fail("Windows must not register a second provider after sourcing") },
    },
    "./ros/build-env-utils": { createConfigFiles() {} },
    "./ros/ros": {
      selectROSApi() {},
      rosApi: { setContext() {}, activateCoreMonitor: () => ({ dispose() {} }) },
    },
    "./build-tool/ros-shell": { registerRosShellTaskProvider: () => [] },
    "./debugger/manager": { registerRosDebugManager() {} },
    "./ros/installer/install-ros": { promptInstallRosIfNeeded: async () => {} },
  }, true);
  extension.outputChannel = { appendLine: text => logs.push(text) };
  return { ...h, extension, context, get discoveries() { return h.discoveries; } };
}

async function activate(options = {}) {
  const h = activationHarness(options);
  const previousDistro = process.env.ROS_DISTRO;
  try {
    if (options.inherited) { process.env.ROS_DISTRO = options.inherited; } else { delete process.env.ROS_DISTRO; }
    await h.extension.activateEnvironment(h.context);
    return { ...h, env: h.extension.env };
  } finally {
    if (previousDistro === undefined) { delete process.env.ROS_DISTRO; } else { process.env.ROS_DISTRO = previousDistro; }
  }
}

test("activation uses cached setup even with configured distro and stale inherited ROS_DISTRO", async () => {
  const result = await activate({ files: [cachedSetup, globalSetup], configured: "lyrical", inherited: "jazzy" });
  assert.deepEqual(result.sourced, [cachedSetup, cachedSetup]);
  assert.equal(result.env.ROS_DISTRO, "lyrical");
  assert.ok(result.logs.some(line => line.includes("Ignoring ROS_DISTRO")));
});

test("activation auto-selects duplicate distro installations using the cached script", async () => {
  for (const inherited of ["", "lyrical"]) {
    const result = await activate({ files: [cachedSetup, globalSetup], inherited });
    assert.deepEqual(result.sourced, [cachedSetup, cachedSetup]);
  }
});

test("explicit setup script wins over cached discovery, configured distro, and inherited distro", async () => {
  const result = await activate({ files: [cachedSetup, globalSetup], explicit: globalSetup,
    configured: "other", inherited: "jazzy" });
  assert.deepEqual(result.sourced, [globalSetup, globalSetup]);
  assert.equal(result.discoveries, 0);
});

test("implicit legacy Pixi default cannot override a configured named distro", async () => {
  const result = await activate({ files: [cachedSetup, "c:\\pixi_ws\\ros2-windows\\local_setup.bat"], configured: "lyrical" });
  assert.deepEqual(result.sourced, [cachedSetup, cachedSetup]);
});

test("missing or failing explicit scripts retain the existing discovery fallback", async () => {
  const missing = await activate({ files: [cachedSetup], explicit: "C:\\missing.bat", configured: "lyrical" });
  assert.deepEqual(missing.sourced, [cachedSetup, cachedSetup]);
  const failed = await activate({ files: [cachedSetup, globalSetup], explicit: globalSetup, failExplicit: true, configured: "lyrical" });
  assert.deepEqual(failed.sourced, [globalSetup, cachedSetup, globalSetup, cachedSetup]);
  assert.ok(failed.logs.some(line => line.includes(`[ROS setup failed] ${globalSetup}`)));
  assert.match(failed.logs.join("\n"), /Explicit setup failed/);
  assert.deepEqual(failed.panels, [failed.extension.outputChannel, failed.extension.outputChannel]);
  assert.equal(failed.statuses.length, 2);
  assert.ok(failed.statuses.every(message => message.includes("See Output > ROS 2. Attempting discovery.")));
});

test("no discovery, missing configured distro, and ambiguity never synthesize a global setup path", async () => {
  for (const options of [
    {}, { inherited: "lyrical" }, { configured: "lyrical", inherited: "jazzy" },
    { files: [cachedSetup], configured: "missing", inherited: "lyrical" },
    { files: [cachedSetup, "c:\\pixi_ws\\jazzy\\install\\setup.bat"] },
  ]) {
    const result = await activate(options);
    assert.deepEqual(result.sourced, []);
    assert.equal(result.env, undefined);
    assert.match(result.statuses[0], /No ROS 2 setup script|Multiple ROS 2 distros/);
  }
});

test("failed startup still offers builds without package.xml, and retries preserve the same provider", async () => {
  let broken = true;
  const failure = Object.assign(new Error("startup setup failed"), { code: 17, stderr: "fixture missing dependency" });
  const h = activationHarness({ files: [cachedSetup], configured: "lyrical", workspace: "C:\\fixture workspace",
    buildToolDetected: true, source: async () => {
      if (broken) { throw failure; }
      return { ROS_VERSION: "2", ROS_DISTRO: "lyrical" };
    } });
  await h.extension.activateEnvironment(h.context);
  assert.equal(h.extension.env, undefined);
  assert.ok(h.logs.some(line => line.includes(`[ROS setup failed] ${cachedSetup}`)));
  assert.match(h.logs.join("\n"), /startup setup failed[\s\S]*Exit\/error code: 17[\s\S]*fixture missing dependency/);
  assert.deepEqual(h.panels, [h.extension.outputChannel]);
  assert.match(h.statuses[0], /See Output > ROS 2 for the cause/);
  assert.deepEqual(h.events.slice(0, 2), ["register", "source"]);
  assert.equal(h.registrations.length, 1);
  assert.equal(h.registrations[0].type, "colcon");
  assert.ok(h.context.subscriptions.includes(h.registrations[0]));
  assert.ok(!h.extension.subscriptions.includes(h.registrations[0]), "Environment refresh must not dispose colcon discovery");
  const tasks = await h.registrations[0].provider.provideTasks();
  assert.equal(tasks.length, 4);
  assert.equal(tasks.filter(task => task.group === "build").length, 2);
  assert.deepEqual(h.probes, [], "Offering tasks must not run ROS or colcon");
  broken = false;
  await h.extension.activateEnvironment(h.context);
  assert.equal(h.extension.env.ROS_VERSION, "2");
  assert.equal(h.registrations.length, 1, "One registration for the entire extension lifetime");
  assert.equal(h.registrations[0].disposed, false);
  assert.equal(h.activeProviders.size, 1);
  assert.ok(h.activeProviders.has(h.registrations[0]));
  assert.ok(h.context.subscriptions.includes(h.registrations[0]));
  assert.equal(h.extension.processingWorkspace, false);
});

test("full extension activation returns with build tasks while ROS sourcing is still pending", async () => {
  let release;
  const pending = new Promise(resolve => { release = resolve; });
  const h = activationHarness({ workspace: "C:\\unbuilt workspace", files: [cachedSetup], configured: "lyrical",
    source: async () => { await pending; return { ROS_VERSION: "2", ROS_DISTRO: "lyrical" }; } });
  try {
    const activation = h.extension.activate(h.context);
    assert.equal(h.registrations.length, 1, "Register before the first asynchronous startup operation");
    const result = await Promise.race([activation, new Promise(resolve => setImmediate(() => resolve("blocked")))]);
    assert.notEqual(result, "blocked", "ROS startup must not block task-triggered extension activation");
    assert.equal(h.extension.env, undefined);
    assert.equal(h.extension.processingWorkspace, true);
    const tasks = await h.registrations[0].provider.provideTasks();
    assert.equal(tasks.filter(task => task.group === "build").length, 2);
    assert.deepEqual(h.probes, []);
  } finally {
    release();
    await h.extension.activateEnvironment(h.context);
  }
});

for (const workspace of ["C:\\empty non-ROS workspace", undefined]) {
  test(`startup without ROS registers a provider but offers tasks only in an open workspace: ${workspace ?? "empty window"}`, async () => {
    const h = await activate({ workspace });
    assert.equal(h.env, undefined);
    assert.equal(h.registrations.length, 1);
    assert.equal(h.registrations[0].type, "colcon");
    const tasks = await h.registrations[0].provider.provideTasks();
    assert.equal(tasks.length, workspace ? 4 : 0);
    assert.equal(tasks.filter(task => task.group === "build").length, workspace ? 2 : 0);
    assert.deepEqual(h.sourced, []);
    assert.deepEqual(h.probes, []);
    assert.deepEqual(h.events, ["register"]);
  });
}

for (const explicit of [false, true]) {
  test(`build preparation freshly sources ${explicit ? "explicit" : "discovered"} underlay but never a valid self overlay before native help probes`, async () => {
    const workspace = "C:\\fixture user's & workspace!";
    const overlay = path.join(workspace, "install", "setup.bat");
    const setup = explicit ? globalSetup : cachedSetup;
    const h = activationHarness({ files: [cachedSetup, globalSetup, overlay], configured: "lyrical",
      explicit: explicit ? globalSetup : "", workspace,
      source: async (script, baseEnv) => ({ ...baseEnv, ROS_VERSION: "2", ROS_DISTRO: "lyrical",
        ...(script === overlay ? { OVERLAY: "ready" } : { UNDERLAY: "ready" }) }),
    });
    const stale = { ROS_VERSION: "2", ROS_DISTRO: "stale", STALE: "do not reuse" };
    h.extension.env = stale;
    for (let run = 0; run < 2; run++) {
      const base = { Path: `C:\\compiler${run}`, INCLUDE: `SDK${run}`, VisualStudioVersion: "17.0" };
      const prepared = await h.extension.prepareRosBuildEnvironment(base);
      assert.deepEqual(prepared, { ...base, ROS_VERSION: "2", ROS_DISTRO: "lyrical", UNDERLAY: "ready" });
      assert.deepEqual(h.sourceCalls[run].baseEnv, base);
      assert.notEqual(h.sourceCalls[run].baseEnv, base);
      assert.deepEqual(base, { Path: `C:\\compiler${run}`, INCLUDE: `SDK${run}`, VisualStudioVersion: "17.0" });
      for (const [index, command] of ["ros2.exe", "colcon.exe"].entries()) {
        const probe = h.probes[run * 2 + index];
        assert.equal(probe.command, command);
        assert.deepEqual(probe.args, ["--help"]);
        assert.equal(probe.options.env, prepared);
        assert.equal(probe.options.cwd, workspace);
        assert.equal(probe.options.windowsHide, true);
        assert.equal(probe.options.timeout, 30000);
        assert.equal(probe.options.maxBuffer, 1024 * 1024);
      }
    }
    assert.deepEqual(h.sourced, [setup, setup]);
    assert.ok(h.sourceCalls.every(call => call.failOnMissingSetup === true));
    assert.match(h.logs.join("\n"), /Skipping current-workspace install overlay/);
    assert.equal(h.discoveries, explicit ? 0 : 2);
    assert.equal(h.extension.env, stale, "Build preparation must not publish or reuse the activation environment");
    assert.deepEqual(h.registrations, [], "Build preparation must not reload providers");
  });
}

for (const kind of ["explicit", "discovered"]) {
  test(`strict build preparation aborts on ${kind} source failure and shows detailed output`, async () => {
    const workspace = "C:\\fixture workspace";
    const overlay = path.join(workspace, "install", "setup.bat");
    const failedScript = kind === "explicit" ? globalSetup : kind === "overlay" ? overlay : cachedSetup;
    const failure = Object.assign(new Error("fixture setup failed"), { code: 23, signal: "SIGTERM", stderr: "  setup stderr detail\n" });
    const h = activationHarness({ files: [globalSetup, cachedSetup, overlay], configured: "lyrical", workspace,
      explicit: kind === "explicit" ? globalSetup : "", source: async (script, base) => {
        if (script === failedScript) { throw failure; }
        return { ...base, ROS_VERSION: "2", ROS_DISTRO: "lyrical" };
      } });
    await assert.rejects(h.extension.prepareRosBuildEnvironment({ INCLUDE: "SDK" }), error => error === failure);
    assert.deepEqual(h.sourced, kind === "overlay" ? [cachedSetup, overlay] : [failedScript]);
    assert.equal(h.discoveries, kind === "explicit" ? 0 : 1, "Failed explicit build setup must not silently fall back");
    assert.deepEqual(h.probes, []);
    assert.ok(h.logs.some(line => line.includes(`[ROS setup failed] ${failedScript}`)));
    assert.match(h.logs.join("\n"), /fixture setup failed[\s\S]*Exit\/error code: 23[\s\S]*Signal: SIGTERM[\s\S]*setup stderr detail/);
    assert.deepEqual(h.panels, [h.extension.outputChannel]);
  });
}

test("source diagnostics do not duplicate stderr already included in the error message", async () => {
  const h = activationHarness({ files: [cachedSetup], configured: "lyrical", source: async () => {
    throw Object.assign(new Error("setup failed: unique stderr detail"), { code: "ENOENT", stderr: "unique stderr detail\n" });
  } });
  await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /setup failed/);
  assert.equal(h.logs.join("\n").split("unique stderr detail").length - 1, 1);
  assert.ok(h.logs.includes("Exit/error code: ENOENT"));
  assert.equal(h.panels.length, 1);
});

for (const flags of [{}, { ROS_VERSION: "2" }, { ROS_DISTRO: "lyrical" },
  { ROS_VERSION: "1", ROS_DISTRO: "noetic" }, { ROS_VERSION: "2", ROS_DISTRO: "" }]) {
  test(`build preparation rejects incomplete or unsupported ROS flags: ${JSON.stringify(flags)}`, async () => {
    const h = activationHarness({ files: [cachedSetup], configured: "lyrical", source: async () => ({ ...flags }) });
    await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /ROS_VERSION=2 and ROS_DISTRO/);
    assert.deepEqual(h.probes, []);
  });
}

for (const failedCommand of ["ros2.exe", "colcon.exe"]) {
  test(`build preparation rejects a failing ${failedCommand} help probe`, async () => {
    const h = activationHarness({ files: [cachedSetup], configured: "lyrical", probe: async command => {
      if (command === failedCommand) { throw new Error("fixture CLI import failed"); }
      return { stdout: "help", stderr: "" };
    } });
    await assert.rejects(h.extension.prepareRosBuildEnvironment({}), error => {
      assert.ok(error.message.includes(`${failedCommand} --help`));
      assert.match(error.message, /fixture CLI import failed/);
      return true;
    });
    assert.deepEqual(h.probes.map(probe => probe.command), failedCommand === "ros2.exe" ? ["ros2.exe"] : ["ros2.exe", "colcon.exe"]);
  });
}

test("missing overlay permits a first build from a validated underlay", async () => {
  const h = activationHarness({ files: [cachedSetup], configured: "lyrical",
    workspace: "C:\\unbuilt workspace" });
  const prepared = await h.extension.prepareRosBuildEnvironment({ INCLUDE: "SDK" });
  assert.equal(prepared.ROS_VERSION, "2");
  assert.deepEqual(h.sourced, [cachedSetup]);
  assert.equal(h.probes.length, 2);
  assert.match(h.logs.join("\n"), /Skipping current-workspace install overlay/);
});

test("missing explicit build underlay remains fatal even when discovery could succeed", async () => {
  const h = activationHarness({ files: [cachedSetup], configured: "lyrical", explicit: "C:\\missing.bat" });
  await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /Configured ROS underlay is missing/);
  assert.deepEqual(h.sourced, []);
  assert.deepEqual(h.probes, []);
});

for (const files of [[], [cachedSetup]]) {
  test(`missing requested build distro rejects instead of reusing cached activation (${files.length} installs)`, async () => {
    const h = activationHarness({ files, configured: "missing" });
    h.extension.env = { ROS_VERSION: "2", ROS_DISTRO: "stale" };
    await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /No ROS 2 setup script found for "missing"/);
    assert.deepEqual(h.sourced, []);
    assert.deepEqual(h.probes, []);
  });
}

test("partial workspace hooks and repeated self parents cannot prevent a clean recovery build", async () => {
  const workspace = "S:\\ws\\fixture camera";
  const install = path.join(workspace, "install");
  const overlay = path.join(install, "setup.bat");
  const local = path.join(install, "local_setup.bat");
  const packageHook = path.join(install, "share", "camera", "package.bat");
  const external = "D:\\external deps\\install\\local_setup.bat";
  const overlayText = ':: generated from colcon_core/shell/template/prefix_chain.bat.em\n' +
    [cachedSetup, cachedSetup.toLowerCase(), external, `${install}\\local_setup.bat`,
      `${install.toUpperCase()}\\\\local_setup.bat`, "%%~dp0local_setup.bat"]
      .map(script => `call:_colcon_prefix_chain_bat_call_script "${script}"`).join("\n");
  const h = activationHarness({ files: [cachedSetup, overlay, local, packageHook, external],
    configured: "lyrical", workspace, overlayText, source: async (script, base) => {
      assert.notEqual(script, overlay, "Must not try the broken workspace chain, even speculatively");
      assert.notEqual(script, local, "Must not execute incomplete package hooks");
      assert.equal(base.CAMERA_DIR, undefined);
      assert.equal(base.PARTIAL_HOOK, undefined);
      return { ...base, ROS_VERSION: "2", ROS_DISTRO: "lyrical",
        ...(script === external ? { EXTERNAL_HOOK: "sourced" } : { UNDERLAY: "sourced" }) };
    } });
  h.extension.env = { PARTIAL_HOOK: "left behind", CAMERA_DIR: install };
  const inherited = { Path: `${install}\\bin;C:\\compiler;D:\\external deps\\install\\bin`,
    COLCON_PREFIX_PATH: `${install};${install.toUpperCase()}\\;D:\\external deps\\install`,
    AMENT_PREFIX_PATH: `${install}\\camera;D:\\external deps\\install`,
    CMAKE_PREFIX_PATH: `${install};D:\\external deps\\install`,
    PYTHONPATH: `${install}\\Lib\\site-packages;D:\\external deps\\install\\python`,
    CAMERA_DIR: `${install}\\share\\camera`, INCLUDE: "SDK", LIB: "SDK libs" };
  const before = { ...inherited };
  const messages = [];
  const prepared = await h.extension.prepareRosBuildEnvironment(inherited,
    { cwd: workspace, args: ["build", "--packages-select", "camera"], onOutput: message => messages.push(message) });
  assert.deepEqual(h.sourced, [cachedSetup, external], "Keep the legitimate external parent, deduplicate the selected ROS parent");
  assert.equal(prepared.EXTERNAL_HOOK, "sourced");
  assert.equal(prepared.COLCON_PREFIX_PATH, "D:\\external deps\\install");
  assert.equal(prepared.Path, "C:\\compiler;D:\\external deps\\install\\bin");
  assert.equal(prepared.PYTHONPATH, "D:\\external deps\\install\\python");
  assert.equal(prepared.CAMERA_DIR, undefined);
  assert.equal(prepared.PARTIAL_HOOK, undefined);
  assert.deepEqual(inherited, before);
  assert.equal(h.extension.env.PARTIAL_HOOK, "left behind", "Runtime state must not be published or mutated");
  assert.equal(h.probes.length, 2);
  assert.match(messages.join("\n"), /Skipping current-workspace install overlay[\s\S]*--packages-up-to/);
});

test("a broken recorded external parent is fatal, not silently discarded for recovery", async () => {
  const workspace = "C:\\fixture workspace";
  const overlay = path.join(workspace, "install", "setup.bat");
  const parent = "D:\\external\\local_setup.bat";
  const h = activationHarness({ workspace, files: [cachedSetup, overlay, parent], configured: "lyrical",
    overlayText: ':: generated from colcon_core/shell/template/prefix_chain.bat.em\n' +
      `call:_colcon_prefix_chain_bat_call_script "${parent}"`,
    source: async (script, base) => {
      if (script === parent) { base.PARTIAL = "must never be used"; throw new Error("external parent missing hooks"); }
      return { ...base, ROS_VERSION: "2", ROS_DISTRO: "lyrical" };
    } });
  await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /external parent missing hooks/);
  assert.deepEqual(h.sourced, [cachedSetup, parent]);
  assert.deepEqual(h.probes, []);
});

test("selected self-overlay and external underlay that reintroduces self paths fail explicitly", async () => {
  const workspace = "C:\\fixture workspace";
  const overlay = path.join(workspace, "install", "setup.bat");
  const self = activationHarness({ workspace, files: [overlay], explicit: overlay });
  await assert.rejects(self.extension.prepareRosBuildEnvironment({}), /Selected ROS underlay is inside/);
  assert.deepEqual(self.sourced, []);
  const h = activationHarness({ workspace, files: [cachedSetup], configured: "lyrical", source: async () => ({
    ROS_VERSION: "2", ROS_DISTRO: "lyrical", COLCON_PREFIX_PATH: path.join(workspace, "install"), PARTIAL: "hook",
  }) });
  await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /reintroduced the current workspace install/);
  assert.deepEqual(h.probes, []);
});

test("inherited ROS identity cannot make a broken selected underlay pass validation", async () => {
  const h = activationHarness({ files: [cachedSetup], configured: "lyrical", source: async (_script, base) => ({ ...base }) });
  await assert.rejects(h.extension.prepareRosBuildEnvironment({ ROS_VERSION: "2", ROS_DISTRO: "stale" }), /ROS_VERSION=2/);
  assert.deepEqual(h.probes, []);
});

test("runtime activation still sources valid overlays and reports broken overlays", async () => {
  const workspace = "C:\\fixture workspace";
  const overlay = path.join(workspace, "install", "setup.bat");
  for (const broken of [false, true]) {
    const h = await activate({ workspace, files: [cachedSetup, overlay], configured: "lyrical", source: async (script, base) => {
      if (script === overlay && broken) { throw new Error("runtime missing package hook"); }
      return { ...base, ROS_VERSION: "2", ROS_DISTRO: "lyrical", ...(script === overlay ? { RUNTIME: "yes" } : {}) };
    } });
    assert.ok(h.sourced.includes(overlay));
    assert.ok(h.sourceCalls.every(call => !call.failOnMissingSetup));
    assert.equal(h.env.RUNTIME, broken ? undefined : "yes");
    assert.equal(h.errors.length, broken ? 2 : 0);
    if (broken) { assert.match(h.logs.join("\n"), /runtime missing package hook/); }
    assert.doesNotMatch(h.logs.join("\n"), /Build recovery/);
  }
});

test("postbuild runtime sources local hooks strictly, not the contaminated prefix chain", async () => {
  const workspace = "C:\\fixture workspace";
  const local = path.join(workspace, "install", "local_setup.bat");
  for (const broken of [false, true]) {
    const h = activationHarness({ workspace, files: [local], source: async (_script, base) => {
      if (broken) { throw new Error("runtime local hook missing"); }
      return { ...base, RUNTIME: "ready" };
    } });
    const base = { ROS_VERSION: "2", ROS_DISTRO: "lyrical" };
    const result = h.extension.prepareRosTestEnvironment(base, workspace);
    if (broken) { await assert.rejects(result, /runtime local hook missing/); }
    else { assert.equal((await result).RUNTIME, "ready"); }
    assert.deepEqual(h.sourced, [local]);
    assert.equal(h.sourceCalls[0].failOnMissingSetup, true);
    assert.equal(base.RUNTIME, undefined);
  }
});

test("external underlay cannot silently replace the selected ROS distro", async () => {
  const workspace = "C:\\fixture workspace";
  const overlay = path.join(workspace, "install", "setup.bat");
  const external = "D:\\old ROS\\local_setup.bat";
  const h = activationHarness({ workspace, files: [cachedSetup, overlay, external], configured: "lyrical",
    overlayText: ':: generated from colcon_core/shell/template/prefix_chain.bat.em\n' +
      `call:_colcon_prefix_chain_bat_call_script "${external}"`,
    source: async (script, base) => ({ ...base, ROS_VERSION: "2", ROS_DISTRO: script === external ? "jazzy" : "lyrical" }) });
  await assert.rejects(h.extension.prepareRosBuildEnvironment({}), /changed the selected ROS distro/);
  assert.deepEqual(h.probes, []);
});

test("real CMD partial install is skipped and a child receives a clean recovery environment", {
  skip: process.platform !== "win32",
}, async t => {
  const root = await fs.mkdtemp(path.join(os.tmpdir(), "RDE recovery user's & fixture!-"));
  t.after(() => fs.rm(root, { recursive: true, force: true }));
  const workspace = path.join(root, "workspace");
  const install = path.join(workspace, "install");
  const overlay = path.join(install, "setup.bat");
  const local = path.join(install, "local_setup.bat");
  const packageHook = path.join(install, "share", "camera", "package.bat");
  const underlay = path.join(root, "ROS", "local_setup.bat");
  const external = path.join(root, "external", "local_setup.bat");
  const sdk = path.join(root, "SDK");
  const bin = path.join(root, "tools");
  const scripts = new Map([
    [underlay, '@echo off\r\nset "ROS_VERSION=2"\r\nset "ROS_DISTRO=lyrical"\r\n'],
    [external, '@echo off\r\nset "EXTERNAL_READY=yes"\r\n'],
    [local, `@echo off\r\nset "PARTIAL_HOOK=must not escape"\r\ncall "${packageHook}"\r\n`],
    [packageHook, `@echo off\r\ncall "${path.dirname(packageHook)}\\local_setup.bat"\r\nexit /b 23\r\n`],
    [overlay, ':: generated from colcon_core/shell/template/prefix_chain.bat.em\r\n@echo off\r\n' +
      [underlay, external, local, local.replace("\\local_setup", "\\\\local_setup"), "%%~dp0local_setup.bat"]
        .map(script => `call:_colcon_prefix_chain_bat_call_script "${script}"`).join("\r\n") +
      '\r\ngoto:eof\r\n:_colcon_prefix_chain_bat_call_script\r\ncall "%~1"\r\ngoto:eof\r\n'],
  ]);
  for (const file of ["tools/cl.exe", "tools/link.exe", "tools/rc.exe", "SDK/Include/10.0/um/Windows.h",
    "SDK/Include/10.0/ucrt/stdio.h", "SDK/Lib/10.0/um/x64/kernel32.lib", "SDK/Lib/10.0/ucrt/x64/ucrt.lib"]) {
    scripts.set(path.join(root, file), "");
  }
  for (const [filename, content] of scripts) {
    await fs.mkdir(path.dirname(filename), { recursive: true });
    await fs.writeFile(filename, content);
  }
  const base = Object.fromEntries(Object.entries(process.env).filter(([key]) => key.toLowerCase() !== "path"));
  Object.assign(base, { Path: `${install}\\bin;${bin};${process.env.Path || process.env.PATH}`,
    VisualStudioVersion: "17.0", VSCMD_ARG_TGT_ARCH: "x64", WindowsSdkDir: sdk, WindowsSDKVersion: "10.0\\",
    INCLUDE: sdk, LIB: sdk, COLCON_PREFIX_PATH: `${install};${install.toUpperCase()}\\` });
  await assert.rejects(sourceWindowsEnvironment(local, base), error => error.code === 23);
  const h = activationHarness({ workspace, explicit: underlay, files: [...scripts.keys()],
    overlayText: scripts.get(overlay), source: (script, env, strict) =>
      sourceWindowsEnvironment(script, env, { failOnMissingSetup: strict }) });
  const prepared = await h.extension.prepareRosBuildEnvironment(base);
  assert.deepEqual(h.sourced, [underlay, external]);
  const { stdout } = await promisify(childProcess.execFile)(process.execPath,
    ["-e", "console.log(JSON.stringify({partial:process.env.PARTIAL_HOOK,external:process.env.EXTERNAL_READY,ros:process.env.ROS_DISTRO,prefix:process.env.COLCON_PREFIX_PATH}))"],
    { env: prepared, cwd: workspace });
  assert.deepEqual(JSON.parse(stdout), { external: "yes", ros: "lyrical" });
  assert.equal(await fs.readFile(overlay, "utf8"), scripts.get(overlay));
  await assert.rejects(fs.access(path.join(path.dirname(packageHook), "local_setup.bat")), /ENOENT/);
  assert.equal(h.probes.length, 2, "Both CLI checks must run after successful recovery preparation");
});