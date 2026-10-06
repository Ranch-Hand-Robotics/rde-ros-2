const assert = require("node:assert/strict");
const { EventEmitter } = require("node:events");
const fs = require("node:fs");
const Module = require("node:module");
const os = require("node:os");
const path = require("node:path");
const { PassThrough } = require("node:stream");
const { test } = require("node:test");
const { promisify } = require("node:util");
const manifest = require("../package.json");

// Load the compiled implementation, not a reimplementation of its private methods.
// A module-local process permits platform coverage without changing the host process.
// Mock installation is synchronous; no Module._load override survives an await.
function loadWithMocks(filename, mocks, platform = process.platform, hostProcess) {
  const resolved = require.resolve(filename);
  const original = Module._load;
  const loaded = new Module(resolved, module);
  loaded.filename = resolved;
  loaded.paths = Module._nodeModulePaths(path.dirname(resolved));
  loaded.testProcess = hostProcess ?? { ...process, platform, env: {} };
  Module._load = function(request, parent, isMain) {
    return Object.hasOwn(mocks, request) ? mocks[request] : original.call(this, request, parent, isMain);
  };
  try {
    loaded._compile("const process = module.testProcess;\n" + fs.readFileSync(resolved, "utf8"), resolved);
    return loaded.exports;
  } finally {
    Module._load = original;
  }
}

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function fakeChild() {
  const child = new EventEmitter();
  child.pid = 4242;
  child.stdout = new PassThrough();
  child.stderr = new PassThrough();
  child.input = [];
  child.stdin = { write: text => { child.input.push(text); return true; } };
  child.kills = 0;
  child.kill = () => { child.kills++; queueMicrotask(() => child.emit("close", null)); return true; };
  return child;
}

class FakeVsCodeEventEmitter {
  constructor() {
    this.emitter = new EventEmitter();
    this.event = listener => {
      this.emitter.on("event", listener);
      return { dispose: () => this.emitter.off("event", listener) };
    };
  }
  fire(value) { this.emitter.emit("event", value); }
  dispose() { this.emitter.removeAllListeners(); }
}

function harness(settings = {}) {
  const h = {
    activations: [], warnings: [], copied: [], spawns: [], kills: [], logs: [], registrations: [],
    spawned: deferred(), workspace: settings.workspace ?? "C:\\ROS user's & workspace!", envReads: 0,
    preparations: [], errors: [], events: [], packageChecks: [], packageLists: [],
    preparationOptions: [], runtimePreparations: [], debugSessions: [],
  };
  const platform = settings.platform ?? "win32";
  // All loaded consumers share a live fixture baseline, never the real process.env.
  h.process = { ...process, platform, env: { ...(settings.hostEnv ?? {
    Path: "C:\\host\\Scripts", ROS_DISTRO: "jazzy", SystemRoot: "C:\\Windows",
  }) } };
  const debugEnds = new FakeVsCodeEventEmitter();
  h.vscode = {
    EventEmitter: FakeVsCodeEventEmitter,
    CustomExecution: class { constructor(callback) { this.callback = callback; } },
    ShellExecution: class {
      constructor(command, args, options) { Object.assign(this, { command, args, options }); }
    },
    Task: class {
      constructor(definition, scope, name, source) { Object.assign(this, { definition, scope, name, source }); }
    },
    TaskScope: { Workspace: 2 },
    TaskGroup: { Build: { id: "build" }, Test: { id: "test" } },
    debug: {
      startDebugging: async (folder, config) => {
        h.events.push("debug");
        h.debugSessions.push({ folder, config });
        setImmediate(() => debugEnds.fire({ name: config.name }));
        return true;
      },
      onDidTerminateDebugSession: debugEnds.event,
    },
    workspace: {
      rootPath: h.workspace, workspaceFolders: [{ uri: { fsPath: h.workspace } }],
      findFiles: () => assert.fail("Task discovery must not search the filesystem"),
    },
    window: { showErrorMessage: async (...args) => { h.errors.push(args); }, showWarningMessage: async (...args) => {
      h.warnings.push(args);
      return typeof settings.choice === "function" ? settings.choice(...args) : settings.choice;
    } },
    env: { clipboard: { writeText: async text => { h.copied.push(text); } } },
    tasks: { registerTaskProvider: (type, provider) => {
      h.registrations.push({ type, provider });
      return { dispose() {} };
    } },
  };
  h.extension = {
    env: settings.env ?? { Path: "C:\\old ROS\\Scripts", ROS_DISTRO: "stale", STALE_RUNTIME_HOOK: "old scalar" },
    outputChannel: { appendLine: text => h.logs.push(text) },
    resolvedEnv: async () => { h.envReads++; return h.extension.env; },
    prepareRosBuildEnvironment: async (env, options) => {
      h.events.push("ros");
      h.preparations.push(env);
      h.preparationOptions.push(options);
      return settings.prepareRos ? settings.prepareRos(env, options) : env;
    },
    prepareRosTestEnvironment: async (env, workspace) => {
      h.events.push("runtime");
      h.runtimePreparations.push({ env, workspace });
      return settings.prepareRuntime ? settings.prepareRuntime(env, workspace) : { ...env, RUNTIME_OVERLAY: "fresh" };
    },
  };
  const toolchain = settings.toolchain ?? {
    WINDOWS_BUILD_TOOLS_COMMAND: "review-only fixture install command",
    activateWindowsToolchain: async (env, options) => {
      h.events.push("compiler");
      h.activations.push({ env, options });
      return settings.activate ? settings.activate(env, options) : env;
    },
  };
  h.installCommand = toolchain.WINDOWS_BUILD_TOOLS_COMMAND;
  h.preflight = loadWithMocks("../out/src/build-tool/windows-build-preflight", {
    vscode: h.vscode, "../ros/windows-toolchain": toolchain,
  });
  h.cp = {
    spawn: (command, args, options) => {
      h.events.push("spawn");
      const child = fakeChild();
      h.spawns.push({ command, args, options, child });
      if (settings.spawnError) { throw settings.spawnError; }
      h.spawned.resolve(child);
      if (settings.autoClose !== false) { queueMicrotask(() => child.emit("close", 0)); }
      return child;
    },
    execFile: (command, args, options, callback) => {
      h.kills.push({ command, args, options });
      callback(settings.killError);
    },
  };
  const execution = loadWithMocks("../out/src/build-tool/windows-colcon-task", {
    vscode: h.vscode, child_process: h.cp, "./windows-build-preflight": h.preflight,
  }, platform, h.process);
  const deferredExecution = loadWithMocks("../out/src/build-tool/deferred-ros-task", {
    vscode: h.vscode, child_process: h.cp,
  }, platform, h.process);
  h.shell = loadWithMocks("../out/src/build-tool/ros-shell", {
    vscode: h.vscode, "../extension": h.extension, "./windows-colcon-task": execution,
    "./deferred-ros-task": deferredExecution,
  }, platform, h.process);
  const noDiscoveryPrerequisite = new Proxy({}, {
    get: (_, name) => assert.fail(`Colcon task discovery must not access filesystem/subprocess APIs: ${String(name)}`),
  });
  const colconUtils = loadWithMocks("../out/src/build-tool/colcon-utils", {
    vscode: h.vscode, "../extension": h.extension,
    "../vscode-utils": { getExtensionConfiguration: () => ({ get: (key, fallback) => {
      assert.equal(key, "colconIgnore");
      return settings.ignored ?? fallback;
    } }) },
    child_process: { execFile: () => assert.fail("Skip configuration must not execute colcon") },
  }, platform);
  h.colcon = loadWithMocks("../out/src/build-tool/colcon", {
    vscode: h.vscode, "./ros-shell": h.shell,
    fs: noDiscoveryPrerequisite, "node:fs": noDiscoveryPrerequisite,
    "fs/promises": noDiscoveryPrerequisite, "node:fs/promises": noDiscoveryPrerequisite,
    child_process: noDiscoveryPrerequisite, "node:child_process": noDiscoveryPrerequisite,
    "../vscode-utils": { workspaceContainsPackageXml: async depth => {
      h.packageChecks.push(depth);
      assert.fail("Task discovery must not require package.xml");
    } },
    "./colcon-utils": {
      ...colconUtils,
      getNonIgnoredPackages: async () => {
        assert.notEqual(platform, "win32", "Windows task discovery must not run colcon list");
        h.packageLists.push("getNonIgnoredPackages");
        return ["camera"];
      },
      getPackages: async () => { h.packageLists.push("getPackages"); return [{ name: "camera" }]; },
    },
  }, platform);
  h.buildTool = loadWithMocks("../out/src/build-tool/build-tool", {
    vscode: h.vscode, "../extension": h.extension, "../telemetry-helper": {},
    "./colcon": h.colcon, "../vscode-utils": { getWorkspaceFolder: () => h.workspace },
  }, platform);
  const { RosTestRunner } = loadWithMocks("../out/src/test-provider/ros-test-runner", {
    vscode: h.vscode, child_process: h.cp, "../extension": h.extension,
    "../vscode-utils": {
      isCppToolsExtensionInstalled: () => true,
      isLldbExtensionInstalled: () => false,
      isCursorEditor: () => false,
    },
    "./ros-test-provider": { TestType: { CppGtest: "cpp_gtest" } },
    "./test-discovery-utils": { TestDiscoveryUtils: { getCppTestExecutable: () => "camera_test" } },
    "../build-tool/windows-build-preflight": h.preflight,
    "../build-tool/colcon-utils": colconUtils,
  }, platform, h.process);
  h.runner = new RosTestRunner({});
  h.make = (definition = {}) => h.shell.make("fixture build", {
    type: "colcon", command: "colcon", args: ["build"], ...definition,
  });
  return h;
}

async function terminal(task, definition = task.definition) {
  const pty = await task.execution.callback(definition);
  const writes = [];
  const codes = [];
  const closed = deferred();
  pty.onDidWrite(text => writes.push(text));
  pty.onDidClose(code => { codes.push(code); closed.resolve(code); });
  return { pty, writes, codes, done: closed.promise };
}

const flush = () => new Promise(resolve => setImmediate(resolve));

for (const mode of ["task default install", "task --install-base", "task --install-base=", "Test Explorer"]) {
  test(`${mode}: compiler receives clean current-install paths while external underlays and SDKs survive`, async () => {
    const realEnv = { ...process.env };
    const workspace = path.resolve(os.tmpdir(), "ROS clean user's & workspace!");
    const customInstall = mode.startsWith("task --install-base");
    const cwd = customInstall ? path.join(workspace, "nested build") : workspace;
    const installBase = path.join("artifacts", "custom install");
    const install = path.resolve(cwd, customInstall ? installBase : "install");
    const workspaceInstall = path.join(workspace, "install");
    const external = path.resolve(os.tmpdir(), "external ROS underlay");
    const sdk = path.resolve(os.tmpdir(), "external Windows SDK");
    const sibling = path.join(workspace, "install-external", "bin");
    const expected = {
      Path: [path.join(external, "bin"), path.join(sdk, "bin"), sibling].join(";"),
      aMeNt_PrEfIx_PaTh: external, CMAKE_PREFIX_PATH: external, COLCON_PREFIX_PATH: external,
      PYTHONPATH: path.join(external, "Lib"),
      INCLUDE: path.join(sdk, "include"), LIB: path.join(sdk, "lib"), HOST_SCALAR: "live baseline",
    };
    const hostEnv = {
      ...expected,
      Path: [path.join(workspaceInstall, "bin"), path.join(install, "camera", "bin"), expected.Path].join(";"),
      aMeNt_PrEfIx_PaTh: [workspaceInstall, install, external].join(";"),
      CMAKE_PREFIX_PATH: [install, external, workspaceInstall].join(";"),
      COLCON_PREFIX_PATH: [workspaceInstall, external, install].join(";"),
      PYTHONPATH: [path.join(install, "Lib"), expected.PYTHONPATH].join(";"),
      INCLUDE: [path.join(install, "include"), expected.INCLUDE].join(";"),
      LIB: [path.join(install, "lib"), expected.LIB].join(";"),
      CURRENT_INSTALL_HOOK: install,
    };
    const h = harness({ workspace, hostEnv });
    const baseline = { ...h.process.env };
    const runtimeSnapshot = { ...h.extension.env };
    if (mode === "Test Explorer") {
      await h.runner.buildTestExecutable("camera", false);
      assert.deepEqual(h.preparationOptions[0], { cwd });
      assert.ok(h.spawns[0].args.includes("--packages-up-to"));
      assert.ok(!h.spawns[0].args.includes("--packages-select"));
    } else {
      const args = ["build", "--packages-select", "camera"];
      if (mode === "task --install-base") { args.push("--install-base", installBase); }
      if (mode === "task --install-base=") { args.push(`--install-base=${installBase}`); }
      const task = h.make({ args, buildOptions: { cwd } });
      const run = await terminal(task);
      run.pty.open();
      assert.equal(await run.done, 0);
      assert.equal(h.preparationOptions[0].cwd, cwd);
      assert.equal(h.preparationOptions[0].args, args, "Forward resolved cwd and install-base arguments to ROS preparation");
      assert.deepEqual(h.spawns[0].args, args, "Custom --packages-select remains unchanged");
      h.preparationOptions[0].onOutput("fixture underlay diagnostic");
      assert.ok(h.logs.includes("fixture underlay diagnostic"));
    }
    assert.deepEqual(h.activations[0].env, expected, "Cleaning must happen before compiler activation, not just ROS preparation");
    assert.equal(h.activations[0].options.cwd, cwd);
    assert.equal(h.preparations[0], h.activations[0].env);
    assert.deepEqual(h.spawns[0].options.env, expected);
    assert.equal(h.spawns[0].options.env.STALE_RUNTIME_HOOK, undefined, "Arbitrary non-path runtime hook scalars must never enter a build");
    assert.equal(h.spawns[0].options.cwd, cwd);
    assert.deepEqual(h.events, ["compiler", "ros", "spawn"]);
    assert.deepEqual(h.process.env, baseline);
    assert.deepEqual(h.extension.env, runtimeSnapshot);
    assert.deepEqual({ ...process.env }, realEnv, "Fixtures must not contaminate the real process environment");
  });
}

test("provider discovery, package tasks, registration and resolution never check or prompt before open", async () => {
  const h = harness();
  assert.equal(await h.buildTool.determineBuildTool(h.workspace), true);
  h.buildTool.BuildTool.registerTaskProvider();
  assert.equal(h.registrations.length, 1);
  const { type, provider } = h.registrations[0];
  assert.equal(type, "colcon");
  assert.ok(provider instanceof h.colcon.ColconProvider);
  h.packageLists.length = 0;
  const tasks = await provider.provideTasks();
  assert.equal(tasks.length, 4);
  assert.deepEqual(h.packageChecks, []);
  assert.deepEqual(h.packageLists, [], "Listing build tasks must not execute colcon");
  for (const task of tasks) {
    assert.equal(task.definition.type, "colcon");
    assert.deepEqual(task.problemMatchers, ["$colcon-gcc"]);
    assert.ok(task.definition.args.includes("--merge-install"));
    assert.ok(!task.definition.args.includes("--packages-select"));
    const build = task.definition.args[0] === "build";
    assert.equal(task.group, build ? h.vscode.TaskGroup.Build : h.vscode.TaskGroup.Test);
    assert.equal(task.execution instanceof h.vscode.CustomExecution, build);
    if (build) {
      assert.equal(task.definition.buildOptions.cwd, "${workspaceFolder}");
      await terminal(task);
    }
  }
  for (const buildType of ["Debug", "RelWithDebInfo"]) {
    const task = await h.colcon.makeColconPackageTask("my_camera", buildType);
    assert.ok(task.execution instanceof h.vscode.CustomExecution);
    assert.equal(task.group, h.vscode.TaskGroup.Build);
    assert.deepEqual(task.definition.args.slice(-4), ["--packages-up-to", "my_camera", "--cmake-args", `-DCMAKE_BUILD_TYPE=${buildType}`]);
    assert.ok(!task.definition.args.includes("--packages-select"));
    await terminal(task);
  }
  const input = {
    definition: { type: "colcon", args: ["build"], options: { cwd: "${workspaceFolder}/nested" } },
    isBackground: true, problemMatchers: ["$custom"],
  };
  const resolved = provider.resolveTask(input);
  assert.equal(resolved, input, "Keep VS Code's task and its exact definition during resolution");
  assert.equal(resolved.definition, input.definition);
  assert.ok(resolved.execution instanceof h.vscode.CustomExecution);
  assert.equal(resolved.definition.command, "colcon");
  assert.equal(resolved.definition.options.cwd, "${workspaceFolder}/nested");
  assert.equal(resolved.isBackground, true);
  assert.deepEqual(resolved.problemMatchers, ["$custom"]);
  assert.equal(provider.resolveTask({ definition: { type: "shell" } }), undefined);
  await terminal(resolved);
  await flush();
  assert.deepEqual(h.activations, []);
  assert.deepEqual(h.preparations, []);
  assert.deepEqual(h.warnings, []);
  assert.deepEqual(h.spawns, []);
  assert.ok(manifest.scripts["test:discovery"].includes("test/windows-build-preflight.test.js"));
  assert.ok(manifest.contributes.taskDefinitions.some(definition => definition.type === type));
});

test("generated provider/package builds and resolved tasks all execute the shared preflight", async () => {
  const h = harness({ activate: async () => { throw new Error("fixture missing compiler"); } });
  const provider = new h.colcon.ColconProvider();
  const tasks = (await provider.provideTasks()).filter(task => task.group === h.vscode.TaskGroup.Build);
  tasks.push(await h.colcon.makeColconPackageTask("camera", "Debug"));
  tasks.push(await h.colcon.makeColconPackageTask("camera", "RelWithDebInfo"));
  tasks.push(provider.resolveTask({ definition: { type: "colcon", args: ["build", "--packages-select", "camera"] } }));
  tasks.push(new h.shell.RosShellTaskProvider().resolveTask({
    definition: { type: "colcon", command: "colcon", args: ["build"] },
  }));
  assert.equal(tasks.length, 6);
  assert.equal(h.activations.length, 0);
  for (const [index, task] of tasks.entries()) {
    // Emulate VS Code supplying the resolved definition to CustomExecution.
    const definition = { ...task.definition, buildOptions: { ...task.definition.buildOptions, cwd: h.workspace } };
    const run = await terminal(task, definition);
    assert.equal(h.activations.length, index);
    run.pty.open();
    assert.equal(await run.done, 1);
    assert.equal(h.activations.length, index + 1);
    assert.equal(h.activations[index].options.cwd, h.workspace);
    assert.equal(h.warnings.length, index + 1);
  }
  assert.deepEqual(h.preparations, [], "Compiler failures must stop before sourcing ROS");
  assert.deepEqual(h.spawns, []);
});

test("ready/activated environment and resolved args, cwd and case-insensitive overrides reach child unchanged", async () => {
  const gate = deferred();
  const current = { Path: "old", PATH: "duplicate", ROS_DISTRO: "jazzy", DELETE_ME: "old", EMPTY: "old" };
  let activated;
  const h = harness({ hostEnv: current, activate: async (env, options) => {
    options.onOutput("compiler activated");
    await gate.promise;
    activated = { ...env, VisualStudioVersion: "17.0", INCLUDE: "SDK includes" };
    return activated;
  } });
  const task = h.make({ args: ["build", "${input:package}"], options: { env: { pAtH: "${env:Path}" } } });
  const args = ["--log-base", "C:\\log & files", "build", "--packages-select", "camera & lidar", "--cmake-args", "-DFOO=a b"];
  const definition = { ...task.definition, args, buildOptions: {
    cwd: h.workspace, env: { pAtH: "C:\\resolved ROS\\Scripts", delete_me: null, EMPTY: "", NEW: "resolved" },
  } };
  const run = await terminal(task, definition);
  run.pty.open();
  assert.equal(h.spawns.length, 0, "Spawn must await activation");
  assert.equal(h.activations.length, 1);
  assert.equal(h.activations[0].options.cwd, h.workspace);
  assert.deepEqual(h.activations[0].env, {
    ROS_DISTRO: "jazzy", pAtH: "C:\\resolved ROS\\Scripts", EMPTY: "", NEW: "resolved",
  });
  gate.resolve();
  assert.equal(await run.done, 0);
  const spawn = h.spawns[0];
  assert.equal(spawn.command, "colcon.exe");
  assert.equal(spawn.args, args);
  assert.ok(!spawn.args.includes("--packages-up-to"), "Custom --packages-select must not be rewritten");
  assert.equal(spawn.options.env, activated);
  assert.equal(h.preparations[0], activated);
  assert.equal(h.preparationOptions[0].cwd, h.workspace);
  assert.equal(h.preparationOptions[0].args, args);
  assert.equal(typeof h.preparationOptions[0].onOutput, "function");
  assert.deepEqual(h.events, ["compiler", "ros", "spawn"]);
  assert.equal(spawn.options.cwd, h.workspace);
  assert.equal(spawn.options.shell, false);
  assert.equal(spawn.options.windowsHide, true);
  assert.equal(spawn.options.stdio, "pipe");
  assert.deepEqual(current, { Path: "old", PATH: "duplicate", ROS_DISTRO: "jazzy", DELETE_ME: "old", EMPTY: "old" });
  assert.equal(task.definition.args[1], "${input:package}", "The unresolved definition is not used or mutated");
  assert.deepEqual(h.logs, ["compiler activated"]);
  assert.match(run.writes.join(""), /compiler activated\r\n/);
  assert.deepEqual(h.warnings, []);
});

for (const choice of ["Copy Install Command", "Cancel Build", undefined]) {
  test(`missing toolchain stops before spawn: ${choice ?? "dismiss"}`, async () => {
    const h = harness({ choice, activate: async () => { throw new Error("MSVC/SDK missing"); } });
    const run = await terminal(h.make());
    run.pty.open();
    assert.equal(await run.done, 1);
    assert.equal(h.spawns.length, 0);
    assert.deepEqual(h.preparations, [], "Do not source ROS before offering compiler repair");
    assert.equal(h.kills.length, 0, "No installer or other child process may run");
    assert.equal(h.warnings.length, 1);
    const [message, options, ...actions] = h.warnings[0];
    assert.match(message, /Colcon build stopped/);
    assert.equal(options.modal, true);
    assert.ok(message.length + options.detail.length < 200, "Keep the dialog concise");
    assert.match(options.detail, /Visual Studio 2022 C\+\+ tools and a Windows SDK/);
    assert.match(options.detail, /Output > ROS 2/);
    assert.doesNotMatch(options.detail, /MSVC\/SDK missing|Administrator PowerShell/);
    assert.deepEqual(actions, ["Copy Install Command", "Cancel Build"]);
    assert.deepEqual(h.copied, choice === "Copy Install Command" ? [h.installCommand] : []);
    assert.match(h.logs.join("\n"), /preflight failed.*MSVC\/SDK missing/);
    assert.match(h.logs.join("\n"), /Visual Studio Installer > Modify/);
    assert.match(h.logs.join("\n"), /Administrator PowerShell/);
    assert.match(h.logs.join("\n"), /Nothing will be installed automatically/);
    assert.match(run.writes.join(""), /Build stopped before launching colcon/);
  });
}

test("long toolchain diagnostics stay in the log, not the preflight dialog", async () => {
  const detail = "fixture setup diagnostic ".repeat(200);
  const h = harness({ activate: async () => { throw new Error(detail); } });
  const run = await terminal(h.make());
  run.pty.open();
  assert.equal(await run.done, 1);
  const [message, options] = h.warnings[0];
  assert.ok(message.length + options.detail.length < 200);
  assert.doesNotMatch(options.detail, /fixture setup diagnostic/);
  assert.ok(h.logs.join("\n").includes(detail));
  assert.deepEqual(h.spawns, []);
});

test("each run reads the live host baseline independently of the runtime extension environment", async () => {
  let ready = true;
  const h = harness({ activate: async env => {
    if (!ready) { throw new Error("removed after previous build"); }
    return { ...env, ACTIVATED: "yes" };
  } });
  const task = h.make();
  for (const [index, canBuild] of [true, false, true].entries()) {
    ready = canBuild;
    const run = await terminal(task);
    h.process.env = { Path: `C:\\host ROS${index}`, ROS_DISTRO: `distro${index}` };
    h.extension.env = { Path: `C:\\stale ROS${index}`, ROS_DISTRO: "stale", STALE_RUNTIME_HOOK: `old scalar ${index}` };
    run.pty.open();
    assert.equal(await run.done, canBuild ? 0 : 1);
    assert.equal(h.activations[index].env.ROS_DISTRO, `distro${index}`);
    assert.equal(h.activations[index].env.Path, `C:\\host ROS${index}`);
    assert.equal(h.activations[index].env.STALE_RUNTIME_HOOK, undefined);
  }
  assert.equal(h.activations.length, 3);
  assert.equal(h.spawns.length, 2);
  assert.equal(h.preparations.length, 2);
  assert.equal(h.spawns[1].options.env.ROS_DISTRO, "distro2");
  assert.equal(h.spawns[1].options.env.ACTIVATED, "yes");
  assert.equal(h.warnings.length, 1);
});

test("undefined extension environment still uses the isolated current host baseline", async () => {
  const h = harness();
  h.extension.env = undefined;
  const run = await terminal(h.make());
  run.pty.open();
  assert.equal(await run.done, 0);
  assert.deepEqual(h.activations[0].env, h.process.env);
  assert.notEqual(h.activations[0].env, h.process.env, "Cleaning must not mutate the baseline");
});

for (const removed of ["tools/cl.exe", "tools/link.exe", "tools/rc.exe", "SDK/Include/10.0/um/Windows.h",
  "SDK/Include/10.0/ucrt/stdio.h", "SDK/Lib/10.0/um/x64/kernel32.lib", "SDK/Lib/10.0/ucrt/x64/ucrt.lib"]) {
  test(`real readiness/activation rejects a stale toolchain after removing ${removed}`, async t => {
    const root = await fs.promises.mkdtemp(path.join(os.tmpdir(), "ROS build user's & fixture!-"));
    t.after(() => fs.promises.rm(root, { recursive: true, force: true }));
    const files = ["tools/cl.exe", "tools/link.exe", "tools/rc.exe", "SDK/Include/10.0/um/Windows.h",
      "SDK/Include/10.0/ucrt/stdio.h", "SDK/Lib/10.0/um/x64/kernel32.lib", "SDK/Lib/10.0/ucrt/x64/ucrt.lib",
      "Microsoft Visual Studio/Installer/vswhere.exe"];
    for (const file of files) {
      await fs.promises.mkdir(path.dirname(path.join(root, file)), { recursive: true });
      await fs.promises.writeFile(path.join(root, file), "");
    }
    let discoveries = 0;
    const execFile = () => assert.fail("Discovery must use promisified execFile");
    execFile[promisify.custom] = async (command, args, options) => {
      discoveries++;
      assert.equal(command, path.join(root, "Microsoft Visual Studio", "Installer", "vswhere.exe"));
      assert.deepEqual(args, ["-products", "*", "-version", "[17.0,18.0)", "-requires",
        "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-sort", "-property", "installationPath", "-utf8"]);
      assert.equal(options.env["ProgramFiles(x86)"], root);
      return { stdout: "", stderr: "" };
    };
    const toolchain = loadWithMocks("../out/src/ros/windows-toolchain", {
      child_process: { execFile },
      "./windows-batch": { sourceWindowsBatch: () => assert.fail("Never source or install real tools"), quoteBatchPath: value => value },
    });
    const env = { pAtH: path.join(root, "tools"), visualstudioversion: "17.0", vscmd_arg_tgt_arch: "x64",
      windowssdkdir: path.join(root, "SDK"), windowssdkversion: "10.0\\", include: "includes", lib: "libs",
      "ProgramFiles(x86)": root };
    assert.equal(await toolchain.hasWindowsToolchain(env), true);
    assert.equal(await toolchain.activateWindowsToolchain(env), env);
    const h = harness({ toolchain, hostEnv: env });
    const task = h.make();
    const first = await terminal(task);
    first.pty.open();
    assert.equal(await first.done, 0);
    assert.equal(discoveries, 0, "A ready inherited toolchain requires no discovery");
    assert.deepEqual(h.spawns[0].options.env, env);
    await fs.promises.unlink(path.join(root, removed));
    assert.equal(await toolchain.hasWindowsToolchain(env), false);
    await assert.rejects(toolchain.activateWindowsToolchain(env), /Windows SDK/);
    const second = await terminal(task);
    second.pty.open();
    assert.equal(await second.done, 1);
    assert.equal(discoveries, 2, "Both direct activation and the next build must rediscover tools");
    assert.equal(h.spawns.length, 1, "The stale environment must not reach another child");
    assert.equal(h.warnings.length, 1);
    await fs.promises.writeFile(path.join(root, removed), "");
    const third = await terminal(task);
    third.pty.open();
    assert.equal(await third.done, 0);
    assert.equal(h.spawns.length, 2, "Repair permits retry without recreating the task");
    assert.equal(discoveries, 2);
  });
}

test("PTY encodes both streams, normalizes output, forwards input and reports exactly one exit", async () => {
  const h = harness({ autoClose: false });
  const run = await terminal(h.make({ command: "C:\\custom tools\\colcon.exe" }));
  run.pty.open();
  const child = await h.spawned.promise;
  assert.equal(h.spawns[0].command, "C:\\custom tools\\colcon.exe");
  assert.equal(child.stdout.readableEncoding, "utf8");
  assert.equal(child.stderr.readableEncoding, "utf8");
  const encoded = Buffer.from("café\n");
  child.stdout.write(encoded.subarray(0, 4));
  child.stdout.write(encoded.subarray(4));
  child.stderr.write("warning\r\nnext\n");
  run.pty.handleInput("answer\r");
  assert.deepEqual(child.input, ["answer\n"]);
  child.emit("close", 7);
  assert.equal(await run.done, 7);
  assert.match(run.writes.join(""), /café\r\nwarning\r\nnext\r\n/);
  assert.match(run.writes.join(""), /exited with code 7/);
  const written = run.writes.length;
  child.stdout.write("late data");
  child.emit("error", new Error("late error"));
  run.pty.close();
  assert.equal(run.writes.length, written);
  assert.deepEqual(run.codes, [7]);
  assert.deepEqual(h.kills, []);
});

for (const mode of ["throw", "error", "null exit"]) {
  test(`PTY terminates a failed child once: ${mode}`, async () => {
    const h = harness({ autoClose: false, spawnError: mode === "throw" ? new Error("ENOENT fixture") : undefined });
    const run = await terminal(h.make());
    run.pty.open();
    if (mode !== "throw") {
      const child = await h.spawned.promise;
      if (mode === "error") { child.emit("error", new Error("ENOENT fixture")); }
      child.emit("close", null);
    }
    assert.equal(await run.done, 1);
    assert.deepEqual(run.codes, [1]);
    assert.match(run.writes.join(""), mode === "null exit" ? /exited with code 1/ : /ENOENT fixture/);
    assert.deepEqual(h.warnings, [], "Spawn failure is not an invitation to install a compiler");
  });
}

test("closing before open performs no activation or spawn", async () => {
  const h = harness();
  const run = await terminal(h.make());
  run.pty.close();
  run.pty.open();
  assert.equal(await run.done, 130);
  assert.deepEqual(h.activations, []);
  assert.deepEqual(h.spawns, []);
  assert.deepEqual(h.warnings, []);
});

for (const outcome of ["ready", "failure"]) {
  test(`cancelling pending preflight suppresses late ${outcome}, prompts and spawn`, async () => {
    const gate = deferred();
    const h = harness({ activate: () => gate.promise });
    const run = await terminal(h.make());
    run.pty.open();
    assert.equal(h.activations.length, 1);
    run.pty.close();
    assert.equal(await run.done, 130);
    if (outcome === "ready") { gate.resolve({ Path: "activated" }); }
    else { gate.reject(new Error("compiler disappeared")); }
    await flush();
    assert.deepEqual(h.spawns, []);
    assert.deepEqual(h.warnings, []);
    assert.deepEqual(h.preparations, []);
    assert.deepEqual(h.copied, []);
    assert.deepEqual(run.codes, [130]);
  });
}

test("cancelling while the repair dialog is open prevents a late copy action", async () => {
  const shown = deferred();
  const choice = deferred();
  const h = harness({
    activate: async () => { throw new Error("SDK missing"); },
    choice: () => { shown.resolve(); return choice.promise; },
  });
  const run = await terminal(h.make());
  run.pty.open();
  await shown.promise;
  run.pty.close();
  choice.resolve("Copy Install Command");
  await flush();
  assert.equal(await run.done, 130);
  assert.deepEqual(h.copied, []);
  assert.deepEqual(h.spawns, []);
});

for (const fallback of [false, true]) {
  test(`running cancellation kills the Windows process tree${fallback ? " with child.kill fallback" : " via Ctrl+C"}`, async () => {
    const h = harness({ autoClose: false, killError: fallback ? new Error("taskkill failed") : undefined });
    const run = await terminal(h.make());
    run.pty.open();
    const child = await h.spawned.promise;
    if (fallback) { run.pty.close(); }
    else { run.pty.handleInput("\x03"); }
    assert.equal(h.kills.length, 1);
    assert.equal(h.kills[0].command, path.join(h.process.env.SystemRoot || "C:\\Windows", "System32", "taskkill.exe"));
    assert.deepEqual(h.kills[0].args, ["/pid", "4242", "/T", "/F"]);
    assert.deepEqual(h.kills[0].options, { windowsHide: true });
    assert.equal(child.kills, fallback ? 1 : 0);
    assert.deepEqual(child.input, []);
    if (!fallback) { child.emit("close", 0); }
    assert.equal(await run.done, 130, "Even a racing successful exit must report cancellation");
    assert.deepEqual(run.codes, [130]);
  });
}

for (const platform of ["win32", "linux", "darwin"]) {
  test(`${platform}: only Windows builds use compiler preflight; Unix tasks defer environment resolution`, async () => {
    const h = harness({ platform });
    const definitions = [
      { type: "colcon", command: "colcon", args: ["test"] },
      { type: "colcon", command: "colcon", args: ["list"] },
      { type: "colcon", command: "colcon", args: ["build-helper"] },
      { type: "colcon", command: "colcon" },
      { type: "ROS2", command: "ros2", args: ["build"] },
    ];
    if (platform !== "win32") { definitions.push({ type: "colcon", command: "colcon", args: ["build"] }); }
    for (const definition of definitions) {
      const task = h.shell.make("unchanged", definition);
      if (platform === "win32") {
        assert.ok(task.execution instanceof h.vscode.ShellExecution);
        assert.equal(task.execution.command, definition.command);
        assert.deepEqual(task.execution.args, definition.args ?? []);
        assert.equal(task.execution.options.env, h.extension.env);
      } else {
        assert.ok(task.execution instanceof h.vscode.CustomExecution);
        await terminal(task);
      }
      assert.equal(task.definition.options, undefined);
    }
    if (platform !== "win32") {
      const tasks = await new h.colcon.ColconProvider().provideTasks();
      for (const task of tasks) {
        assert.ok(task.execution instanceof h.vscode.CustomExecution);
        assert.ok(task.definition.args.includes("--symlink-install"));
      }
      const packageTask = await h.colcon.makeColconPackageTask("camera");
      assert.ok(packageTask.execution instanceof h.vscode.CustomExecution);
      assert.ok(packageTask.definition.args.includes("--packages-select"));
      assert.ok(!packageTask.definition.args.includes("--packages-up-to"));
    }
    assert.deepEqual(h.activations, []);
    assert.deepEqual(h.warnings, []);
    assert.deepEqual(h.spawns, []);
    assert.equal(h.envReads, 0);
  });
}

for (const choice of ["Copy Install Command", "Cancel Build", undefined]) {
  test(`Test Explorer actual buildTestExecutable rejects before spawn: ${choice ?? "dismiss"}`, async () => {
    const h = harness({ choice, activate: async () => { throw new Error("missing SDK"); } });
    assert.equal(typeof h.runner.buildTestExecutable, "function");
    await assert.rejects(h.runner.buildTestExecutable("camera", false), /install or repair the Windows C\+\+ toolchain/);
    assert.equal(h.envReads, 0, "Windows must not wait for a failed startup activation");
    assert.deepEqual(h.activations[0].env, h.process.env);
    assert.equal(h.activations[0].env.STALE_RUNTIME_HOOK, undefined);
    assert.equal(h.activations[0].options.cwd, h.workspace);
    assert.equal(h.warnings.length, 1);
    assert.deepEqual(h.spawns, []);
    assert.deepEqual(h.preparations, []);
    assert.deepEqual(h.copied, choice === "Copy Install Command" ? [h.installCommand] : []);
    assert.match(h.logs.join("\n"), /missing SDK/);
  });
}

test("Test Explorer waits for activation and rereads the host baseline independently of extension.env", async () => {
  let gate = deferred();
  const h = harness({ activate: () => gate.promise });
  for (const debug of [false, true]) {
    h.process.env = { Path: `C:\\host ROS ${debug}`, ROS_DISTRO: "jazzy" };
    h.extension.env = { Path: `C:\\stale ROS ${debug}`, STALE_RUNTIME_HOOK: "old scalar" };
    gate = deferred();
    const building = h.runner.buildTestExecutable("camera & lidar", debug);
    await flush();
    assert.equal(h.activations.length, debug ? 2 : 1);
    assert.equal(h.spawns.length, debug ? 1 : 0, "No child before activation settles");
    assert.deepEqual(h.activations.at(-1).env, h.process.env);
    assert.equal(h.activations.at(-1).env.STALE_RUNTIME_HOOK, undefined);
    const activated = { ...h.process.env, VisualStudioVersion: "17.0", INCLUDE: "SDK", LIB: "SDK libs" };
    gate.resolve(activated);
    await building;
    const spawn = h.spawns.at(-1);
    assert.equal(spawn.command, "colcon");
    assert.deepEqual(spawn.args, ["build", "--merge-install", "--packages-up-to", "camera & lidar",
      "--event-handlers", "console_cohesion+", "--base-paths", h.workspace,
      "--cmake-args", `-DCMAKE_BUILD_TYPE=${debug ? "Debug" : "RelWithDebInfo"}`]);
    assert.equal(spawn.options.env, activated);
    assert.equal(h.preparations.at(-1), activated);
    assert.deepEqual(h.preparationOptions.at(-1), { cwd: h.workspace });
    assert.equal(spawn.options.cwd, h.workspace);
    assert.equal(spawn.options.stdio, "pipe");
  }
  assert.equal(h.envReads, 0);
  assert.deepEqual(h.events, ["compiler", "ros", "spawn", "compiler", "ros", "spawn"]);
  assert.deepEqual(h.warnings, []);
});

for (const mode of ["exit", "error", "throw"]) {
  test(`Test Explorer propagates build ${mode} failures after preflight`, async () => {
    const h = harness({ autoClose: false, spawnError: mode === "throw" ? new Error("spawn failed") : undefined });
    const building = h.runner.buildTestExecutable("camera", false);
    const rejected = assert.rejects(building, mode === "exit" ? /exit code 9[\s\S]*stdout detail[\s\S]*stderr detail/ : /spawn failed/);
    if (mode !== "throw") {
      const child = await h.spawned.promise;
      // Always deliver close, including after error, just as Node does, to release the build timer.
      child.stdout.write("stdout detail\n");
      child.stderr.write("stderr detail\n");
      if (mode === "error") { child.emit("error", new Error("spawn failed")); }
      child.emit("close", 9);
    }
    await rejected;
    assert.equal(h.activations.length, 1);
    assert.deepEqual(h.warnings, []);
  });
}

for (const platform of ["linux", "darwin"]) {
  test(`Test Explorer ${platform} build bypasses Windows preflight and retains its environment`, async () => {
    const h = harness({ platform, activate: () => assert.fail("Windows preflight on non-Windows") });
    await h.runner.buildTestExecutable("camera", false);
    assert.equal(h.spawns.length, 1);
    assert.equal(h.spawns[0].options.env, h.extension.env);
    assert.ok(h.spawns[0].args.includes("--symlink-install"));
    assert.equal(h.envReads, 1);
    assert.deepEqual(h.preparations, []);
    assert.deepEqual(h.activations, []);
    assert.deepEqual(h.warnings, []);
  });
}

test("Test Explorer without a workspace rejects before environment resolution or preflight", async () => {
  const h = harness();
  h.vscode.workspace.workspaceFolders = undefined;
  await assert.rejects(h.runner.buildTestExecutable("camera", false), /No workspace folder found/);
  assert.equal(h.envReads, 0);
  assert.deepEqual(h.activations, []);
  assert.deepEqual(h.spawns, []);
});

for (const platform of ["win32", "linux", "darwin"]) {
test(`${platform} discovery honors skip config without filesystem, ROS, compiler, or subprocess prerequisites`, async () => {
  const ignored = { camera: true, lidar: false, broken_package: true };
  const h = harness({ ignored, platform });
  h.extension.env = undefined;
  const tasks = await new h.colcon.ColconProvider().provideTasks();
  assert.equal(tasks.length, 4);
  assert.deepEqual(h.packageChecks, []);
  for (const task of tasks) {
    const args = task.definition.args;
    assert.deepEqual(args.slice(args.indexOf("--packages-skip"), args.indexOf("--cmake-args")),
      ["--packages-skip", "camera", "broken_package"]);
    assert.ok(!args.includes("--packages-select"));
    assert.ok(!args.includes("lidar"));
  }
  assert.deepEqual(ignored, { camera: true, lidar: false, broken_package: true });
  assert.deepEqual(h.packageLists, []);
  assert.deepEqual(h.events, []);
  assert.deepEqual(h.warnings, []);
  assert.deepEqual(h.errors, []);
  assert.deepEqual(h.spawns, []);
  assert.deepEqual(h.kills, []);
  assert.equal(h.envReads, 0);
});

test(`${platform} discovery offers builds in any open workspace without package.xml or an activated environment`, async () => {
  const h = harness({ platform });
  h.extension.env = undefined;
  const tasks = await new h.colcon.ColconProvider().provideTasks();
  assert.equal(tasks.length, 4);
  assert.equal(tasks.filter(task => task.group === h.vscode.TaskGroup.Build).length, 2);
  assert.deepEqual(h.packageChecks, []);
  assert.deepEqual(h.packageLists, []);
  assert.deepEqual(h.events, []);
  assert.deepEqual(h.spawns, []);
  assert.deepEqual(h.kills, []);
  assert.deepEqual(h.warnings, []);
  assert.deepEqual(h.errors, []);
  assert.equal(h.envReads, 0);
});
}

for (const rootPath of [undefined, ""]) {
  test(`Windows discovery returns no tasks only when rootPath is ${JSON.stringify(rootPath)}`, async () => {
    const h = harness();
    const provider = new h.colcon.ColconProvider();
    h.vscode.workspace.rootPath = rootPath;
    assert.deepEqual(await provider.provideTasks(), [], "workspaceFolders alone must not bypass the rootPath guard");
    h.vscode.workspace.rootPath = h.workspace;
    h.vscode.workspace.workspaceFolders = undefined;
    assert.equal((await provider.provideTasks()).length, 4, "A rootPath is sufficient without workspaceFolders");
    assert.deepEqual(h.packageChecks, []);
    assert.deepEqual(h.packageLists, []);
    assert.deepEqual(h.events, []);
    assert.deepEqual(h.spawns, []);
    assert.deepEqual(h.kills, []);
    assert.equal(h.envReads, 0);
  });
}

test("Windows skip configuration is reread rather than cached by a provider", async () => {
  const ignored = { camera: false };
  const h = harness({ ignored });
  const provider = new h.colcon.ColconProvider();
  assert.ok((await provider.provideTasks()).every(task => !task.definition.args.includes("--packages-skip")));
  ignored.camera = true;
  assert.ok((await provider.provideTasks()).every(task => task.definition.args.includes("--packages-skip")));
  assert.deepEqual(h.packageLists, []);
  assert.deepEqual(h.events, []);
});

for (const platform of ["win32", "linux", "darwin"]) {
  test(`${platform} generated package and Test Explorer builds reread ignore config without changing explicit selection`, async () => {
    const settings = { platform, ignored: { broken_package: true, camera: false } };
    const h = harness(settings);
    for (const [index, ignored] of [
      { broken_package: true, camera: false },
      { broken_package: false, camera: true, other_dependency: true },
      { camera: false, other_dependency: false },
    ].entries()) {
      settings.ignored = ignored;
      const snapshot = { ...ignored };
      const task = await h.colcon.makeColconPackageTask("camera");
      await h.runner.buildTestExecutable("camera", false);
      const skipArgs = platform === "win32"
        ? Object.keys(ignored).filter(name => ignored[name]) : [];
      for (const args of [task.definition.args, h.spawns[index].args]) {
        const selection = platform === "win32" ? "--packages-up-to" : "--packages-select";
        assert.equal(args[args.indexOf(selection) + 1], "camera");
        if (skipArgs.length) {
          assert.deepEqual(args.slice(args.indexOf("--packages-skip"), args.indexOf("--cmake-args")),
            ["--packages-skip", ...skipArgs]);
          assert.equal(args.filter(arg => arg === "--packages-skip").length, 1);
        } else {
          assert.ok(!args.includes("--packages-skip"));
        }
        assert.equal(args.at(-1), "-DCMAKE_BUILD_TYPE=RelWithDebInfo");
      }
      assert.deepEqual(ignored, snapshot, "Reading skip configuration must not mutate it");
    }
    assert.deepEqual(h.packageLists, [], "Selecting ignored packages must not need discovery");
  });
}

test("Windows configured ignores do not alter custom package selection or skip arguments", async () => {
  const h = harness({ ignored: { camera: true, unrelated: true } });
  for (const selection of ["--packages-select", "--packages-up-to"]) {
    const args = ["build", selection, "camera", "--packages-skip", "custom_skip", "--cmake-args", "-DCUSTOM=ON"];
    const task = new h.colcon.ColconProvider().resolveTask({ definition: {
      type: "colcon", command: "colcon", args,
    } });
    const run = await terminal(task);
    run.pty.open();
    assert.equal(await run.done, 0);
    assert.deepEqual(h.spawns.at(-1).args,
      ["build", selection, "camera", "--packages-skip", "custom_skip", "--cmake-args", "-DCUSTOM=ON"]);
  }
});

test("PTY waits for fresh ROS preparation and uses its result, not the compiler-only environment", async () => {
  const gate = deferred();
  const activated = { Path: "C:\\compiler", INCLUDE: "SDK" };
  const prepared = { ...activated, Path: "C:\\fresh ROS;C:\\compiler", ROS_VERSION: "2", ROS_DISTRO: "lyrical", OVERLAY: "ready" };
  const h = harness({ activate: async () => activated, prepareRos: () => gate.promise });
  const run = await terminal(h.make());
  run.pty.open();
  await flush();
  assert.deepEqual(h.events, ["compiler", "ros"]);
  assert.equal(h.preparations[0], activated);
  assert.deepEqual(h.spawns, []);
  gate.resolve(prepared);
  assert.equal(await run.done, 0);
  assert.equal(h.spawns[0].options.env, prepared);
  assert.match(run.writes.join(""), /Checking ROS underlays, ros2, and colcon \(without the workspace install overlay\)/);
});

test("PTY ROS preparation failure logs to the terminal and output and shows an error without a build spawn", async () => {
  const h = harness({ prepareRos: async () => { throw new Error("fixture overlay setup failed (code 23)"); } });
  const run = await terminal(h.make());
  run.pty.open();
  assert.equal(await run.done, 1);
  assert.deepEqual(h.events, ["compiler", "ros"]);
  assert.deepEqual(h.spawns, []);
  assert.deepEqual(h.kills, []);
  assert.deepEqual(h.warnings, [], "ROS failure must not offer compiler installation");
  assert.match(h.logs.join("\n"), /fixture overlay setup failed \(code 23\)/);
  assert.match(run.writes.join(""), /fixture overlay setup failed \(code 23\)/);
  assert.equal(h.errors.length, 1);
  assert.match(h.errors[0][0], /fixture overlay setup failed[\s\S]*Output > ROS 2[\s\S]*No build was started/);
});

for (const outcome of ["ready", "failure"]) {
  test(`cancelling ROS preparation suppresses late ${outcome}, error dialogs, and build spawn`, async () => {
    const gate = deferred();
    const h = harness({ prepareRos: () => gate.promise });
    const run = await terminal(h.make());
    run.pty.open();
    await flush();
    assert.equal(h.preparations.length, 1);
    run.pty.close();
    assert.equal(await run.done, 130);
    const writes = run.writes.length;
    if (outcome === "ready") { gate.resolve({ ROS_VERSION: "2", ROS_DISTRO: "lyrical" }); }
    else { gate.reject(new Error("late ROS failure")); }
    await flush();
    assert.deepEqual(run.codes, [130]);
    assert.equal(run.writes.length, writes);
    assert.deepEqual(h.spawns, []);
    assert.deepEqual(h.errors, []);
    assert.deepEqual(h.warnings, []);
  });
}

test("Test Explorer without an activated ROS environment still reaches compiler repair before sourcing ROS", async () => {
  const h = harness({ choice: "Copy Install Command", activate: async () => { throw new Error("fixture SDK missing"); } });
  h.extension.env = undefined;
  h.extension.resolvedEnv = () => assert.fail("Do not wait for an environment that failed startup");
  await assert.rejects(h.runner.buildTestExecutable("camera", false), /install or repair the Windows C\+\+ toolchain/);
  assert.deepEqual(h.activations[0].env, h.process.env);
  assert.deepEqual(h.events, ["compiler"]);
  assert.deepEqual(h.copied, [h.installCommand]);
  assert.deepEqual(h.spawns, []);
});

test("Test Explorer waits for fresh ROS preparation after compiler activation and spawns with its result", async () => {
  const gate = deferred();
  const activated = { INCLUDE: "SDK", VisualStudioVersion: "17.0" };
  const prepared = { ...activated, ROS_VERSION: "2", ROS_DISTRO: "lyrical", OVERLAY: "ready" };
  const h = harness({ activate: async () => activated, prepareRos: () => gate.promise });
  h.extension.env = undefined;
  h.extension.resolvedEnv = () => assert.fail("Windows builds must not wait for startup environment resolution");
  const building = h.runner.buildTestExecutable("camera", false);
  await flush();
  assert.deepEqual(h.activations[0].env, h.process.env);
  assert.deepEqual(h.events, ["compiler", "ros"]);
  assert.equal(h.preparations[0], activated);
  assert.deepEqual(h.spawns, []);
  gate.resolve(prepared);
  await building;
  assert.equal(h.spawns[0].options.env, prepared);
  assert.deepEqual(h.events, ["compiler", "ros", "spawn"]);
  assert.equal(h.envReads, 0);
});

test("Test Explorer propagates ROS preparation failures without spawning a build or offering compiler repair", async () => {
  const h = harness({ prepareRos: async () => { throw new Error("fixture ros2.exe --help failed"); } });
  await assert.rejects(h.runner.buildTestExecutable("camera", false), /fixture ros2.exe --help failed/);
  assert.deepEqual(h.events, ["compiler", "ros"]);
  assert.deepEqual(h.spawns, []);
  assert.deepEqual(h.warnings, []);
  assert.equal(h.envReads, 0);
});

for (const debug of [false, true]) {
  test(`Test Explorer runTest builds before preparing a distinct runtime environment for ${debug ? "debugging" : "execution"}`, async () => {
    const compilerGate = deferred();
    const rosGate = deferred();
    const runtimeGate = deferred();
    const activated = { Path: "C:\\compiler", INCLUDE: "SDK", VisualStudioVersion: "17.0" };
    const prepared = { ...activated, Path: "C:\\fresh ROS;C:\\compiler", ROS_VERSION: "2", ROS_DISTRO: "lyrical" };
    const runtime = { ...prepared, Path: "C:\\fresh workspace\\install\\bin;" + prepared.Path, RUNTIME_OVERLAY: "fresh" };
    const h = harness({ autoClose: false, activate: () => compilerGate.promise,
      prepareRos: () => rosGate.promise, prepareRuntime: () => runtimeGate.promise });
    h.extension.env = undefined;
    h.extension.resolvedEnv = () => {
      h.envReads++;
      assert.fail("runTest must not await an environment that failed startup, even after building");
    };
    const executable = path.join(h.workspace, "build", "camera", "camera_test.exe");
    const lookups = [];
    h.runner.findTestExecutable = (...args) => {
      lookups.push(args);
      return lookups.length === 1 ? undefined : executable;
    };

    const running = h.runner.runTest({
      type: "cpp_gtest", packageName: "camera", filePath: "camera_test.cpp",
      testClass: "Camera", testMethod: "CapturesFrame",
    }, debug);
    // Observe rejection immediately so a regression fails assertions, not as an unhandled rejection.
    running.catch(() => {});
    await flush();
    assert.deepEqual(h.events, ["compiler"]);
    assert.deepEqual(h.activations[0].env, h.process.env);
    assert.equal(h.activations[0].options.cwd, h.workspace);
    assert.equal(h.envReads, 0);
    assert.deepEqual(h.spawns, []);

    compilerGate.resolve(activated);
    await flush();
    assert.deepEqual(h.events, ["compiler", "ros"]);
    assert.equal(h.preparations[0], activated);
    assert.deepEqual(h.spawns, [], "Build and test must wait for fresh ROS preparation");
    rosGate.resolve(prepared);
    const buildChild = await h.spawned.promise;
    assert.deepEqual(h.runtimePreparations, [], "Runtime preparation must wait for successful build completion");
    assert.deepEqual(h.events, ["compiler", "ros", "spawn"]);
    buildChild.emit("close", 0);
    await flush();
    assert.deepEqual(h.events, ["compiler", "ros", "spawn", "runtime"]);
    assert.equal(h.runtimePreparations[0].env, prepared);
    assert.equal(h.runtimePreparations[0].workspace, h.workspace);
    assert.equal(h.spawns.length, 1, "No test process before runtime preparation completes");
    assert.deepEqual(h.debugSessions, [], "No debugger before runtime preparation completes");
    runtimeGate.resolve(runtime);
    await flush();
    if (!debug) { h.spawns[1].child.emit("close", 0); }
    await running;

    assert.deepEqual(lookups, [
      [h.workspace, "camera", "camera_test"], [h.workspace, "camera", "camera_test"],
    ]);
    assert.deepEqual(h.events, ["compiler", "ros", "spawn", "runtime", debug ? "debug" : "spawn"]);
    assert.equal(h.spawns.length, debug ? 1 : 2);
    const [build, execution] = h.spawns;
    assert.equal(build.command, "colcon");
    assert.deepEqual(build.args, ["build", "--merge-install", "--packages-up-to", "camera",
      "--event-handlers", "console_cohesion+", "--base-paths", h.workspace,
      "--cmake-args", `-DCMAKE_BUILD_TYPE=${debug ? "Debug" : "RelWithDebInfo"}`]);
    assert.equal(build.options.env, prepared);
    assert.equal(build.options.cwd, h.workspace);
    assert.notEqual(runtime, prepared);
    assert.equal(prepared.RUNTIME_OVERLAY, undefined, "Preparing runtime must not contaminate the build environment");
    const testArgs = ["--gtest_filter=Camera.CapturesFrame", "--gtest_output=xml", "--gtest_color=yes"];
    if (debug) {
      assert.equal(h.debugSessions.length, 1);
      const { folder, config } = h.debugSessions[0];
      assert.equal(folder, h.vscode.workspace.workspaceFolders[0]);
      assert.equal(config.type, "cppvsdbg");
      assert.equal(config.program, executable);
      assert.equal(config.cwd, h.workspace);
      assert.deepEqual(config.args, testArgs);
      assert.deepEqual(Object.fromEntries(config.environment.map(({ name, value }) => [name, value])), runtime);
    } else {
      assert.equal(execution.command, executable);
      assert.deepEqual(execution.args, testArgs);
      assert.equal(execution.options.env, runtime, "The test must use the post-build runtime environment");
      assert.equal(execution.options.cwd, h.workspace);
      assert.equal(execution.options.shell, false);
      assert.deepEqual(h.debugSessions, []);
    }
    assert.equal(h.envReads, 0);
    assert.equal(h.extension.env, undefined, "Recovery must not depend on startup environment being populated");
    assert.deepEqual(h.warnings, []);
  });
}

for (const debug of [false, true]) {
  for (const initiallyBuilt of [false, true]) {
    test(`Test Explorer consecutive ${debug ? "debug" : "run"} requests refresh existing binaries after undefined startup (${initiallyBuilt ? "already built" : "recovery build"})`, async () => {
      let configuration = "first setup";
      const h = harness({
        prepareRos: async env => ({ ...env, ROS_SETUP: configuration }),
        prepareRuntime: async env => ({ ...env, RUNTIME_OVERLAY: configuration }),
      });
      h.extension.env = undefined;
      h.extension.resolvedEnv = () => {
        h.envReads++;
        assert.fail("Existing binaries must not wait for undefined startup or reuse stale runtime state");
      };
      const executable = path.join(h.workspace, "build", "camera", "camera_test.exe");
      let built = initiallyBuilt;
      h.runner.findTestExecutable = () => {
        if (!built) { built = true; return undefined; }
        return executable;
      };
      for (const index of [0, 1]) {
        configuration = `setup ${index}`;
        h.process.env = { Path: `C:\\host${index}`, HOST_VALUE: String(index) };
        // Recovery stays undefined across both runs; pre-existing binaries also cover stale startup state.
        if (initiallyBuilt && index === 1) { h.extension.env = { STALE_RUNTIME_HOOK: "stale startup" }; }
        const startupEnv = h.extension.env;
        const startupSnapshot = startupEnv && { ...startupEnv };
        const hostSnapshot = { ...h.process.env };
        await h.runner.runTest({
          type: "cpp_gtest", packageName: "camera", filePath: "camera_test.cpp",
          testClass: "Camera", testMethod: "CapturesFrame",
        }, debug);
        const expected = { ...hostSnapshot, ROS_SETUP: configuration, RUNTIME_OVERLAY: configuration };
        const runtime = debug
          ? Object.fromEntries(h.debugSessions[index].config.environment.map(({ name, value }) => [name, value]))
          : h.spawns.at(-1).options.env;
        assert.deepEqual(runtime, expected, "Each request must use fresh underlays and runtime overlay");
        assert.deepEqual(h.activations[index].env, hostSnapshot);
        assert.deepEqual(h.preparationOptions[index], { cwd: h.workspace });
        assert.deepEqual(h.runtimePreparations[index], {
          env: { ...hostSnapshot, ROS_SETUP: configuration }, workspace: h.workspace,
        });
        assert.equal(h.extension.env, startupEnv, "Never publish a build-only or test-local environment");
        assert.deepEqual(h.extension.env, startupSnapshot);
        assert.deepEqual(h.process.env, hostSnapshot);
        assert.equal(h.spawns.filter(spawn => spawn.command === "colcon").length, initiallyBuilt ? 0 : 1,
          "Existing binaries must never be rebuilt");
      }
      assert.equal(h.envReads, 0);
      assert.equal(h.activations.length, 2);
      assert.equal(h.runtimePreparations.length, 2);
      assert.deepEqual(h.events, [
        "compiler", "ros", ...(initiallyBuilt ? [] : ["spawn"]), "runtime", debug ? "debug" : "spawn",
        "compiler", "ros", "runtime", debug ? "debug" : "spawn",
      ]);
      assert.deepEqual(h.warnings, []);
    });
  }
}

for (const [debug, failure] of [false, true].flatMap(debug =>
  ["compiler", "ros", "runtime"].map(failure => [debug, failure]))) {
  test(`Test Explorer existing executable stops on ${failure} failure before ${debug ? "debugging" : "execution"}`, async () => {
    const h = harness({
      activate: async env => {
        if (failure === "compiler") { throw new Error("fixture missing SDK"); }
        return env;
      },
      prepareRos: async env => {
        if (failure === "ros") { throw new Error("fixture underlay failed"); }
        return env;
      },
      prepareRuntime: async () => { throw new Error("fixture runtime failed"); },
    });
    h.extension.env = undefined;
    h.extension.resolvedEnv = () => assert.fail("Existing executables must bypass startup resolution");
    h.runner.findTestExecutable = () => path.join(h.workspace, "build", "camera", "camera_test.exe");
    await assert.rejects(h.runner.runTest({
      type: "cpp_gtest", packageName: "camera", filePath: "camera_test.cpp",
    }, debug), {
      compiler: /install or repair the Windows C\+\+ toolchain/,
      ros: /fixture underlay failed/,
      runtime: /fixture runtime failed/,
    }[failure]);
    assert.deepEqual(h.events, {
      compiler: ["compiler"], ros: ["compiler", "ros"], runtime: ["compiler", "ros", "runtime"],
    }[failure]);
    assert.deepEqual(h.spawns, [], "Neither a rebuild nor a test may start");
    assert.deepEqual(h.debugSessions, []);
    assert.equal(h.extension.env, undefined);
  });
}

for (const [debug, failure] of [false, true].flatMap(debug =>
  ["compiler", "ros", "build", "runtime"].map(failure => [debug, failure]))) {
  test(`Test Explorer runTest rejects ${failure} failure without startup resolution or test ${debug ? "debugging" : "spawn"}`, async () => {
    const h = harness({
      autoClose: failure !== "build",
      activate: async env => {
        if (failure === "compiler") { throw new Error("fixture SDK missing"); }
        return env;
      },
      prepareRos: async env => {
        if (failure === "ros") { throw new Error("fixture ROS preparation failed"); }
        return env;
      },
      prepareRuntime: async () => { throw new Error("fixture post-build runtime preparation failed"); },
    });
    h.extension.env = undefined;
    h.extension.resolvedEnv = () => {
      h.envReads++;
      assert.fail("runTest must reach build preflight without waiting for failed startup");
    };
    const lookups = [];
    h.runner.findTestExecutable = (...args) => { lookups.push(args); return undefined; };

    const rejected = assert.rejects(h.runner.runTest({
      type: "cpp_gtest", packageName: "camera", filePath: "camera_test.cpp",
    }, debug), {
      compiler: /Failed to build C\+\+ test package camera:.*install or repair the Windows C\+\+ toolchain/,
      ros: /Failed to build C\+\+ test package camera: fixture ROS preparation failed/,
      build: /Failed to build C\+\+ test package camera:.*exit code 9/,
      runtime: /Failed to build C\+\+ test package camera: fixture post-build runtime preparation failed/,
    }[failure]);
    if (failure === "build") { (await h.spawned.promise).emit("close", 9); }
    await rejected;
    assert.deepEqual(lookups, [[h.workspace, "camera", "camera_test"]]);
    assert.deepEqual(h.events, {
      compiler: ["compiler"], ros: ["compiler", "ros"],
      build: ["compiler", "ros", "spawn"], runtime: ["compiler", "ros", "spawn", "runtime"],
    }[failure]);
    assert.deepEqual(h.activations[0].env, h.process.env);
    assert.equal(h.preparations.length, failure === "compiler" ? 0 : 1);
    assert.equal(h.runtimePreparations.length, failure === "runtime" ? 1 : 0);
    assert.equal(h.warnings.length, failure === "compiler" ? 1 : 0);
    assert.equal(h.envReads, 0);
    assert.equal(h.spawns.length, ["build", "runtime"].includes(failure) ? 1 : 0);
    assert.ok(h.spawns.every(spawn => spawn.command === "colcon"), "Only the build may spawn; never the test");
    assert.deepEqual(h.debugSessions, []);
    assert.deepEqual(h.kills, []);
  });
}