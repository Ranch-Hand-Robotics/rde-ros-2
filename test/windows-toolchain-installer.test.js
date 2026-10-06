const assert = require("node:assert/strict");
const { EventEmitter } = require("node:events");
const fs = require("node:fs");
const Module = require("node:module");
const os = require("node:os");
const path = require("node:path");
const { test } = require("node:test");
const ts = require("typescript");

const root = path.resolve(__dirname, "..");
const installSource = path.join(root, "src/ros/installer/install-ros.ts");
const diagnosticsSource = path.join(root, "src/ros/installer/install-diagnostics.ts");
const buildToolsCommand = "winget install --id Microsoft.VisualStudio.2022.BuildTools --exact --override 'test-only-command'";
const compiled = new Map();

// Transpile only these source files in memory: no project build or out/ writes.
// Like windows-environment.test.js, intercept imports only while loading the module.
function loadWithMocks(filename, mocks) {
  if (!compiled.has(filename)) {
    const result = ts.transpileModule(fs.readFileSync(filename, "utf8"), {
      fileName: filename,
      compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
      reportDiagnostics: true,
    });
    assert.deepEqual(result.diagnostics, [], "Source must transpile without syntax errors");
    compiled.set(filename, result.outputText);
  }
  const loaded = new Module(filename, module);
  loaded.filename = filename;
  loaded.paths = Module._nodeModulePaths(path.dirname(filename));
  const original = Module._load;
  Module._load = function(request, parent, isMain) {
    return Object.hasOwn(mocks, request) ? mocks[request] : original.call(this, request, parent, isMain);
  };
  try {
    loaded._compile(compiled.get(filename), filename);
    return loaded.exports;
  } finally {
    Module._load = original;
  }
}

const diagnosticModule = loadWithMocks(diagnosticsSource, {});

async function fixture(t, options = {}) {
  const directory = await fs.promises.mkdtemp(path.join(os.tmpdir(), "rde-toolchain-installer-"));
  t.after(() => fs.promises.rm(directory, { recursive: true, force: true }));
  const activated = { Path: "C:\\compiler;C:\\Windows\\System32", INCLUDE: "sdk-include", LIB: "sdk-lib" };
  const events = [];
  const prompts = [];
  const clipboard = [];
  const errors = [];
  const tasks = [];
  const solverCalls = [];
  const activationCalls = [];
  const target = { kind: "pixi", distro: "jazzy", workspace: path.join(directory, "jazzy"),
    pixiExecutable: path.join(directory, "custom pixi", "pixi.exe") };
  const diagnostics = await diagnosticModule.InstallDiagnostics.create(directory, target, "install");
  diagnostics.report.preflight = { ready: true, checks: [] };
  let reportPath;
  let processEnded;
  let endDisposed = 0;
  const vscode = {
    workspace: { isTrusted: true },
    ConfigurationTarget: { Global: 1, Workspace: 2 },
    TaskScope: { Global: 1 },
    TaskRevealKind: { Always: 1 },
    TaskPanelKind: { New: 1 },
    ProgressLocation: { Notification: 1 },
    env: { clipboard: { writeText: async text => { clipboard.push(text); } } },
    window: {
      showWarningMessage: async (...args) => { prompts.push(args); return options.choice; },
      showErrorMessage: async message => { errors.push(message); },
      showInformationMessage: async () => {},
      showQuickPick: async () => ({ distro: installer.ROS2_DISTROS.find(distro => distro.name === "jazzy") }),
      withProgress: async (_options, callback) => callback(),
    },
    ShellExecution: class {
      constructor(command, taskOptions) { this.command = command; this.options = taskOptions; }
    },
    Task: class {
      constructor(definition, scope, name, source, execution) { Object.assign(this, { definition, scope, name, source, execution }); }
    },
    tasks: {
      onDidEndTaskProcess: callback => {
        processEnded = callback;
        return { dispose: () => { processEnded = undefined; } };
      },
      onDidEndTask: () => ({ dispose: () => { endDisposed++; } }),
      executeTask: async task => {
        events.push("task");
        tasks.push(task);
        assert.ok(processEnded, "Subscribe before launching the mocked task");
        const execution = { task };
        processEnded({ execution, exitCode: 0 });
        return execution;
      },
    },
  };
  class MockWorker extends EventEmitter {
    constructor() { super(); events.push("worker"); }
    postMessage(request) {
      events.push(request.type);
      assert.equal(request.type, "check_pixi", "No automatic package or Build Tools installation is allowed in tests");
      queueMicrotask(() => this.emit("message", { type: "pixi_available", executable: target.pixiExecutable }));
    }
    terminate() { events.push("worker-terminated"); }
  }
  const installer = loadWithMocks(installSource, {
    vscode,
    worker_threads: { Worker: MockWorker },
    "../../vscode-utils": { getExtensionConfiguration: () => ({ update: async () => {} }) },
    "../../extension": {
      extPath: root,
      outputChannel: { appendLine: () => {}, show: () => {} },
      extensionContext: {
        globalStorageUri: { fsPath: directory },
        globalState: { update: async (_key, value) => { reportPath = value; } },
      },
      activateEnvironment: async () => {},
    },
    "./pixi": { pixiPlatform: () => "win-64" },
    "./install-diagnostics": diagnosticModule,
    "./pixi-location": { selectPixiInstallRoot: async () => directory, cachePixiInstallRoot: async () => {} },
    "../windows-toolchain": {
      WINDOWS_BUILD_TOOLS_COMMAND: buildToolsCommand,
      activateWindowsToolchain: async (env, activationOptions) => {
        events.push("compiler");
        activationCalls.push({ env, options: activationOptions });
        if (options.activationError) { throw options.activationError; }
        return activated;
      },
    },
    "./install-preflight": {
      preflightInstallation: async () => ({ ready: true, checks: [] }),
      runPreflightCommand: async (command, args, timeout) => {
        solverCalls.push({ command, args, timeout, legacy: true });
        return { stdout: "", stderr: "legacy mocked failure", exitCode: 1 };
      },
    },
    "./health-check": {
      runHealthProcess: async (command, timeout) => {
        events.push("solver");
        solverCalls.push({ ...command, timeout });
        if (options.solverFailure) {
          return { stdout: "", stderr: "MSVC solver failure", exitCode: 1 };
        }
        if (!options.missingLockfile) {
          await fs.promises.writeFile(path.join(command.cwd, "pixi.lock"), "mock solved lockfile");
        }
        return { stdout: "solved", stderr: "", exitCode: 0 };
      },
      validateInstallation: async healthTarget => {
        events.push("health");
        return { target: healthTarget, healthy: true, checks: [] };
      },
    },
  });
  return { installer, diagnostics, activated, events, prompts, clipboard, errors, tasks, solverCalls, activationCalls,
    target, directory, readReport: () => fs.promises.readFile(reportPath, "utf8").then(JSON.parse),
    endDisposed: () => endDisposed };
}

test("working compiler environment is returned unchanged without prompting or installation", async t => {
  const f = await fixture(t);
  const inherited = { TEST_INPUT: "preserved" };
  assert.equal(await f.installer.ensureWindowsBuildTools(f.diagnostics, inherited), f.activated);
  assert.equal(f.activationCalls[0].env, inherited);
  assert.equal(f.activationCalls[0].options.cwd, f.diagnostics.directory);
  assert.equal(typeof f.activationCalls[0].options.onOutput, "function");
  assert.deepEqual(f.prompts, []);
  assert.deepEqual(f.clipboard, []);
  assert.deepEqual(f.events, ["compiler"]);
  assert.equal(f.diagnostics.report.preflight.checks.at(-1).status, "passed");
  assert.match(await f.diagnostics.readLog(), /RDE_STEP_OK:windows-compiler/);
});

for (const choice of [undefined, "Cancel", "Copy Install Command"]) {
  test(`missing or incomplete compiler blocks installation; choice=${choice}`, async t => {
    const f = await fixture(t, { choice, activationError: new Error("Existing Build Tools has no working Windows SDK") });
    await assert.rejects(f.installer.ensureWindowsBuildTools(f.diagnostics), /Administrator PowerShell.*retry/s);
    assert.equal(f.activationCalls[0].env, process.env);
    assert.equal(f.prompts.length, 1);
    const [message, modal, ...actions] = f.prompts[0];
    assert.deepEqual(modal, { modal: true });
    assert.deepEqual(actions, ["Copy Install Command", "Cancel"]);
    for (const expected of [/Existing Build Tools/, /administrator privileges/, /large download/, /MSVC/, /Windows SDK/,
      /agreements/, /consent/, /Visual Studio Installer > Modify > Desktop development with C\+\+/, /winget install will not add/]) {
      assert.match(message, expected);
    }
    assert.deepEqual(f.clipboard, choice === "Copy Install Command" ? [buildToolsCommand] : []);
    assert.deepEqual(f.events, ["compiler"], "No worker, solver, task or package installation should start");
    assert.equal(fs.existsSync(f.target.workspace), false);
    const report = JSON.parse(await fs.promises.readFile(f.diagnostics.reportPath, "utf8"));
    assert.equal(report.status, "blocked");
    assert.equal(report.preflight.ready, false);
    assert.equal(report.preflight.checks.at(-1).id, "windows-compiler");
    assert.match(await f.diagnostics.readLog(), /RDE_STEP_FAILED:windows-compiler:1/);
  });
}

test("Pixi solver uses captured compiler env and the detected executable, not PATH pixi", async t => {
  const f = await fixture(t);
  const staged = await f.installer.preflightPixiEnvironment({ name: "jazzy" }, f.diagnostics, f.activated);
  assert.deepEqual(f.solverCalls, [{ command: f.target.pixiExecutable,
    args: ["lock", "--manifest-path", staged], cwd: path.dirname(staged), env: f.activated, timeout: 120000 }]);
  assert.equal(f.solverCalls[0].env, f.activated);
  assert.equal(f.diagnostics.report.preflight.ready, true);
  assert.equal(fs.existsSync(f.target.workspace), false);
});

test("preflight without env retains the original mocked command path", async t => {
  const f = await fixture(t);
  await assert.rejects(f.installer.preflightPixiEnvironment({ name: "jazzy" }, f.diagnostics), /dependency preflight failed/);
  assert.equal(f.solverCalls[0].legacy, true);
  assert.equal(f.solverCalls[0].command, "pixi");
  assert.equal(f.solverCalls[0].timeout, 120000);
});

for (const mode of ["solverFailure", "missingLockfile", "missingExecutable"]) {
  test(`env-aware solver blocks on ${mode} without target creation`, async t => {
    const f = await fixture(t, { [mode]: true });
    if (mode === "missingExecutable") { delete f.target.pixiExecutable; }
    await assert.rejects(f.installer.preflightPixiEnvironment({ name: "jazzy" }, f.diagnostics, f.activated), /dependency preflight failed/);
    assert.equal(f.diagnostics.report.status, "blocked");
    assert.equal(f.diagnostics.report.preflight.ready, false);
    assert.equal(fs.existsSync(f.target.workspace), false);
    if (mode === "missingExecutable") { assert.deepEqual(f.solverCalls, []); }
  });
}

test("Windows install task receives captured env and keeps the non-elevated PowerShell boundary", async t => {
  const f = await fixture(t);
  assert.equal(await f.installer.runInstallTerminal({ name: "jazzy", displayName: "Jazzy" }, f.diagnostics,
    "exit 0\r\n", true, f.activated), 0);
  const execution = f.tasks[0].execution;
  assert.equal(execution.options.env, f.activated);
  assert.equal(execution.options.executable, "powershell.exe");
  assert.deepEqual(execution.options.shellArgs, ["-NoProfile", "-ExecutionPolicy", "Bypass", "-Command"]);
  assert.doesNotMatch(execution.command, /winget|RunAs|Start-Process/i);
  assert.equal(f.endDisposed(), 1);
  assert.equal(fs.existsSync(path.join(f.diagnostics.directory, "install.ps1")), false);
});

test("Windows orchestration activates before Pixi and reuses the same env for lock and install", {
  skip: process.platform !== "win32",
}, async t => {
  const f = await fixture(t);
  await f.installer.installRos();
  assert.deepEqual(f.errors, []);
  assert.deepEqual(f.prompts, []);
  assert.deepEqual(f.events, ["compiler", "worker", "check_pixi", "solver", "task", "worker-terminated", "health"]);
  assert.equal(f.solverCalls[0].env, f.activated);
  assert.equal(f.tasks[0].execution.options.env, f.activated);
  assert.equal((await f.readReport()).status, "passed");
});

test("Windows orchestration cannot start Pixi or create a target after copying a Build Tools command", {
  skip: process.platform !== "win32",
}, async t => {
  const f = await fixture(t, { choice: "Copy Install Command", activationError: new Error("No compiler") });
  await f.installer.installRos();
  assert.deepEqual(f.events, ["compiler"]);
  assert.deepEqual(f.clipboard, [buildToolsCommand]);
  assert.equal(fs.existsSync(f.target.workspace), false);
  const report = await f.readReport();
  assert.equal(report.status, "blocked");
  assert.equal(report.steps.find(step => step.id === "windows-compiler").status, "failed");
  assert.match(f.errors[0], /Install command copied.*Administrator PowerShell.*retry/s);
});