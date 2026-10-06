const assert = require("node:assert/strict");
const fs = require("node:fs");
const Module = require("node:module");
const path = require("node:path");
const { test } = require("node:test");
const { promisify } = require("node:util");

function load(filename, mocks) {
  const resolved = require.resolve(filename);
  const original = Module._load;
  delete require.cache[resolved];
  Module._load = function(request, parent, isMain) {
    if (parent?.filename === resolved && Object.hasOwn(mocks, request)) { return mocks[request]; }
    return original.call(this, request, parent, isMain);
  };
  try { return require(resolved); }
  finally { Module._load = original; delete require.cache[resolved]; }
}

function resolver(files, platform = "win32") {
  const seen = [];
  const normalize = file => platform === "win32" ? path.win32.normalize(file).toLowerCase() : file;
  const existing = new Set(files.map(normalize));
  const { resolveRosPython } = load("../out/src/ros/python", {
    fs: { constants: fs.constants, promises: {
      stat: async file => { seen.push(file); return { isFile: () => existing.has(normalize(file)) }; },
      access: async () => {},
    } },
  });
  return { seen, resolve: env => resolveRosPython(env, platform) };
}

const prefix = "C:\\ROS user's & project!\\.pixi\\envs\\lyrical";
const python = path.win32.join(prefix, "python.exe");

test("Pixi Python wins over host/Store python3 and an unrelated virtualenv", async () => {
  const r = resolver([python, "C:\\host\\python3.exe"]);
  const env = { conda_prefix: prefix, VIRTUAL_ENV: "C:\\editor venv", Path: "C:\\host;C:\\Users\\me\\WindowsApps" };
  const before = { ...env };
  assert.equal(await r.resolve(env), python);
  assert.deepEqual(env, before);
  assert.deepEqual(r.seen, [python]);
});

test("a missing selected Pixi interpreter cannot fall back to system Python", async () => {
  const r = resolver(["C:\\host\\python.exe"]);
  await assert.rejects(r.resolve({ CONDA_PREFIX: prefix, PATH: "C:\\host" }), /refusing to fall back/);
  assert.deepEqual(r.seen, [python]);
});

test("Windows searches directory-first for python.exe and skips Store aliases and relative paths", async () => {
  const store = "C:\\Users\\me\\AppData\\Local\\Microsoft\\WindowsApps";
  const r = resolver([python, `${store}\\python.exe`, "C:\\host\\python3.exe"]);
  assert.equal(await r.resolve({ pAtH: `.;relative;${store};"${prefix}";C:\\host` }), python);
  assert.deepEqual(r.seen, [python]);
  await assert.rejects(r.resolve({ Path: `${store};.` }), /No Python interpreter/);
  await assert.rejects(r.resolve({}), /No Python interpreter/);
});

test("native Windows python3.exe remains supported when no python.exe exists", async () => {
  const r = resolver(["C:\\native\\python3.exe"]);
  assert.equal(await r.resolve({ PATH: "C:\\native" }), "C:\\native\\python3.exe");
});

for (const platform of ["linux", "darwin"]) {
  test(`${platform} uses activated Pixi or native Python without consulting host PATH`, async () => {
    const r = resolver(["/pixi/bin/python", "/usr/bin/python3"], platform);
    assert.equal(await r.resolve({ CONDA_PREFIX: "/pixi", PATH: "/usr/bin" }), "/pixi/bin/python");
    assert.equal(await r.resolve({ PATH: "/usr/bin" }), "/usr/bin/python3");
    await assert.rejects(r.resolve({ PATH: "." }), /No Python interpreter/);
  });
}

test("virtual environments use their own platform-specific interpreter", async () => {
  const win = resolver(["C:\\venv\\Scripts\\python.exe"]);
  assert.equal(await win.resolve({ VIRTUAL_ENV: "C:\\venv" }), "C:\\venv\\Scripts\\python.exe");
  const unix = resolver(["/venv/bin/python"], "linux");
  assert.equal(await unix.resolve({ VIRTUAL_ENV: "/venv" }), "/venv/bin/python");
});

function launchHarness() {
  const env = { CONDA_PREFIX: prefix, Path: prefix, ROS_DISTRO: "lyrical", PYTHONPATH: "ROS packages",
    LD_DEBUG: "libs", LD_DEBUG_OUTPUT: "debug.log", SECRET: "must-not-log" };
  const logs = [], calls = [], debug = [];
  const r = resolver([python]);
  const execFile = () => assert.fail("Use promisified execFile");
  execFile[promisify.custom] = async (command, args, options) => {
    calls.push({ command, args, options });
    return { stdout: JSON.stringify({ processes: [], lifecycle_nodes: [] }), stderr: "" };
  };
  const extension = { env, extPath: "C:\\Extension path & tools", resolvedEnv: async () => env,
    outputChannel: { appendLine: text => logs.push(text) } };
  const vscode = { workspace: { getConfiguration: () => ({ get: () => false }) },
    debug: { startDebugging: async (_folder, config) => { debug.push(config); return true; } } };
  const mocks = {
    vscode,
    child_process: { execFile, exec: () => assert.fail("Launch dumper must not use a shell") },
    "fs/promises": { access: async () => {} },
    "../../../../extension": extension,
    "../../extension": extension,
    "../../../../vscode-utils": { showOutputPanel() {} },
    "../../../utils": { mergeEnvFile: env => ({ FROM_ENV_FILE: "retained", ...env }) },
    "../../../../ros/ros": { rosApi: { getCoreStatus: async () => true } },
    "../../../../ros/ros2/lifecycle": {},
    "../../../../ros/python": { resolveRosPython: r.resolve },
    "../python": { resolveRosPython: r.resolve },
  };
  return { env, logs, calls, debug, extension, mocks };
}

test("launch debugger invokes Pixi Python with separate paths/arguments and retains ROS environment", async () => {
  const h = launchHarness();
  const { LaunchResolver } = load("../out/src/debugger/configuration/resolvers/ros2/launch", h.mocks);
  const args = ["camera_name:=camera & lidar", "config_file:=C:\\config files\\camera.yaml", "literal:=%PATH%"];
  const target = path.resolve("launch user's & file!.py");
  await new LaunchResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined,
    { target, arguments: args, env: { CUSTOM: "yes" } });
  assert.equal(h.calls.length, 1);
  const call = h.calls[0];
  assert.equal(call.command, python);
  assert.deepEqual(call.args, [path.resolve(h.extension.extPath, "assets/scripts/ros2_launch_dumper.py"), target, ...args]);
  assert.equal(call.options.env.PYTHONPATH, h.env.PYTHONPATH);
  assert.equal(call.options.env.CUSTOM, "yes");
  assert.equal(call.options.env.FROM_ENV_FILE, "retained");
  assert.equal(call.options.env.LD_DEBUG, undefined);
  assert.equal(call.options.env.LD_DEBUG_OUTPUT, undefined);
  assert.equal(h.env.LD_DEBUG, "libs");
  assert.equal(call.options.shell, undefined);
  assert.ok(h.logs.some(line => line.includes(python)));
});

test("launch tree uses the same interpreter and preserves its timeout/buffer limits", async () => {
  const h = launchHarness();
  const { LaunchFileParser } = load("../out/src/ros/launch-tree/launch-parser", h.mocks);
  const parser = new LaunchFileParser(h.extension.outputChannel, h.extension.extPath);
  const target = "C:\\launch path & file!.py";
  assert.deepEqual((await parser.parseLaunchFile(target)).errors, []);
  assert.equal(h.calls[0].command, python);
  assert.deepEqual(h.calls[0].args, [path.join(h.extension.extPath, "assets/scripts/ros2_launch_dumper.py"), target]);
  assert.equal(h.calls[0].options.env, h.env);
  assert.equal(h.calls[0].options.timeout, 30000);
  assert.equal(h.calls[0].options.maxBuffer, 10 * 1024 * 1024);
  await parser.parseLaunchFile(target);
  assert.equal(h.calls.length, 1, "Successful parsing is still cached");
});

test("debug_launch pins the Python adapter interpreter and never logs environment secrets", async () => {
  const h = launchHarness();
  const { LaunchResolver } = load("../out/src/debugger/configuration/resolvers/ros2/debug_launch", h.mocks);
  await new LaunchResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined,
    { target: path.resolve("launch.py"), arguments: ["name:=a b"] });
  assert.equal(h.debug[0].python, python);
  assert.equal(h.debug[0].env.CONDA_PREFIX, prefix);
  assert.doesNotMatch(h.logs.join("\n"), /must-not-log/);
});

test("Python nodes launched by the resolver use the ROS interpreter too", { skip: process.platform !== "win32" }, async () => {
  const h = launchHarness();
  const { LaunchResolver } = load("../out/src/debugger/configuration/resolvers/ros2/launch", h.mocks);
  await new LaunchResolver().executeLaunchRequest({ executable: "C:\\workspace\\talker.py", nodeName: "talker",
    arguments: [], env: h.env }, false);
  assert.equal(h.debug[0].python, python);
  assert.equal(h.debug[0].program, "C:\\workspace\\talker.py");
});

test("invalid Pixi Python stops launch rather than executing a fallback", async () => {
  const h = launchHarness();
  h.env.CONDA_PREFIX = "C:\\missing environment";
  const { LaunchResolver } = load("../out/src/debugger/configuration/resolvers/ros2/launch", h.mocks);
  await assert.rejects(new LaunchResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined,
    { target: path.resolve("launch.py") }), /refusing to fall back/);
  assert.deepEqual(h.calls, []);
  assert.deepEqual(h.debug, []);
});