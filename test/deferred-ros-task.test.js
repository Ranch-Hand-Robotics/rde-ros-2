const assert = require("node:assert/strict");
const { EventEmitter } = require("node:events");
const fs = require("node:fs");
const Module = require("node:module");
const path = require("node:path");
const { PassThrough } = require("node:stream");
const { test } = require("node:test");
const { parse } = require("shell-quote");

// Each compiled consumer gets a private process and require. Never modify the
// host platform/environment or spawn Unix executables on a Windows test host.
function loadWithMocks(filename, mocks, testProcess) {
  const resolved = require.resolve(filename);
  const loaded = new Module(resolved, module);
  loaded.filename = resolved;
  loaded.paths = Module._nodeModulePaths(path.dirname(resolved));
  loaded.testProcess = testProcess;
  loaded.require = request => Object.hasOwn(mocks, request)
    ? mocks[request] : Module.prototype.require.call(loaded, request);
  loaded._compile("const process = module.testProcess;\n" + fs.readFileSync(resolved, "utf8"), resolved);
  return loaded.exports;
}

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

class FakeEventEmitter {
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

function harness(platform, settings = {}) {
  const h = { reads: 0, spawns: [], kills: [], windows: [], registrations: [] };
  h.process = {
    platform, env: { PATH: "/host/bin", HOST_ONLY: "must not leak into ROS" },
    kill: (pid, signal) => {
      h.kills.push({ pid, signal });
      if (settings.killError) { throw settings.killError; }
      return true;
    },
  };
  h.env = undefined;
  h.extension = {
    get env() {
      assert.equal(platform, "win32", "Unix discovery must never read extension.env");
      return h.env;
    },
    resolvedEnv: () => { h.reads++; return settings.getEnv ? settings.getEnv() : Promise.resolve(h.env); },
    prepareRosBuildEnvironment: () => assert.fail("Unix execution must not use Windows recovery"),
  };
  h.vscode = {
    EventEmitter: FakeEventEmitter,
    CustomExecution: class { constructor(callback) { this.callback = callback; } },
    ShellExecution: class {
      constructor(command, args, options) { Object.assign(this, { command, args, options }); }
    },
    Task: class {
      constructor(definition, scope, name, source) { Object.assign(this, { definition, scope, name, source }); }
    },
    TaskScope: { Global: 1, Workspace: 2 },
    TaskGroup: { Build: { id: "build" }, Test: { id: "test" } },
    workspace: {
      rootPath: "/first workspace", workspaceFolders: [{ uri: { fsPath: "/first workspace" } }],
      getConfiguration: (section, uri) => {
        assert.equal(section, "tasks");
        return settings.config ? settings.config(uri) : { inspect: () => undefined, get: () => undefined };
      },
    },
    tasks: { registerTaskProvider: (type, provider) => {
      h.registrations.push({ type, provider });
      return { dispose() {} };
    } },
  };
  const cp = {
    spawn: (command, args, options) => {
      if (settings.spawnError) { throw settings.spawnError; }
      const child = new EventEmitter();
      child.pid = 4242;
      child.stdout = new PassThrough();
      child.stderr = new PassThrough();
      child.stdin = new PassThrough();
      child.input = [];
      child.stdin.on("data", data => child.input.push(data.toString()));
      child.kills = [];
      child.kill = signal => { child.kills.push(signal); return true; };
      h.spawns.push({ command, args, options, child });
      return child;
    },
  };
  const execution = loadWithMocks("../out/src/build-tool/deferred-ros-task", {
    vscode: h.vscode, child_process: cp,
  }, h.process);
  h.shell = loadWithMocks("../out/src/build-tool/ros-shell", {
    vscode: h.vscode, "../extension": h.extension, "./deferred-ros-task": execution,
    "./windows-colcon-task": { windowsColconExecution: (...args) => {
      const sentinel = { windowsPreflight: true };
      h.windows.push({ args, sentinel });
      return sentinel;
    } },
  }, h.process);
  h.colcon = loadWithMocks("../out/src/build-tool/colcon", {
    vscode: h.vscode, "./ros-shell": h.shell,
    "./colcon-utils": {
      getColconIgnoreConfig: () => ({}),
      getPackages: () => assert.fail("Discovery must not run colcon list"),
      getNonIgnoredPackages: () => assert.fail("Discovery must not run colcon list"),
    },
  }, h.process);
  h.make = (definition = {}, scope) => h.shell.make("fixture", {
    type: "colcon", command: "colcon", args: ["build"], ...definition,
  }, undefined, scope);
  return h;
}

const flush = () => new Promise(resolve => setImmediate(resolve));

async function terminal(task, overrides = {}) {
  // Emulate VS Code's resolvedDefinition, including removal of reserved options.
  const definition = {
    ...task.definition,
    taskOptions: { ...task.definition.taskOptions, cwd: "/resolved workspace" },
    ...overrides,
  };
  delete definition.options;
  const pty = await task.execution.callback(definition);
  const writes = [];
  const codes = [];
  const closed = deferred();
  pty.onDidWrite(text => writes.push(text));
  pty.onDidClose(code => { codes.push(code); closed.resolve(code); });
  return { pty, writes, codes, done: closed.promise };
}

for (const platform of ["linux", "darwin"]) {
  test(`${platform}: provider/package/ROS2 discovery and resolution never wait for activation`, async () => {
    const h = harness(platform, { getEnv: () => assert.fail("No environment read before open") });
    const provider = new h.colcon.ColconProvider();
    const tasks = await provider.provideTasks();
    assert.equal(tasks.length, 4);
    assert.equal(tasks.filter(task => task.group === h.vscode.TaskGroup.Test).length, 2);
    for (const task of tasks) {
      assert.deepEqual(task.problemMatchers, ["$colcon-gcc"]);
      assert.ok(task.definition.args.includes("--symlink-install"));
      assert.equal(task.definition.taskOptions.cwd, "${workspaceFolder}");
    }
    tasks.push(await h.colcon.makeColconPackageTask("camera"));
    const ros = new h.shell.RosShellTaskProvider();
    tasks.push(...ros.provideTasks());
    tasks.push(ros.resolveTask({ definition: { type: "ROS2", command: "ros2", args: ["topic", "list"] } }));
    tasks.push(provider.resolveTask({ definition: { type: "colcon", args: ["test"] } }));
    h.shell.registerRosShellTaskProvider();
    assert.equal(h.registrations[0].type, "ROS2");
    for (const task of tasks) {
      assert.ok(task.execution instanceof h.vscode.CustomExecution);
      await terminal(task); // Even constructing the PTY must not start activation.
    }
    assert.equal(h.reads, 0);
    assert.deepEqual(h.spawns, []);
    assert.deepEqual(h.windows, []);
  });

  test(`${platform}: build, test and ROS2 tasks wait for resolution and read a new environment every run`, async () => {
    let gate = deferred();
    const h = harness(platform, { getEnv: () => gate.promise });
    const tasks = [h.make(), h.make({ args: ["test"] }), h.make({ type: "ROS2", command: "ros2", args: ["node", "list"] })];
    let iteration = 0;
    for (const task of tasks) {
      for (let retry = 0; retry < 2; retry++) {
        gate = deferred();
        const run = await terminal(task);
        assert.equal(h.reads, iteration);
        run.pty.open();
        run.pty.open(); // Defensive: opening twice must not launch twice.
        assert.equal(h.reads, iteration + 1);
        assert.equal(h.spawns.length, iteration);
        const env = { PATH: `/pixi/env${iteration}/bin`, ROS_DISTRO: `distro${iteration}` };
        gate.resolve(env);
        await flush();
        const spawn = h.spawns[iteration];
        assert.deepEqual(spawn.options.env, env);
        assert.notEqual(spawn.options.env, env);
        assert.equal(spawn.options.env.HOST_ONLY, undefined);
        assert.deepEqual(parse(spawn.args.at(-1)), [task.definition.command, ...task.definition.args]);
        spawn.child.emit("close", 0);
        assert.equal(await run.done, 0);
        iteration++;
      }
    }
    assert.equal(h.reads, 6);
    assert.deepEqual(h.windows, []);
  });

  test(`${platform}: resolves in place, preserving definition identity, scope, customizations and execution options`, async () => {
    const h = harness(platform);
    h.env = { PATH: "/pixi/bin", DELETE: "old", EMPTY: "old", KEEP: "kept", lower: "kept" };
    const scope = { uri: { fsPath: "/second workspace" }, name: "second", index: 1 };
    const definition = { type: "colcon", args: ["test", "${input:package}"], options: {
      cwd: "${workspaceFolder}/nested", env: { PATH: "${env:PATH}", DELETE: null },
      shell: { executable: "/bin/bash", args: ["--noprofile", "--norc", "-c"] },
    } };
    const task = {
      definition, scope, name: "custom test", group: h.vscode.TaskGroup.Test,
      presentationOptions: { reveal: 2 }, runOptions: { reevaluateOnRerun: true },
      problemMatchers: ["$custom"], isBackground: true,
    };
    const original = { ...task };
    assert.equal(h.shell.resolve(task), task);
    assert.equal(task.definition, definition);
    assert.equal(definition.command, "colcon");
    assert.deepEqual(definition.taskOptions, definition.options);
    for (const key of Object.keys(original)) { assert.equal(task[key], original[key], key); }
    const args = ["test", "camera & lidar", "", "a'b", 'a"b', "$HOME", "$(touch nope)", "`nope`", "*", "a;b", "a\nb", "a\\b"];
    const env = { PATH: "/resolved/pixi/bin", DELETE: null, EMPTY: "", LOWER: "new" };
    const run = await terminal(task, {
      command: "/resolved tools/colcon", args,
      taskOptions: { ...definition.taskOptions, cwd: "/second workspace/nested", env },
    });
    run.pty.open();
    await flush();
    const spawn = h.spawns[0];
    assert.equal(spawn.command, "/bin/bash");
    assert.deepEqual(spawn.args.slice(0, -1), ["--noprofile", "--norc", "-c"]);
    // shell-quote's parser labels even escaped wildcards as globs. Verify the
    // actual escape separately rather than mistaking that parser quirk for expansion.
    assert.ok(spawn.args.at(-1).includes(" \\* "));
    assert.deepEqual(parse(spawn.args.at(-1)), ["/resolved tools/colcon",
      ...args.map(arg => arg === "*" ? { op: "glob", pattern: "*" } : arg)]);
    assert.equal(spawn.options.cwd, "/second workspace/nested");
    assert.deepEqual(spawn.options.env, { PATH: "/resolved/pixi/bin", EMPTY: "", KEEP: "kept", lower: "kept", LOWER: "new" });
    assert.deepEqual(h.env, { PATH: "/pixi/bin", DELETE: "old", EMPTY: "old", KEEP: "kept", lower: "kept" });
    assert.deepEqual(definition.args, ["test", "${input:package}"]);
    assert.deepEqual(env, { PATH: "/resolved/pixi/bin", DELETE: null, EMPTY: "", LOWER: "new" });
    assert.equal(spawn.options.shell, false);
    assert.equal(spawn.options.detached, true);
    assert.equal(spawn.options.stdio, "pipe");
    spawn.child.emit("close", 9);
    assert.equal(await run.done, 9);
  });

  for (const type of ["ROS2", "colcon"]) {
    test(`${platform}: ${type} restores stripped legacy options only from the matching task scope`, async () => {
      const scope = { uri: { fsPath: "/second workspace" } };
      const platformKey = platform === "darwin" ? "osx" : "linux";
      const configured = { label: "custom", type, command: "ros2", options: {
        cwd: "${workspaceFolder}/nested", env: { CUSTOM: "base", KEEP: "base" },
      }, [platformKey]: { options: { env: { CUSTOM: "platform", DELETE: null } } } };
      const h = harness(platform, { config: uri => {
        assert.equal(uri, scope.uri);
        return {
          inspect: key => {
            assert.equal(key, "tasks");
            return {
              workspaceFolderValue: [configured],
              workspaceValue: [{ ...configured, options: { cwd: "/wrong workspace" } }],
              globalValue: [{ ...configured, options: { cwd: "/wrong global" } }],
            };
          },
          get: key => {
            assert.equal(key, "options");
            return { env: { GLOBAL: "default", KEEP: "global" } };
          },
        };
      } });
      const definition = { type, command: "ros2", args: ["node", "list"], taskOptions: { env: { CUSTOM: "explicit" } } };
      const task = { definition, scope, name: "custom" };
      assert.equal(h.shell.resolve(task), task);
      assert.equal(task.definition, definition);
      assert.equal(definition.options, undefined);
      assert.deepEqual(definition.taskOptions, {
        cwd: "${workspaceFolder}/nested",
        env: { GLOBAL: "default", KEEP: "base", CUSTOM: "explicit", DELETE: null },
      });
      assert.equal(h.reads, 0);
      assert.equal(configured.options.env.CUSTOM, "base", "Do not mutate tasks.json configuration");
      const unrelated = { definition: { type, command: "ros2" }, scope, name: "unrelated" };
      h.shell.resolve(unrelated);
      assert.deepEqual(unrelated.definition.taskOptions, { cwd: "${workspaceFolder}" });
    });
  }

  test(`${platform}: a multi-root folder never borrows a same-named workspace task's options`, () => {
    const h = harness(platform, { config: () => ({
      inspect: () => ({ workspaceValue: [{ label: "custom", type: "colcon", options: { cwd: "/wrong" } }] }),
      get: () => assert.fail("Do not read options from a different task scope"),
    }) });
    h.vscode.workspace.workspaceFile = { fsPath: "/fixture.code-workspace" };
    const task = { name: "custom", scope: { uri: { fsPath: "/second" } }, definition: { type: "colcon" } };
    h.shell.resolve(task);
    assert.deepEqual(task.definition.taskOptions, { cwd: "${workspaceFolder}" });
  });

  test(`${platform}: explicit execution options merge legacy environment overrides`, () => {
    const h = harness(platform);
    const task = h.make({
      options: { cwd: "/legacy", env: { KEEP: "base", OVERRIDE: "base", DELETE: "base" } },
      taskOptions: { env: { OVERRIDE: "custom", DELETE: null } },
    });
    assert.deepEqual(task.definition.taskOptions, {
      cwd: "/legacy", env: { KEEP: "base", OVERRIDE: "custom", DELETE: null },
    });
    assert.equal(task.definition.options.env.DELETE, "base");
  });

  test(`${platform}: default cwd follows the task folder, not the first workspace`, async () => {
    const h = harness(platform);
    h.env = { PATH: "/pixi/bin" };
    const scope = { uri: { fsPath: "/second workspace" } };
    const task = h.make({}, scope);
    assert.equal(task.scope, scope);
    const run = await terminal(task, { taskOptions: {} });
    run.pty.open();
    await flush();
    assert.equal(h.spawns[0].options.cwd, scope.uri.fsPath);
    h.spawns[0].child.emit("close", 0);
    assert.equal(await run.done, 0);
  });

  test(`${platform}: POSIX built-ins and explicitly requested shell scripts retain shell execution`, async () => {
    const h = harness(platform);
    h.env = { PATH: "/pixi/bin" };
    for (const definition of [
      { command: "printf", args: ["%s\\n", "literal $HOME"] },
      { command: "sh", args: ["-c", "printf '%s\\n' \"$ROS_DISTRO\" | cat"] },
    ]) {
      const run = await terminal(h.make(definition));
      run.pty.open();
      await flush();
      const spawn = h.spawns.at(-1);
      assert.equal(spawn.command, "/bin/sh");
      assert.equal(spawn.args[0], "-c");
      assert.deepEqual(parse(spawn.args[1]), [definition.command, ...definition.args]);
      spawn.child.emit("close", 0);
      assert.equal(await run.done, 0);
    }
  });

  test(`${platform}: stdin, UTF-8, split CRLF, both output streams and close-after-exit`, async () => {
    const h = harness(platform);
    h.env = { PATH: "/pixi/bin" };
    const run = await terminal(h.make());
    run.pty.open();
    await flush();
    const child = h.spawns[0].child;
    assert.equal(child.stdout.readableEncoding, "utf8");
    assert.equal(child.stderr.readableEncoding, "utf8");
    const encoded = Buffer.from("café\n");
    child.stdout.write(encoded.subarray(0, 4));
    child.stdout.write(encoded.subarray(4));
    child.stderr.write("warning\r");
    child.stderr.write("\nnext\n");
    run.pty.handleInput("answer\r");
    assert.deepEqual(child.input, ["answer\n"]);
    child.emit("exit", 7);
    assert.deepEqual(run.codes, [], "Do not close until stdio drains");
    child.stdout.write("final diagnostic\n");
    child.emit("close", 7);
    assert.equal(await run.done, 7);
    assert.match(run.writes.join(""), /café\r\nwarning\r\nnext\r\nfinal diagnostic\r\n/);
    const count = run.writes.length;
    child.stdout.write("late output");
    child.emit("error", new Error("late error"));
    run.pty.handleInput("late input\r");
    run.pty.close();
    assert.equal(run.writes.length, count);
    assert.deepEqual(child.input, ["answer\n"]);
    assert.deepEqual(run.codes, [7]);
    assert.deepEqual(h.kills, []);
  });

  test(`${platform}: cancellation before open never reads environment`, async () => {
    const h = harness(platform);
    const run = await terminal(h.make());
    run.pty.close();
    run.pty.open();
    assert.equal(await run.done, 130);
    assert.equal(h.reads, 0);
    assert.deepEqual(h.spawns, []);
  });

  for (const outcome of ["resolve", "reject"]) {
    test(`${platform}: cancellation while awaiting environment suppresses late ${outcome}`, async () => {
      const gate = deferred();
      const h = harness(platform, { getEnv: () => gate.promise });
      const run = await terminal(h.make());
      run.pty.open();
      run.pty.handleInput("\x03");
      assert.equal(await run.done, 130);
      const count = run.writes.length;
      if (outcome === "resolve") { gate.resolve({ PATH: "/late/bin" }); }
      else { gate.reject(new Error("late activation failure")); }
      await flush();
      assert.deepEqual(h.spawns, []);
      assert.deepEqual(h.kills, []);
      assert.equal(run.writes.length, count);
      assert.deepEqual(run.codes, [130]);
    });
  }

  for (const fallback of [false, true]) {
    test(`${platform}: running cancellation kills the process group${fallback ? " with fallback" : ""}`, async () => {
      const h = harness(platform, { killError: fallback ? new Error("fixture group kill failure") : undefined });
      h.env = { PATH: "/pixi/bin" };
      const run = await terminal(h.make());
      run.pty.open();
      await flush();
      const child = h.spawns[0].child;
      if (fallback) { run.pty.close(); }
      else { run.pty.handleInput("\x03"); }
      run.pty.close();
      assert.equal(await run.done, 130);
      assert.deepEqual(h.kills, [{ pid: -child.pid, signal: "SIGKILL" }]);
      assert.deepEqual(child.kills, fallback ? ["SIGKILL"] : []);
      child.emit("close", 0);
      assert.deepEqual(run.codes, [130]);
    });
  }

  for (const failure of ["environment", "undefined environment", "spawn throw", "child error", "stdin error", "null exit", "unsupported shell"]) {
    test(`${platform}: ${failure} produces one failed close`, async () => {
      const h = harness(platform, {
        getEnv: failure === "environment" ? () => Promise.reject(new Error("fixture sourcing failure")) : undefined,
        spawnError: failure === "spawn throw" ? new Error("fixture ENOENT") : undefined,
      });
      if (failure !== "undefined environment") { h.env = { PATH: "/pixi/bin" }; }
      const overrides = failure === "unsupported shell" ? { taskOptions: { shell: { executable: "/bin/fish" } } } : {};
      const run = await terminal(h.make(), overrides);
      run.pty.open();
      await flush();
      const child = h.spawns[0]?.child;
      if (failure === "child error") { child.emit("error", new Error("fixture ENOENT")); }
      if (failure === "stdin error") { child.stdin.emit("error", new Error("fixture EPIPE")); }
      if (child) { child.emit("close", null); }
      assert.equal(await run.done, 1);
      run.pty.close();
      assert.deepEqual(run.codes, [1]);
      if (failure !== "null exit") { assert.match(run.writes.join(""), /ROS task failed:/); }
      if (["environment", "undefined environment", "unsupported shell"].includes(failure)) {
        assert.deepEqual(h.spawns, []);
      }
    });
  }
}

test("Windows colcon build still delegates unchanged to its isolated preflight", () => {
  const h = harness("win32");
  const task = h.make({ options: { cwd: "C:\\build", env: { CUSTOM: "value" } } });
  assert.equal(task.execution, h.windows[0].sentinel);
  assert.deepEqual(task.definition.buildOptions, { cwd: "C:\\build", env: { CUSTOM: "value" } });
  assert.equal(task.definition.taskOptions, undefined);
  assert.equal(h.windows[0].args[0](), h.process.env);
  assert.equal(h.reads, 0);
  assert.deepEqual(h.spawns, []);
  const testTask = h.make({ args: ["test"] });
  assert.ok(testTask.execution instanceof h.vscode.ShellExecution);
});