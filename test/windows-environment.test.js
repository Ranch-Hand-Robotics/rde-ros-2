const assert = require("node:assert/strict");
const childProcess = require("node:child_process");
const { EventEmitter } = require("node:events");
const fs = require("node:fs/promises");
const Module = require("node:module");
const os = require("node:os");
const path = require("node:path");
const { test } = require("node:test");
const { promisify } = require("node:util");
const { findWindowsPixiEnvironment, sourceWindowsEnvironment } = require("../out/src/ros/windows-env");
const { pixiExecutableCandidates } = require("../out/src/ros/installer/pixi");
const { activateWindowsToolchain, hasWindowsToolchain, WINDOWS_BUILD_TOOLS_COMMAND } = require("../out/src/ros/windows-toolchain");
const { buildParentScripts, cleanBuildEnvironment, isWorkspaceInstall } = require("../out/src/ros/build-environment");

const realExecFile = promisify(childProcess.execFile);

function loadWithMocks(filename, mocks) {
  const resolved = require.resolve(filename);
  const original = Module._load;
  delete require.cache[resolved];
  Module._load = function(request, parent, isMain) {
    return Object.hasOwn(mocks, request) ? mocks[request] : original.call(this, request, parent, isMain);
  };
  try {
    return require(resolved);
  } finally {
    Module._load = original;
    delete require.cache[resolved];
  }
}

function mockExecFile(run) {
  const stub = () => { throw new Error("Expected promisified execFile"); };
  stub[promisify.custom] = run;
  return { execFile: stub };
}

async function fixture(t) {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), "ROS user's & test!-"));
  t.after(() => fs.rm(directory, { recursive: true, force: true }));
  return directory;
}

async function compilerFixture(root) {
  const sdk = path.join(root, "SDK");
  const bin = path.join(root, "tools");
  for (const file of ["Include/10.0/um/Windows.h", "Include/10.0/ucrt/stdio.h",
    "Lib/10.0/um/x64/kernel32.lib", "Lib/10.0/ucrt/x64/ucrt.lib"]) {
    const filename = path.join(sdk, file);
    await fs.mkdir(path.dirname(filename), { recursive: true });
    await fs.writeFile(filename, "");
  }
  await fs.mkdir(bin, { recursive: true });
  for (const tool of ["cl.exe", "link.exe", "rc.exe"]) { await fs.writeFile(path.join(bin, tool), ""); }
  const env = Object.fromEntries(Object.entries(process.env).filter(([key]) => key.toLowerCase() !== "path"));
  // Never let a real host compiler conceal a missing fixture tool. Native CMD
  // still needs Windows utilities such as chcp, independently of the host PATH.
  const directories = [bin];
  if (process.platform === "win32") { directories.push(path.join(process.env.SystemRoot, "System32")); }
  return { ...env, Path: directories.join(";"), VisualStudioVersion: "17.0",
    VSCMD_ARG_TGT_ARCH: "x64", WindowsSdkDir: sdk, WindowsSDKVersion: "10.0\\",
    INCLUDE: path.join(sdk, "Include"), LIB: path.join(sdk, "Lib") };
}

test("toolchain readiness requires actual compiler and SDK files, and preserves ready environments", async t => {
  const root = await fixture(t);
  const env = await compilerFixture(root);
  assert.equal(await hasWindowsToolchain(env), true);
  assert.equal(await activateWindowsToolchain(env, { visualStudioSetup: [] }), env);
  for (const key of ["VisualStudioVersion", "INCLUDE", "LIB", "WindowsSdkDir", "WindowsSDKVersion", "VSCMD_ARG_TGT_ARCH"]) {
    assert.equal(await hasWindowsToolchain({ ...env, [key]: "" }), false, key);
  }
  assert.equal(await hasWindowsToolchain({ ...env, VSCMD_ARG_TGT_ARCH: "x86" }), false);
  await fs.unlink(path.join(root, "tools", "cl.exe"));
  assert.equal(await hasWindowsToolchain(env), false);
  await fs.writeFile(path.join(root, "tools", "cl.exe"), "");
  await fs.unlink(path.join(root, "SDK", "Lib", "10.0", "um", "x64", "kernel32.lib"));
  assert.equal(await hasWindowsToolchain(env), false);
  await assert.rejects(activateWindowsToolchain(env, { visualStudioSetup: [] }), /Windows SDK.*Administrator PowerShell/);
});

test("vswhere includes Build Tools, filters for VS2022 MSVC, and handles custom installation paths", async t => {
  const root = await fixture(t);
  const vswhere = path.join(root, "Microsoft Visual Studio", "Installer", "vswhere.exe");
  await fs.mkdir(path.dirname(vswhere), { recursive: true });
  await fs.writeFile(vswhere, "");
  const loaded = loadWithMocks("../out/src/ros/windows-toolchain", {
    child_process: mockExecFile(async (exe, args) => {
      assert.equal(exe, vswhere);
      assert.deepEqual(args, ["-products", "*", "-version", "[17.0,18.0)", "-requires",
        "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-sort", "-property", "installationPath", "-utf8"]);
      return { stdout: `${root}\r\nrelative-invalid\r\n`, stderr: "" };
    }),
  });
  assert.deepEqual(await loaded.findWindowsToolchainSetups({ "ProgramFiles(x86)": root }),
    [path.join(root, "VC", "Auxiliary", "Build", "vcvarsall.bat")]);
  assert.match(WINDOWS_BUILD_TOOLS_COMMAND, /--id Microsoft.VisualStudio.2022.BuildTools --exact --source winget/);
  assert.match(WINDOWS_BUILD_TOOLS_COMMAND, /--wait --passive --norestart/);
  assert.match(WINDOWS_BUILD_TOOLS_COMMAND, /--add Microsoft.VisualStudio.Component.VC.Tools.x86.x64/);
  const installerArguments = WINDOWS_BUILD_TOOLS_COMMAND.match(/--override "([^"]+)"$/)?.[1];
  assert.ok(installerArguments, "Component selections must be inside the installer override");
  assert.match(installerArguments, /(?:^| )--add Microsoft\.VisualStudio\.Component\.VC\.ATL(?: |$)/);
  assert.match(WINDOWS_BUILD_TOOLS_COMMAND, /--add Microsoft.VisualStudio.Component.Windows11SDK.26100/);
  assert.doesNotMatch(WINDOWS_BUILD_TOOLS_COMMAND, /--force|--includeRecommended|--accept/);
});

test("missing tools stop activation before any Pixi process is started", async () => {
  const loaded = loadWithMocks("../out/src/ros/windows-env", {
    "./installer/pixi": { findPixi: async () => assert.fail("Must not start Pixi") },
  });
  await assert.rejects(loaded.sourceWindowsEnvironment("missing.bat", {}, { visualStudioSetup: [] }), /Visual Studio 2022/);
});

test("finds Pixi in a sourced Windows Path regardless of variable casing", () => {
  const directory = path.join(os.tmpdir(), "custom pixi");
  for (const key of ["Path", "PATH", "pAtH"]) {
    assert.ok(pixiExecutableCandidates("win32", os.tmpdir(), { [key]: directory }, [])
      .includes(path.join(directory, "pixi.exe")));
  }
});

test("selects the nearest Pixi manifest and the environment in the setup path", async t => {
  const root = await fixture(t);
  const project = path.join(root, "lyrical");
  const setup = path.join(project, ".pixi", "envs", "lyrical", "Library", "local_setup.bat");
  await fs.mkdir(path.dirname(setup), { recursive: true });
  await fs.writeFile(path.join(root, "pixi.toml"), "");
  await fs.writeFile(path.join(project, "pixi.toml"), "");
  assert.deepEqual(await findWindowsPixiEnvironment(setup), {
    manifest: path.join(project, "pixi.toml"), environment: "lyrical",
  });
  await fs.rename(path.join(project, "pixi.toml"), path.join(project, "pyproject.toml"));
  assert.deepEqual(await findWindowsPixiEnvironment(setup), {
    manifest: path.join(project, "pyproject.toml"), environment: "lyrical",
  });
});

test("supports legacy default environments and leaves non-Pixi setups alone", async t => {
  const root = await fixture(t);
  const setup = path.join(root, "ros2-windows", "local_setup.bat");
  assert.equal(await findWindowsPixiEnvironment(setup), undefined);
  await fs.writeFile(path.join(root, "pixi.toml"), "");
  assert.deepEqual(await findWindowsPixiEnvironment(setup), {
    manifest: path.join(root, "pixi.toml"), environment: "default",
  });
});

test("CMD sources compiler, named Pixi hook and ROS, preserves overlays, and cleans up", {
  skip: process.platform !== "win32",
}, async t => {
  const root = await fixture(t);
  const project = path.join(root, "lyrical");
  const setup = path.join(project, ".pixi", "envs", "lyrical", "Library", "local_setup.bat");
  await fs.mkdir(path.dirname(setup), { recursive: true });
  await fs.writeFile(path.join(project, "pixi.toml"), "");
  await fs.writeFile(setup, '@echo off\r\nset "ROS_DISTRO=lyrical"\r\nset "ROS_VERSION=2"\r\necho diagnostic=value\r\n');
  const compiler = path.join(root, "vcvarsall.bat");
  const compilerEnv = await compilerFixture(root);
  await fs.writeFile(compiler, '@echo off\r\nset "TEST_COMPILER=%1"\r\n' +
    ["Path", "VisualStudioVersion", "VSCMD_ARG_TGT_ARCH", "WindowsSdkDir", "WindowsSDKVersion", "INCLUDE", "LIB"]
      .map(key => `set "${key}=${compilerEnv[key]}"`).join("\r\n") + "\r\n");
  const scripts = [];
  const logs = [];
  let hooks = 0;
  const { sourceWindowsEnvironment } = loadWithMocks("../out/src/ros/windows-env", {
    "./installer/pixi": { findPixi: async () => "test-pixi.exe" },
    child_process: mockExecFile(async (exe, args, options) => {
      if (exe === "test-pixi.exe") {
        hooks++;
        assert.deepEqual(args, ["shell-hook", "--shell", "cmd", "--manifest-path", path.join(project, "pixi.toml"),
          "--environment", "lyrical", "--frozen", "--no-install"]);
        assert.equal(options.cwd, project);
        assert.equal(options.env.VisualStudioVersion, "17.0", "Compiler must be activated BEFORE hook generation");
        assert.equal(options.env.TEST_COMPILER, "x64");
        return { stdout: '@echo off\r\nset "TEST_PIXI=active"\r\nset "TEST_ORDER=%TEST_COMPILER%"\r\n', stderr: "" };
      }
      scripts.push(args.at(-1).replace(/^"|"$/g, ""));
      return realExecFile(exe, args, options);
    }),
  });
  const env = await sourceWindowsEnvironment(setup, {
    ...process.env, TEST_SECRET: "not for logs", TEST_EQUALS: "first=second", PIXI_TEMP_BAT: "stale-missing-file.bat",
  }, { cwd: root, visualStudioSetup: [path.join(root, "missing.bat"), compiler], onOutput: text => logs.push(text) });
  assert.equal(env.ROS_DISTRO, "lyrical");
  assert.equal(env.ROS_VERSION, "2");
  assert.equal(env.TEST_PIXI, "active");
  assert.equal(env.TEST_ORDER, "x64");
  assert.equal(env.VisualStudioVersion, "17.0");
  assert.equal(env.TEST_EQUALS, "first=second");
  assert.equal(env.diagnostic, undefined);
  assert.doesNotMatch(logs.join("\n"), /not for logs/);
  const overlay = path.join(root, "overlay.bat");
  await fs.writeFile(overlay, '@echo off\r\nset "TEST_OVERLAY=yes"\r\n');
  const overlaid = await sourceWindowsEnvironment(overlay, env);
  assert.equal(overlaid.TEST_PIXI, "active");
  assert.equal(overlaid.TEST_OVERLAY, "yes");
  const nestedOverlay = path.join(project, "install", "setup.bat");
  await fs.mkdir(path.dirname(nestedOverlay), { recursive: true });
  await fs.copyFile(overlay, nestedOverlay);
  const nested = await sourceWindowsEnvironment(nestedOverlay, env);
  assert.equal(nested.TEST_PIXI, "active");
  assert.equal(nested.TEST_OVERLAY, "yes");
  assert.equal(hooks, 1, "An overlay must not switch back to a default Pixi environment");
  for (const script of scripts) {
    assert.equal(await fs.stat(path.dirname(script)).then(() => true, () => false), false);
  }
});

test("CMD propagates setup and hook failures instead of accepting a partial environment", {
  skip: process.platform !== "win32",
}, async t => {
  const root = await fixture(t);
  const env = await compilerFixture(root);
  const setup = path.join(root, "setup.bat");
  await fs.writeFile(setup, "@exit /b 42\r\n");
  const { sourceWindowsEnvironment } = loadWithMocks("../out/src/ros/windows-env", {});
  await assert.rejects(sourceWindowsEnvironment(setup, env), error => error.code === 42);
  await fs.writeFile(path.join(root, "pixi.toml"), "");
  for (const mode of ["missing", "failure", "empty", "hook-failure"]) {
    const scripts = [];
    const loaded = loadWithMocks("../out/src/ros/windows-env", {
      "./installer/pixi": { findPixi: async () => mode === "missing" ? undefined : "test-pixi.exe" },
      child_process: mockExecFile(async (exe, args, options) => {
        if (exe !== "test-pixi.exe") {
          scripts.push(args.at(-1).replace(/^"|"$/g, ""));
          return realExecFile(exe, args, options);
        }
        if (mode === "failure") { throw new Error("Pixi activation failed"); }
        return { stdout: mode === "empty" ? "" : "@exit /b 17\r\n", stderr: "" };
      }),
    });
    await assert.rejects(loaded.sourceWindowsEnvironment(setup, env), error => mode === "hook-failure"
      ? error.code === 17 : /Pixi/.test(error.message));
    for (const script of scripts) {
      assert.equal(await fs.stat(path.dirname(script)).then(() => true, () => false), false);
    }
  }
});

test("colcon applicability waits for successful discovery, logs errors and preserves spaces", async () => {
  const logs = [];
  const env = { ROS_VERSION: "2" };
  const root = path.join(os.tmpdir(), "ROS workspace & more");
  let result = { stdout: "camera\tsrc/camera package\t(ros.ament_cmake)\r\n", stderr: "" };
  let complete;
  const utils = loadWithMocks("../out/src/build-tool/colcon-utils", {
    vscode: {},
    "../extension": { env, outputChannel: { appendLine: text => logs.push(text) } },
    "../vscode-utils": {},
    child_process: mockExecFile(async (exe, args, options) => {
      assert.equal(exe, process.platform === "win32" ? "colcon.exe" : "colcon");
      assert.deepEqual(args, ["--log-base", process.platform === "win32" ? "nul" : "/dev/null", "list", "--base-paths", root]);
      assert.equal(options.env, env);
      assert.equal(options.cwd, root);
      assert.equal(options.timeout, 60000);
      if (complete) { await complete; }
      if (result instanceof Error) { throw result; }
      return result;
    }),
  });
  const colcon = loadWithMocks("../out/src/build-tool/colcon", {
    vscode: {}, "./ros-shell": {}, "./colcon-utils": utils,
  });
  assert.deepEqual(await utils.getPackages(root), [{ name: "camera", path: "src/camera package" }]);
  assert.equal(await colcon.isApplicable(root), true);
  for (const output of ["", " \r\n", "not a package"]) {
    result = { stdout: output, stderr: "" };
    assert.equal(await colcon.isApplicable(root), false);
  }
  let finish;
  complete = new Promise(resolve => { finish = resolve; });
  const pending = colcon.isApplicable(root);
  result = Object.assign(new Error("colcon failed: missing executable"), { stdout: "camera\tsrc/camera\t(type)" });
  finish();
  assert.equal(await pending, false, "Failed discovery must not accept partial stdout");
  assert.match(logs.at(-1), /Colcon package discovery failed.*missing executable/);
});

test("CMD compiler failure and incomplete SDK are rejected, with fallback to another installation", {
  skip: process.platform !== "win32",
}, async t => {
  const root = await fixture(t);
  const broken = path.join(root, "broken.bat");
  const partial = path.join(root, "partial.bat");
  const good = path.join(root, "good.bat");
  const ready = await compilerFixture(root);
  await fs.writeFile(broken, "@exit /b 8\r\n");
  await fs.writeFile(partial, '@set "VisualStudioVersion=17.0"\r\n');
  await fs.writeFile(good, "@echo off\r\n" + ["Path", "VisualStudioVersion", "VSCMD_ARG_TGT_ARCH",
    "WindowsSdkDir", "WindowsSDKVersion", "INCLUDE", "LIB"].map(key => `set "${key}=${ready[key]}"`).join("\r\n") + "\r\n");
  await assert.rejects(activateWindowsToolchain({}, { visualStudioSetup: [broken, partial] }), /Windows SDK/);
  const env = await activateWindowsToolchain({}, { visualStudioSetup: [broken, partial, good] });
  assert.equal(await hasWindowsToolchain(env), true);
});

test("batch capture preserves special paths and values and cleans up on failure", {
  skip: process.platform !== "win32",
}, async t => {
  const scripts = [];
  const loaded = loadWithMocks("../out/src/ros/windows-batch", {
    child_process: mockExecFile(async (exe, args, options) => {
      scripts.push(args.at(-1).replace(/^"|"$/g, ""));
      return realExecFile(exe, args, options);
    }),
  });
  const root = await fixture(t);
  const env = await loaded.sourceWindowsBatch(['set "TEST_VALUE=first=second&!"'], process.env, { cwd: root });
  assert.equal(env.TEST_VALUE, "first=second&!");
  await assert.rejects(loaded.sourceWindowsBatch(["exit /b 9"], process.env), error => error.code === 9);
  assert.equal(scripts.length, 2);
  for (const script of scripts) {
    assert.equal(await fs.stat(path.dirname(script)).then(() => true, () => false), false);
  }
});

test("CMD UTF-8 batches roundtrip non-ASCII paths and values despite legacy codepage changes", {
  skip: process.platform !== "win32",
}, async t => {
  const root = path.join(await fixture(t), "caf\u00e9-\u65e5\u672c\u8a9e");
  await fs.mkdir(root);
  const env = await compilerFixture(root);
  const expected = "caf\u00e9 \u65e5\u672c\u8a9e = & !";
  env.TEST_INHERITED = expected;
  env.TEST_SECRET = "private environment value";
  const compiler = path.join(root, "vcvars.bat");
  const setup = path.join(root, "\u74b0\u5883-setup.bat");
  const scripts = [];
  const encodings = [];
  const logs = [];
  const loaded = loadWithMocks("../out/src/ros/windows-batch", {
    fs: { promises: { ...fs, mkdtemp: () => fs.mkdtemp(path.join(root, "wrapper-")) } },
    child_process: mockExecFile(async (exe, args, options) => {
      scripts.push(args.at(-1).replace(/^"|"$/g, ""));
      encodings.push(options.encoding);
      // Start a real CMD on an OEM codepage even on UTF-8-configured hosts.
      // /s removes the outer quotes, retaining the quoted Unicode script path.
      return realExecFile(exe, [...args.slice(0, -1),
        `"chcp 437 >nul && ${args.at(-1)}"`], options);
    }),
  });
  await fs.writeFile(setup, `@echo off\r\nset "TEST_CAPTURE=${expected}"\r\n` +
    "chcp 850 >nul\r\nexit /b 0\r\n", "utf8");
  for (const code of [0, 42, -7]) {
    await fs.writeFile(compiler, `@echo off\r\nset "TEST_COMPILER=${expected}"\r\n` +
      `echo Compiler setup diagnostic\r\nchcp 850 >nul\r\nexit /b ${code}\r\n`, "utf8");
    // Deliberately omit caller-side guards: restoring the codepage must not
    // turn a failed CALL (including a negative exit code) into a success.
    const capture = loaded.sourceWindowsBatch([
      `call ${loaded.quoteBatchPath(compiler)}`, `call ${loaded.quoteBatchPath(setup)}`,
    ], env, { cwd: root, onOutput: message => logs.push(message) });
    if (code) {
      // Node reports the Windows DWORD exit code as an unsigned integer.
      await assert.rejects(capture, error => error.code === (code >>> 0));
    } else {
      const result = await capture;
      assert.equal(result.TEST_COMPILER, expected, "Bootstrap must precede UTF-8 batch parsing");
      assert.equal(result.TEST_CAPTURE, expected, "Restore UTF-8 before the next non-ASCII CALL");
      assert.equal(result.TEST_INHERITED, expected, "Capture must restore UTF-8 after setup changes it");
      assert.equal(result.TEST_SECRET, env.TEST_SECRET);
      assert.equal(result.Path, env.Path, "Non-ASCII environment paths must roundtrip exactly");
    }
  }
  assert.match(logs.join("\n"), /Compiler setup diagnostic/);
  assert.doesNotMatch(logs.join("\n"), /private environment value|TEST_SECRET|__RDE_ROS_ENVIRONMENT__/);
  assert.equal(scripts.length, 3);
  assert.deepEqual(encodings, ["utf8", "utf8", "utf8"], "Decode CMD's UTF-8 output explicitly");
  for (const script of scripts) {
    assert.equal(await fs.stat(path.dirname(script)).then(() => true, () => false), false);
  }
});

test("Windows health probes activate compiler tools in a clean environment before Pixi", {
  skip: process.platform !== "win32",
}, async t => {
  const root = await fixture(t);
  const prefix = path.join(root, ".pixi", "envs", "jazzy");
  await fs.mkdir(prefix, { recursive: true });
  const probe = path.join(root, "probe.py");
  const executable = path.join(root, "pixi.exe");
  for (const filename of [probe, executable, path.join(prefix, "python.exe"),
    path.join(root, "pixi.toml"), path.join(root, "pixi.lock")]) { await fs.writeFile(filename, ""); }
  let activated = false;
  let started = false;
  let failActivation = false;
  const loaded = loadWithMocks("../out/src/ros/installer/health-check", {
    "../windows-toolchain": { activateWindowsToolchain: async env => {
      assert.equal(env.ROS_DISTRO, undefined);
      assert.equal(env.PYTHONPATH, undefined);
      assert.equal(env.ROS_LOCALHOST_ONLY, "1");
      if (failActivation) { throw new Error("Compiler missing"); }
      activated = true;
      return { ...env, VisualStudioVersion: "17.0" };
    } },
    child_process: { execFile: childProcess.execFile, spawn: (command, args, options) => {
      assert.equal(activated, true);
      assert.equal(command, executable, "Use installer-detected Pixi even if absent from PATH");
      assert.equal(options.env.VisualStudioVersion, "17.0");
      assert.ok(args.includes("--no-install"));
      started = true;
      const child = new EventEmitter();
      child.stdout = new EventEmitter();
      child.stderr = new EventEmitter();
      queueMicrotask(() => {
        child.stdout.emit("data", Buffer.from("RDE_ROS_HEALTH:" + JSON.stringify({ version: 1, distro: "jazzy",
          checks: ["environment", "prefix", "ros2-cli", "package-discovery", "rclpy-import", "rmw-init", "pub-sub"]
            .map(id => ({ id, status: "passed", detail: "mock", durationMs: 1 })) })));
        child.emit("close", 0);
      });
      return child;
    } },
  });
  const target = { kind: "pixi", distro: "jazzy", workspace: root, pixiExecutable: executable };
  assert.equal((await loaded.validateInstallation(target, probe)).healthy, true);
  assert.equal(started, true);
  started = false;
  failActivation = true;
  const failed = await loaded.validateInstallation(target, probe);
  assert.equal(failed.healthy, false);
  assert.equal(failed.checks[0].id, "activation");
  assert.equal(started, false);
  await assert.rejects(loaded.buildHealthCommand({ ...target, pixiExecutable: "relative.exe" }, probe), /absolute/);
});

test("batch logs setup diagnostics on success/failure but never the captured environment", async () => {
  for (const fail of [false, true]) {
    const logs = [];
    const result = { stdout: "SDK setup diagnostic\r\n__RDE_ROS_ENVIRONMENT__\r\nSECRET=private-value\r\n",
      stderr: "SDK stderr diagnostic" };
    const failure = Object.assign(new Error("setup failed"), result, { code: 42 });
    const loaded = loadWithMocks("../out/src/ros/windows-batch", {
      child_process: mockExecFile(async () => {
        if (fail) { throw failure; }
        return result;
      }),
    });
    const capture = loaded.sourceWindowsBatch([], {}, { onOutput: message => logs.push(message) });
    if (fail) { await assert.rejects(capture, error => error === failure); }
    else { assert.equal((await capture).SECRET, "private-value"); }
    assert.match(logs.join("\n"), /SDK setup diagnostic/);
    assert.match(logs.join("\n"), /SDK stderr diagnostic/);
    assert.doesNotMatch(logs.join("\n"), /SECRET|private-value|__RDE_ROS_ENVIRONMENT__/);
  }
});

test("real CMD failure retains stdout diagnostics and the original exit code", {
  skip: process.platform !== "win32",
}, async () => {
  const { sourceWindowsBatch } = require("../out/src/ros/windows-batch");
  const logs = [];
  await assert.rejects(sourceWindowsBatch(["echo Missing SDK headers", "exit /b 42"], process.env,
    { onOutput: message => logs.push(message) }), error => error.code === 42);
  assert.match(logs.join("\n"), /Missing SDK headers/);
});

test("real CMD missing-hook diagnostics are fatal only for strict build/test sourcing", {
  skip: process.platform !== "win32",
}, async t => {
  const root = await fixture(t);
  const env = await compilerFixture(root);
  const setup = path.join(root, "local_setup.bat");
  await fs.writeFile(setup, '@echo off\r\nset "PARTIAL_HOOK=not validated"\r\necho not found: "missing\\local_setup.bat" 1>&2\r\nexit /b 0\r\n');
  const logs = [];
  await assert.rejects(sourceWindowsEnvironment(setup, env, {
    failOnMissingSetup: true, onOutput: text => logs.push(text),
  }), /missing setup hook/);
  assert.match(logs.join("\n"), /missing\\local_setup.bat/);
  assert.equal(env.PARTIAL_HOOK, undefined);
  const runtime = await sourceWindowsEnvironment(setup, env);
  assert.equal(runtime.PARTIAL_HOOK, "not validated", "Do not globally change runtime setup semantics");
});

test("build cleanup normalizes self paths and deduplicates external prefixes without losing other values", () => {
  const self = "S:\\ws\\camera\\install";
  const external = "C:\\pixi\\Library";
  const env = { Path: `"${self}\\bin";S:/ws/camera/install/pkg/bin;${self}-other\\bin;SDK`,
    AMENT_PREFIX_PATH: `${self}\\;${external};${external.toLowerCase()}\\\\`,
    CMAKE_PREFIX_PATH: external, COLCON_PREFIX_PATH: `${self.toUpperCase()}\\pkg`,
    CAMERA_DIR: `${self}\\share\\camera`, EXTERNAL_DIR: external, EMPTY: "", SECRET: "do not log" };
  const clean = cleanBuildEnvironment(env, [self]);
  assert.equal(clean.Path, `${self}-other\\bin;SDK`);
  assert.equal(clean.AMENT_PREFIX_PATH, external);
  assert.equal(clean.COLCON_PREFIX_PATH, undefined);
  assert.equal(clean.CAMERA_DIR, undefined);
  assert.equal(clean.EXTERNAL_DIR, external);
  assert.equal(clean.EMPTY, "");
  assert.equal(clean.SECRET, "do not log");
  assert.equal(env.CAMERA_DIR, `${self}\\share\\camera`);
  assert.equal(isWorkspaceInstall(`${self}-other`, [self]), false);
});

test("recorded build parents retain order and external dependencies but skip repeated self aliases", async t => {
  const root = await fixture(t);
  const install = path.join(root, "install");
  await fs.mkdir(install);
  assert.deepEqual(await buildParentScripts([install]), [], "First build has no setup");
  const script = path.join(install, "setup.bat");
  await fs.writeFile(script, "");
  assert.deepEqual(await buildParentScripts([install]), [], "Interrupted empty setup is safe to skip");
  const base = "C:\\pixi\\Library\\local_setup.bat";
  const external = "D:\\external\\local_setup.bat";
  await fs.writeFile(script, ':: generated from colcon_core/shell/template/prefix_chain.bat.em\n' +
    [base, base.toLowerCase(), `${install}\\local_setup.bat`, external,
      `${install.toUpperCase()}\\\\local_setup.bat`, "%%~dp0local_setup.bat"]
      .map(parent => `call:_colcon_prefix_chain_bat_call_script "${parent}"`).join("\n"));
  assert.deepEqual(await buildParentScripts([install]), [base.toLowerCase(), external]);
  for (const unsupported of ['call custom-underlays.bat',
    ':: generated from colcon_core/shell/template/prefix_chain.bat.em\ncall:_colcon_prefix_chain_bat_call_script "%CUSTOM_PARENT%\\local_setup.bat"']) {
    await fs.writeFile(script, unsupported);
    await assert.rejects(buildParentScripts([install]), /external underlay/);
    assert.equal(await fs.readFile(script, "utf8"), unsupported, "Never rewrite generated/user setup scripts");
  }
});