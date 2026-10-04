const assert = require("node:assert/strict");
const fs = require("node:fs/promises");
const os = require("node:os");
const path = require("node:path");
const { execFileSync } = require("node:child_process");
const { promisify } = require("node:util");
const { execFile } = require("node:child_process");
const { Worker } = require("node:worker_threads");
const { test } = require("node:test");
const { findPixi, pixiExecutableCandidates, quoteShell, pixiPlatform, pixiManifest, pixiSetupScript, macInstallScript, macOSVersion, sourceBashEnvironment } = require("../out/src/ros/installer/pixi");

test("finds and validates Pixi outside the inherited PATH", async () => {
  const home = await fs.mkdtemp(path.join(os.tmpdir(), "pixi-test-"));
  try {
    const executable = path.join(home, ".pixi", "bin", "pixi");
    await fs.mkdir(path.dirname(executable), { recursive: true });
    await fs.writeFile(executable, "#!/bin/sh\necho 'pixi 1.0'\n", { mode: 0o700 });
    assert.equal(await findPixi("darwin", home, { PATH: "" }, []), executable);
    await fs.writeFile(executable, "#!/bin/sh\nexit 1\n");
    assert.equal(await findPixi("darwin", home, { PATH: "" }, []), undefined);
    assert.equal(await findPixi("darwin", home, { PATH: "", PIXI_HOME: path.join(home, "custom") }, []), undefined);
  } finally {
    await fs.rm(home, { recursive: true, force: true });
  }
});

test("shell quotes paths containing spaces and shell metacharacters", () => {
  const value = "/tmp/ROS user's $(exit 42) `exit 43` workspace";
  assert.equal(execFileSync("/bin/bash", ["-c", `printf '%s' ${quoteShell(value)}`], { encoding: "utf8" }), value);
});

test("manifest solves only the selected distro and host without a nonexistent overlay", () => {
  assert.equal(pixiPlatform("darwin", "arm64"), "osx-arm64");
  assert.equal(pixiPlatform("darwin", "x64"), "osx-64");
  assert.equal(pixiPlatform("win32", "x64"), "win-64");
  assert.throws(() => pixiPlatform("darwin", "ia32"));
  const manifest = pixiManifest("jazzy", "osx-arm64", "26.6.2");
  assert.match(manifest, /platforms = \[\{ platform = "osx-arm64", macos = "26\.6\.2" \}\]/);
  assert.match(manifest, /robostack-jazzy/);
  assert.doesNotMatch(manifest, /humble|rolling|activation|install\/setup/);
  assert.match(pixiManifest("rolling", "osx-64", "14.0"), /ros2-desktop/);
  assert.throws(() => pixiManifest("$(exit)", "osx-64"));
});

test("macOS solver requirements use the host version for all environments", () => {
  for (const platform of ["osx-arm64", "osx-64"]) {
    for (const macos of ["13.7", "14.0", "26.6.2"]) {
      const manifest = pixiManifest("rolling", platform, macos);
      assert.ok(manifest.includes(`platforms = [{ platform = "${platform}", macos = "${macos}" }]`));
      assert.doesNotMatch(manifest, /system-requirements/);
    }
    for (const invalid of [undefined, "", "Darwin 25", '14.0"\n[dependencies]']) {
      assert.throws(() => pixiManifest("rolling", platform, invalid), /macOS version/);
    }
  }
  assert.doesNotMatch(pixiManifest("jazzy", "win-64"), /system-requirements|macos/);
  assert.match(pixiManifest("jazzy", "win-64"), /platforms = \["win-64"\]/);
});

test("live Rolling solve uses the detected macOS version without installing", {
  skip: process.platform !== "darwin" || process.env.RDE_TEST_PIXI_SOLVE !== "1",
  timeout: 180000,
}, async () => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), "rde-rolling-solve-"));
  try {
    const executable = await findPixi();
    assert.ok(executable, "Pixi must be installed for the solve test");
    const macos = await macOSVersion();
    assert.equal(macos, execFileSync("/usr/bin/sw_vers", ["-productVersion"], { encoding: "utf8" }).trim());
    const manifest = path.join(directory, "pixi.toml");
    await fs.writeFile(manifest, pixiManifest("rolling", pixiPlatform(), macos));
    const result = await promisify(execFile)(executable, ["lock", "--dry-run", "--manifest-path", manifest], { timeout: 150000, maxBuffer: 10 * 1024 * 1024 });
    assert.doesNotMatch(result.stdout + result.stderr, /system-requirements|warnings? while parsing the manifest/i);
    assert.equal(await fs.stat(path.join(directory, ".pixi", "envs")).then(() => true, () => false), false);
  } finally {
    await fs.rm(directory, { recursive: true, force: true });
  }
});

test("generated scripts parse and propagate activation failure", () => {
  const setup = pixiSetupScript("/missing/pixi", "/tmp/ROS user's/pixi.toml", "jazzy");
  execFileSync("/bin/bash", ["-n"], { input: setup });
  assert.throws(() => execFileSync("/bin/bash", ["-c", "source /dev/stdin"], { input: setup, stdio: "pipe" }));
  const install = macInstallScript("/tmp/pixi", "/tmp/ROS user's", "jazzy", "/tmp/setup.bash");
  execFileSync("/bin/bash", ["-n"], { input: install });
  assert.match(install, /xcrun --show-sdk-path/);
  assert.match(install, /rclpy.create_node/);
  assert.doesNotMatch(install, /sudo|DevToolsSecurity|set -.*u/);
});

test("sources Bash setup without a login shell and preserves multiline environment values", async () => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), "ros user's-"));
  const setup = path.join(directory, "setup.bash");
  try {
    await fs.writeFile(setup, "export ROS_DISTRO=jazzy\nexport MULTILINE='first\nsecond=value'\necho diagnostic\n");
    const env = await sourceBashEnvironment(setup, { PATH: "/usr/bin:/bin", SHELL: "/bin/zsh" });
    assert.equal(env.ROS_DISTRO, "jazzy");
    assert.equal(env.MULTILINE, "first\nsecond=value");
    await fs.writeFile(setup, "return 42\n");
    await assert.rejects(sourceBashEnvironment(setup));
  } finally {
    await fs.rm(directory, { recursive: true, force: true });
  }
});

test("live macOS bootstrap, ROS install and environment smoke test", {
  skip: process.platform !== "darwin" || process.env.RDE_TEST_PIXI_LIVE !== "1",
  timeout: 30 * 60 * 1000,
}, async () => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), "rde-live-"));
  const env = { ...process.env, PIXI_HOME: path.join(directory, "pixi-home"), PIXI_BIN_DIR: path.join(directory, "pixi-home/bin"), PIXI_CACHE_DIR: path.join(directory, "cache"), PIXI_NO_PATH_UPDATE: "1", PATH: "/usr/bin:/bin:/usr/sbin:/sbin" };
  const worker = new Worker(path.resolve(__dirname, "../out/src/ros/installer/install-ros-worker.js"), { env });
  const request = message => new Promise((resolve, reject) => {
    const onError = error => { cleanup(); reject(error); };
    const onMessage = response => {
      if (response.type === "log") { process.stdout.write(response.text); return; }
      cleanup();
      if (response.type === "error") { reject(new Error(response.message)); }
      else { resolve(response); }
    };
    const cleanup = () => { worker.off("error", onError); worker.off("message", onMessage); };
    worker.once("error", onError);
    worker.on("message", onMessage);
    worker.postMessage(message);
  });
  try {
    const before = await request({ type: "check_pixi" });
    assert.ok(!before.executable?.startsWith(directory));
    await request({ type: "install_pixi", platform: "darwin" });
    const detected = await request({ type: "check_pixi" });
    assert.equal(detected.available, true);
    const distro = "jazzy";
    const manifest = path.join(directory, "pixi.toml");
    const setup = path.join(directory, ".setup.bash");
    await fs.writeFile(manifest, pixiManifest(distro, pixiPlatform(), await macOSVersion()));
    await fs.writeFile(setup, pixiSetupScript(detected.executable, manifest, distro));
    const script = path.join(directory, "install.sh");
    await fs.writeFile(script, macInstallScript(detected.executable, directory, distro, setup));
    const installation = promisify(execFile)("/bin/bash", [script], { env, timeout: 25 * 60 * 1000, maxBuffer: 10 * 1024 * 1024 });
    installation.child.stdout.on("data", data => process.stdout.write(data));
    installation.child.stderr.on("data", data => process.stderr.write(data));
    await installation;
    const sourced = await sourceBashEnvironment(path.join(directory, "setup.bash"), env);
    assert.equal(sourced.ROS_DISTRO, distro);
    assert.ok(sourced.CONDA_PREFIX.startsWith(await fs.realpath(directory)));
  } finally {
    await worker.terminate();
    await fs.rm(directory, { recursive: true, force: true });
  }
});

test("includes the per-user WinGet MSI install path in Windows Pixi discovery", () => {
  const localAppData = path.join(os.tmpdir(), "pixi-test-local-app-data");
  const candidates = pixiExecutableCandidates("win32", os.tmpdir(), { LOCALAPPDATA: localAppData, PATH: "" }, []);
  assert.ok(candidates.includes(path.join(localAppData, "pixi", "bin", "pixi.exe")));
});