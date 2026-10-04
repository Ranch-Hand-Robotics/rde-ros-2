// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as fs from "fs";
import * as path from "path";
import {
  buildHealthCommand, HEALTH_SENTINEL, HealthTarget, parseHealthOutput,
  runHealthProcess, validateInstallation,
} from "../../src/ros/installer/health-check";

const ids = ["environment", "prefix", "ros2-cli", "package-discovery", "rclpy-import", "rmw-init", "pub-sub"];
const payload = (distro = "jazzy") => ({
  version: 1, distro,
  checks: ids.map((id) => ({ id, status: "passed", detail: `${id} verified`, durationMs: 1 })),
});
const structured = (value = payload()) => HEALTH_SENTINEL + JSON.stringify(value);
const quote = (value: string) => "'" + value.replace(/'/g, "'\\''") + "'";
const unix = process.platform === "win32" ? it.skip : it;

describe("ROS installation health checking", () => {
  let directory: string;
  let target: HealthTarget;
  let probe: string;

  beforeEach(async () => {
    directory = path.join(process.cwd(), "test", `.health-fixtures-${process.pid}-${Math.random().toString(16).slice(2)} with spaces`);
    await fs.promises.mkdir(directory);
    probe = path.join(directory, "probe with spaces.py");
    await fs.promises.writeFile(probe, "# test probe\n");
    target = { kind: "setup", distro: "jazzy", setupScript: path.join(directory, "setup.bash") };
    await fs.promises.writeFile(target.setupScript, "# fake setup\n");
  });

  afterEach(async () => {
    await fs.promises.rm(directory, { recursive: true, force: true });
  });

  it("accepts exactly one complete sentinel result despite shell setup chatter", () => {
    assert.strictEqual(parseHealthOutput(`setup chatter\n${structured()}\n`, target).length, 7);
    assert.throws(() => parseHealthOutput("Installation complete!", target), /structured/);
    assert.throws(() => parseHealthOutput(`${structured()}\n${structured()}`, target), /exactly one/);
    assert.throws(() => parseHealthOutput(HEALTH_SENTINEL + "not json", target));
  });

  it("rejects distribution mismatches, missing checks, duplicates and malformed statuses", () => {
    assert.throws(() => parseHealthOutput(structured(payload("humble")), target), /mismatch/);
    const missing = payload();
    missing.checks.pop();
    assert.throws(() => parseHealthOutput(structured(missing), target), /Malformed/);
    const duplicate = payload();
    duplicate.checks[1] = duplicate.checks[0];
    assert.throws(() => parseHealthOutput(structured(duplicate), target), /Malformed/);
    const invalid = payload();
    invalid.checks[0].status = "skipped";
    assert.throws(() => parseHealthOutput(structured(invalid), target), /Malformed/);
    invalid.checks[0].status = "passed";
    invalid.checks[0].durationMs = -1;
    assert.throws(() => parseHealthOutput(structured(invalid), target), /Malformed/);
  });

  unix("uses positional Bash arguments and drops inherited ROS/Python/loader pollution", async () => {
    const command = await buildHealthCommand(target, probe, "linux", {
      HOME: directory, PATH: "/wrong/ros/bin", ROS_DISTRO: "humble", ROS_VERSION: "2",
      PYTHONPATH: "/wrong/python", PYTHONHOME: "/wrong/python", LD_LIBRARY_PATH: "/wrong/lib",
      AMENT_PREFIX_PATH: "/wrong/ros", COLCON_PREFIX_PATH: "/wrong/ros", CONDA_PREFIX: "/wrong/conda",
      BASH_ENV: "/wrong/injection", LD_PRELOAD: "/wrong/lib.so", ROS_DISCOVERY_SERVER: "robot",
    });
    assert.ok(command.args.includes(probe));
    assert.ok(command.args.includes((target as { setupScript: string }).setupScript));
    assert.ok(!command.args[3].includes(directory));
    assert.strictEqual(command.env.HOME, directory);
    assert.strictEqual(command.env.PATH, "/usr/bin:/bin:/usr/sbin:/sbin");
    assert.strictEqual(command.env.PYTHONNOUSERSITE, "1");
    for (const key of ["ROS_DISTRO", "ROS_VERSION", "PYTHONPATH", "PYTHONHOME",
      "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CONDA_PREFIX",
      "BASH_ENV", "LD_PRELOAD", "ROS_DISCOVERY_SERVER"]) {
      assert.strictEqual(command.env[key], undefined, key);
    }
  });

  it("returns an explicit activation failure for missing setup scripts and manifests", async () => {
    const missingSetup = await validateInstallation({
      kind: "setup", distro: "jazzy", setupScript: path.join(directory, "missing.bash"),
    }, probe);
    const missingManifest = await validateInstallation({ kind: "pixi", distro: "jazzy", workspace: directory }, probe);
    for (const report of [missingSetup, missingManifest]) {
      assert.strictEqual(report.healthy, false);
      assert.strictEqual(report.checks[0].id, "activation");
      assert.match(report.checks[0].detail, /ENOENT/);
    }
  });

  unix("runs Pixi read-only with explicit manifest, environment, executable and no system-Python fallback", async () => {
    const bin = path.join(directory, "bin");
    const python = path.join(directory, ".pixi", "envs", "jazzy", "bin", "python");
    await fs.promises.mkdir(path.dirname(python), { recursive: true });
    await fs.promises.mkdir(bin);
    await fs.promises.writeFile(python, "");
    await fs.promises.writeFile(path.join(directory, "pixi.toml"), "");
    await fs.promises.writeFile(path.join(directory, "pixi.lock"), "");
    const pixi = path.join(bin, "pixi");
    await fs.promises.writeFile(pixi, "#!/bin/sh\nexit 0\n", { mode: 0o755 });
    const pixiTarget: HealthTarget = { kind: "pixi", distro: "jazzy", workspace: directory };
    const command = await buildHealthCommand(pixiTarget, probe, "linux", { PATH: bin });
    assert.strictEqual(command.command, pixi);
    assert.deepStrictEqual(command.args.slice(0, 11), [
      "run", "--frozen", "--no-install", "--executable", "--manifest-path",
      path.join(directory, "pixi.toml"), "-e", "jazzy", "python", "-s", probe,
    ]);
    await assert.rejects(buildHealthCommand(pixiTarget, probe, "linux", { PATH: "/missing" }), /Pixi executable not found/);
    await fs.promises.unlink(python);
    await assert.rejects(buildHealthCommand(pixiTarget, probe, "linux", { PATH: bin }), /python/);
  });

  it("rejects cmd expansion characters rather than interpolating unsafe Windows paths", async () => {
    const setup = path.join(directory, "setup%injected%.bat");
    const python = path.join(directory, "python.exe");
    await fs.promises.writeFile(setup, "");
    await fs.promises.writeFile(python, "");
    await assert.rejects(buildHealthCommand({ kind: "setup", distro: "jazzy", setupScript: setup },
      probe, "win32", { PATH: directory }), /Unsafe cmd/);
  });

  it("resolves native Windows Python outside the ROS prefix without inheriting an activated environment", async () => {
    const pythonDirectory = path.join(directory, "external python");
    await fs.promises.mkdir(pythonDirectory);
    const python = path.join(pythonDirectory, "python.exe");
    await fs.promises.writeFile(python, "");
    const setup = path.join(directory, "setup.bat");
    await fs.promises.writeFile(setup, "");
    const setupTarget: HealthTarget = { kind: "setup", distro: "jazzy", setupScript: setup };
    const command = await buildHealthCommand(setupTarget, probe, "win32", {
      PATH: pythonDirectory, PYTHONPATH: "another-ros", ROS_DISTRO: "humble",
    });
    assert.ok(command.args[4].includes(`"${python}" -s`));
    assert.strictEqual(command.env.COLCON_PYTHON_EXECUTABLE, python);
    assert.strictEqual(command.env.PYTHONPATH, undefined);
    assert.strictEqual(command.env.ROS_DISTRO, undefined);
    assert.ok(!command.env.PATH.includes(pythonDirectory));
    await assert.rejects(buildHealthCommand(setupTarget, probe, "win32", {}), /compatible Python/);
    const explicit = await buildHealthCommand({ ...setupTarget, pythonExecutable: python }, probe, "win32", {});
    assert.strictEqual(explicit.env.COLCON_PYTHON_EXECUTABLE, python);
  });

  async function fakePython(output: string, exitCode = 0): Promise<void> {
    const bin = path.join(directory, "fake bin");
    await fs.promises.mkdir(bin);
    await fs.promises.writeFile(path.join(bin, "python3"),
      `#!/bin/bash\nprintf '%s\\n' ${quote(output)}\nprintf '%s\\n' 'native diagnostic' >&2\nexit ${exitCode}\n`,
      { mode: 0o755 });
    await fs.promises.writeFile((target as { setupScript: string }).setupScript,
      `export PATH=${quote(bin)}\necho 'setup output'\n`);
  }

  unix("validates an explicit target with spaces in a fresh subprocess", async () => {
    await fakePython(structured());
    const report = await validateInstallation(target, probe);
    assert.strictEqual(report.healthy, true, JSON.stringify(report));
    assert.strictEqual(report.checks.length, 7);
  });

  unix("never turns nonzero exits with passing probe JSON into success", async () => {
    await fakePython(structured(), 23);
    const report = await validateInstallation(target, probe);
    assert.strictEqual(report.healthy, false);
    const processCheck = report.checks.find((check) => check.id === "probe-process");
    assert.match(processCheck.detail, /code 23/);
    assert.match(processCheck.detail, /native diagnostic/);
  });

  unix("reports malformed successful subprocess output as a failed check", async () => {
    await fakePython("all good!");
    const report = await validateInstallation(target, probe);
    assert.strictEqual(report.healthy, false);
    assert.strictEqual(report.checks[0].id, "probe-output");
  });

  unix("preserves valid failed checks instead of trusting the process exit code", async () => {
    const result = payload();
    result.checks[0].status = "failed";
    result.checks[0].detail = "ROS_DISTRO mismatch";
    await fakePython(structured(result));
    const report = await validateInstallation(target, probe);
    assert.strictEqual(report.healthy, false);
    assert.match(report.checks[0].detail, /ROS_DISTRO mismatch/);
  });

  unix("reports Python runtime and setup activation failures", async () => {
    await fs.promises.writeFile((target as { setupScript: string }).setupScript, "export PATH=/missing\n");
    const report = await validateInstallation(target, probe);
    assert.strictEqual(report.healthy, false);
    assert.match(report.checks.find((check) => check.id === "probe-process").detail, /python3.*not found/);
    await fs.promises.writeFile((target as { setupScript: string }).setupScript, "echo 'activation failure' >&2\nreturn 19\n");
    const failedSetup = await validateInstallation(target, probe);
    assert.match(failedSetup.checks.find((check) => check.id === "probe-process").detail, /code 19/);
  });

  unix("runs the real Python probe without importing ROS when the activated distro is wrong", async () => {
    await fs.promises.writeFile((target as { setupScript: string }).setupScript,
      "export ROS_VERSION=2\nexport ROS_DISTRO=humble\n");
    const report = await validateInstallation(target,
      path.join(process.cwd(), "assets", "scripts", "ros_install_health.py"));
    assert.strictEqual(report.healthy, false);
    assert.strictEqual(report.checks[0].id, "environment");
    assert.match(report.checks[0].detail, /Expected ROS_DISTRO=jazzy/);
    assert.match(report.checks.find((check) => check.id === "rmw-init").detail, /prerequisite.*failed/);
    assert.match(report.checks.find((check) => check.id === "probe-process").detail, /code 1/);
  });

  unix("bounds timeout and terminates subprocess descendants", async () => {
    const marker = path.join(directory, "must not exist");
    const result = await runHealthProcess({
      command: "/bin/bash",
      args: ["-c", `(sleep 0.4; echo unexpected > ${quote(marker)}) & wait`],
      cwd: directory, env: { PATH: "/usr/bin:/bin" },
    }, 50);
    assert.match(result.error, /timed out/);
    await new Promise((resolve) => setTimeout(resolve, 500));
    assert.strictEqual(fs.existsSync(marker), false);
  });

  it("bounds excessive output and reports executable launch failures", async () => {
    const result = await runHealthProcess({
      command: process.execPath, args: ["-e", 'process.stdout.write("x".repeat(200000))'],
      cwd: directory, env: { ...process.env, ELECTRON_RUN_AS_NODE: "1" },
    });
    assert.match(result.error, /output exceeded/);
    assert.ok(result.stdout.length <= 128 * 1024);
    const missing = await runHealthProcess({
      command: path.join(directory, "missing-executable"), args: [], cwd: directory, env: {},
    });
    assert.match(missing.error, /ENOENT/);
  });
});
