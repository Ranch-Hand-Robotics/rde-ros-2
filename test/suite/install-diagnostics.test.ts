// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import { execFile } from "child_process";
import { promisify } from "util";
import {
  InstallDiagnostics, InstallStep, bashInstallScript, powershellInstallScript,
  recoveryNotes, redactDiagnostics, updateSteps, installationManifest,
} from "../../src/ros/installer/install-diagnostics";
import { runBootstrapProcess } from "../../src/ros/installer/bootstrap-process";

const execFileAsync = promisify(execFile);

describe("ROS installer diagnostics", () => {
  let directory: string;

  beforeEach(async () => {
    directory = await fs.promises.mkdtemp(path.join(os.tmpdir(), "rde-install-diagnostics-"));
  });

  afterEach(async () => {
    await fs.promises.rm(directory, { recursive: true, force: true });
  });

  it("records failed and unfinished steps without treating log prose as success", () => {
    const steps: InstallStep[] = [];
    for (const line of [
      "RDE_STEP_START:preflight", "RDE_STEP_OK:preflight",
      "RDE_STEP_START:packages", "RDE_STEP_FAILED:packages:42",
      "Installation complete!", "prefix RDE_STEP_OK:packages",
    ]) {
      updateSteps(steps, line);
    }
    assert.deepStrictEqual(steps, [
      { id: "preflight", status: "passed" },
      { id: "packages", status: "failed", exitCode: 42 },
    ]);
  });

  it("persists reports and logs for reopening after reload", async () => {
    const diagnostics = await InstallDiagnostics.create(directory, {
      kind: "pixi", distro: "jazzy", workspace: path.join(directory, "workspace"),
    }, "install", "wsl");
    await diagnostics.log("RDE_STEP_START:ros-packages\nsolver failure\nRDE_STEP_FAILED:ros-packages:1\n");
    await diagnostics.finish("failed", "Packages failed");
    const restored = await InstallDiagnostics.load(diagnostics.reportPath);
    assert.strictEqual(restored.report.status, "failed");
    assert.strictEqual(restored.report.host.remote, "wsl");
    assert.strictEqual(restored.report.steps[0].status, "failed");
    assert.ok(restored.report.finishedAt);
    assert.match(await restored.readLog(), /solver failure/);
    assert.ok(restored.report.recovery.includes("No automatic rollback was performed."));
  });

  it("keeps an interrupted attempt distinct from a successful installation", async () => {
    const diagnostics = await InstallDiagnostics.create(directory, {
      kind: "setup", distro: "humble", setupScript: "/opt/ros/humble/setup.bash",
    }, "install");
    await diagnostics.log("RDE_STEP_START:ros-packages\n");
    await diagnostics.finish("interrupted", "Terminal closed");
    assert.strictEqual(diagnostics.report.status, "interrupted");
    assert.strictEqual(diagnostics.report.steps[0].status, "running");
    assert.strictEqual(diagnostics.report.health, undefined);
  });

  it("includes an early selected step even when later logs exceed the tail limit", async () => {
    const diagnostics = await InstallDiagnostics.create(directory, {
      kind: "pixi", distro: "jazzy", workspace: directory,
    }, "install");
    await diagnostics.log("RDE_STEP_START:pixi-bootstrap\nbootstrap detail\nRDE_STEP_OK:pixi-bootstrap\n");
    await diagnostics.log("RDE_STEP_START:ros-packages\n" + "later output\n".repeat(2000));
    const prompt = await diagnostics.troubleshootingPrompt("pixi-bootstrap");
    assert.match(prompt, /bootstrap detail/);
    assert.ok(!prompt.includes("later output"));
    assert.ok((await diagnostics.readLog()).length <= 15000);
  });

  it("redacts common secrets from drafts, while preserving the original local evidence", async () => {
    const text = 'https://user:secret@example.org token=abc123 Authorization: Bearer xyz\nhttps://host/path?api_key=private&ok=1\n{"token": "json-secret"}';
    const redacted = redactDiagnostics(text);
    for (const secret of ["user:secret", "abc123", "xyz", "private", "json-secret"]) {
      assert.ok(!redacted.includes(secret));
    }
    assert.match(redacted, /ok=1/);
    const diagnostics = await InstallDiagnostics.create(directory, {
      kind: "pixi", distro: "jazzy", workspace: directory,
    }, "health");
    await diagnostics.log(text);
    assert.ok((await diagnostics.readLog()).includes("abc123"));
    assert.ok(!(await diagnostics.troubleshootingPrompt()).includes("abc123"));
    assert.match(diagnostics.report.recovery[0], /do not repair/);
  });

  it("documents nontransactional package changes instead of promising rollback", () => {
    const apt = recoveryNotes({ kind: "setup", distro: "humble", setupScript: "/opt/ros/humble/setup.bash" }).join("\n");
    const pixi = recoveryNotes({ kind: "pixi", distro: "jazzy", workspace: directory }).join("\n");
    assert.match(apt, /shared dependencies/);
    assert.match(apt, /JetPack/);
    assert.match(pixi, /manifest.*lockfile/);
    assert.match(pixi, /Do not delete a shared workspace/);
  });

  it("omits nonexistent build overlays without removing Pixi environments or package dependencies", async () => {
    const template = await fs.promises.readFile(path.join(__dirname, "../../../assets/ros/robostack.toml"), "utf8");
    const manifest = installationManifest(template, "jazzy");
    assert.ok(!manifest.includes("[target.win.activation]"));
    assert.ok(!manifest.includes("[target.unix.activation]"));
    assert.ok(!manifest.includes('scripts = ["install/setup'));
    assert.ok(manifest.includes("[environments]"));
    assert.ok(manifest.includes('ros-jazzy-desktop = "*"'));
    assert.ok(manifest.includes("[feature.build.target.win-64.tasks]"));
    assert.ok(!manifest.includes("[feature.humble]"));
    assert.ok(!manifest.includes("[feature.rolling]"));
    assert.ok(!manifest.includes('humble = { features'));
    assert.strictEqual((manifest.match(/^platforms\s*=/gm) || []).length, 1);
    assert.throws(() => installationManifest(template, "unknown"), /no environment/);
    assert.throws(() => installationManifest("", "jazzy"), /empty/);
  });

  it("solves only the intended Windows or macOS host platform", async () => {
    const template = await fs.promises.readFile(path.join(__dirname, "../../../assets/ros/robostack.toml"), "utf8");
    const platform = Object.getOwnPropertyDescriptor(process, "platform");
    const arch = Object.getOwnPropertyDescriptor(process, "arch");
    try {
      for (const [host, cpu, expected] of [
        ["win32", "x64", "win-64"], ["darwin", "x64", "osx-64"], ["darwin", "arm64", "osx-arm64"],
      ]) {
        Object.defineProperty(process, "platform", { configurable: true, value: host });
        Object.defineProperty(process, "arch", { configurable: true, value: cpu });
        assert.ok(installationManifest(template, "jazzy").includes(`platforms = ["${expected}"]`));
      }
    } finally {
      Object.defineProperty(process, "platform", platform);
      Object.defineProperty(process, "arch", arch);
    }
  });

  (process.platform === "win32" ? it.skip : it)("executes Bash steps fail-fast and preserves the failing exit code", async () => {
    const scriptPath = path.join(directory, "failure script.sh");
    await fs.promises.writeFile(scriptPath, bashInstallScript([
      { id: "preflight", commands: ["printf 'preflight output\\n'"] },
      { id: "packages", commands: ["bash -c 'exit 23'"] },
      { id: "must-not-run", commands: ["echo incorrect-success"] },
    ]));
    try {
      await execFileAsync("/bin/bash", [scriptPath]);
      assert.fail("Script should fail");
    } catch (error) {
      assert.strictEqual(error.code, 23);
      assert.match(error.stdout, /RDE_STEP_OK:preflight/);
      assert.match(error.stdout, /RDE_STEP_FAILED:packages:23/);
      assert.ok(!error.stdout.includes("incorrect-success"));
    }
  });

  (process.platform === "win32" ? it.skip : it)("does not hide failures behind a successful pipeline stage", async () => {
    const script = bashInstallScript([{ id: "download", commands: ["false | true"] }]);
    await assert.rejects(execFileAsync("/bin/bash", ["-c", script]), (error: Error & { stdout?: string }) =>
      /RDE_STEP_FAILED:download:1/.test(error.stdout ?? "")
    );
  });

  (process.platform === "win32" ? it.skip : it)("terminates bootstrap descendants before reporting timeout", async () => {
    const marker = path.join(directory, "unexpected-completion");
    const quotedMarker = "'" + marker.replace(/'/g, "'\\''") + "'";
    await assert.rejects(runBootstrapProcess(
      "/bin/bash", ["-c", `(sleep 1; printf escaped > ${quotedMarker}) & wait`], () => undefined, 100
    ), /process tree was terminated/);
    await new Promise((resolve) => setTimeout(resolve, 1200));
    assert.strictEqual(fs.existsSync(marker), false, "A descendant must not continue after the timeout");
  });

  (process.platform === "win32" ? it.skip : it)("records native bootstrap errors and streams both output channels", async () => {
    let log = "";
    await assert.rejects(runBootstrapProcess(
      "/bin/bash", ["-c", "echo stdout; echo stderr >&2; exit 23"], (text) => { log += text; }
    ), /code 23/);
    assert.match(log, /stdout/);
    assert.match(log, /stderr/);
  });

  it("generates an exiting PowerShell script with literal log paths and native exit checks", () => {
    const script = powershellInstallScript([{
      id: "ros-packages",
      commands: ["& pixi install -e jazzy", 'if ($LASTEXITCODE -ne 0) { throw "Install failed" }'],
    }], "C:\\test's folder\\install.log");
    assert.match(script, /Start-Transcript -Path 'C:\\test''s folder\\install.log'/);
    assert.match(script, /RDE_STEP_START:ros-packages/);
    assert.match(script, /RDE_STEP_FAILED/);
    assert.match(script, /Stop-Transcript/);
    assert.ok(script.endsWith("exit $code\r\n"));
  });

  (process.platform === "win32" ? it : it.skip)("executes PowerShell fail-fast and persists the failing step", async () => {
    const logPath = path.join(directory, "installation.log");
    const scriptPath = path.join(directory, "installation.ps1");
    await fs.promises.writeFile(scriptPath, powershellInstallScript([
      { id: "preflight", commands: ["Write-Output 'preflight output'"] },
      { id: "packages", commands: ['throw "test package failure"'] },
      { id: "must-not-run", commands: ["Write-Output 'incorrect-success'"] },
    ], logPath));
    await assert.rejects(execFileAsync("powershell.exe", [
      "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", scriptPath,
    ], { timeout: 15000 }));
    const log = await fs.promises.readFile(logPath, "utf8");
    assert.match(log, /RDE_STEP_OK:preflight/);
    assert.match(log, /RDE_STEP_FAILED:packages:1/);
    assert.ok(!log.includes("incorrect-success"));
  });
});
