// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import * as vscode from "vscode";
import * as extension from "../../src/extension";
import * as health from "../../src/ros/installer/health-check";
import * as preflight from "../../src/ros/installer/install-preflight";
import { installRos, checkRosInstallation, runInstallTerminal, preflightPixiEnvironment, createPixiTarget, ROS2_DISTROS } from "../../src/ros/installer/install-ros";
import { InstallDiagnostics } from "../../src/ros/installer/install-diagnostics";

// Real orchestration with a nonexecuting terminal and deterministic health results.
// No package-manager command is run by these tests.
describe("ROS installation completion and validation", () => {
  let directory: string;
  let restore: (() => void)[];
  let terminalExit: number | undefined;
  let healthPasses: boolean;
  let healthCalls: number;
  let reportPath: string;
  let errorMessages: string[];
  let infoMessages: string[];
  let terminalCount: number;
  let expectedShell: string;

  function replace(object: object, key: string, value: unknown): void {
    const descriptor = Object.getOwnPropertyDescriptor(object, key);
    Object.defineProperty(object, key, { configurable: true, writable: true, value });
    restore.push(() => {
      if (descriptor) {
        Object.defineProperty(object, key, descriptor);
      } else {
        Reflect.deleteProperty(object, key);
      }
    });

  }

  beforeEach(async () => {
    directory = await fs.promises.mkdtemp(path.join(os.tmpdir(), "rde-install-flow-"));
    restore = [];
    terminalExit = 0;
    healthPasses = true;
    healthCalls = 0;
    errorMessages = [];
    infoMessages = [];
    terminalCount = 0;
    expectedShell = "/bin/bash";
    replace(process, "platform", "linux");
    replace(vscode.workspace, "isTrusted", true);
    replace(extension, "outputChannel", { appendLine: () => undefined, show: () => undefined });
    replace(extension, "extPath", path.resolve(__dirname, "../../.."));
    replace(extension, "rosDistributionsProvider", null);
    replace(preflight, "preflightInstallation", async () => ({ ready: true, checks: [] }));
    replace(extension, "extensionContext", {
      globalStorageUri: vscode.Uri.file(directory),
      globalState: {
        update: async (_key: string, value: string) => { reportPath = value; },
      },
    });
    replace(vscode.window, "showQuickPick", async () => ({ distro: ROS2_DISTROS.find((d) => d.name === "jazzy") }));
    replace(vscode.window, "showInformationMessage", async (message: string) => { infoMessages.push(message); });
    replace(vscode.window, "showErrorMessage", async (message: string) => { errorMessages.push(message); });
    replace(vscode.window, "showWarningMessage", async () => undefined);
    replace(vscode.window, "withProgress", async (_options: unknown, task: () => Promise<unknown>) => task());
    let close: ((terminal: unknown) => void) | undefined;
    replace(vscode.window, "onDidCloseTerminal", (listener: typeof close) => {
      close = listener;
      return { dispose: () => { close = undefined; } };
    });
    replace(vscode.window, "createTerminal", (options: vscode.TerminalOptions) => {
      terminalCount++;
      assert.strictEqual(options.shellPath, expectedShell);
      assert.ok(Array.isArray(options.shellArgs));
      const terminal = {
        exitStatus: terminalExit === undefined ? undefined : { code: terminalExit },
        show: () => {
          assert.ok(close, "Must subscribe before showing/running the terminal");
          close(terminal);
        },
      };
      return terminal;
    });
    replace(health, "validateInstallation", async (target: health.HealthTarget): Promise<health.HealthReport> => {
      healthCalls++;
      return {
        target, healthy: healthPasses,
        checks: [{ id: "runtime", status: healthPasses ? "passed" : "failed", detail: "test result", durationMs: 1 }],
      };
    });
  });

  afterEach(async () => {
    restore.reverse().forEach((undo) => undo());
    await fs.promises.rm(directory, { recursive: true, force: true });
  });

  it("requires a successful runtime check after package-manager success", async () => {
    await installRos();
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(healthCalls, 1);
    assert.strictEqual(report.target.setupScript, "/opt/ros/jazzy/setup.bash");
    assert.strictEqual(report.status, "passed");
    assert.strictEqual(report.health.healthy, true);
    assert.strictEqual(errorMessages.length, 0);
    assert.match(infoMessages[0], /runtime validation passed/);
  });

  it("aborts before terminal execution when preflight finds a repair blocker", async () => {
    replace(preflight, "preflightInstallation", async () => ({
      ready: false,
      checks: [{ id: "dpkg-audit", status: "blocked", detail: "Interrupted packages", remediation: "Repair manually" }],
    }));
    await installRos();
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "blocked");
    assert.strictEqual(report.preflight.ready, false);
    assert.strictEqual(report.artifacts.script, undefined);
    assert.strictEqual(terminalCount, 0);
    assert.strictEqual(healthCalls, 0);
    assert.match(errorMessages[0], /aborted by preflight/);
    assert.match(report.recovery[0], /before Pixi bootstrap/);
    replace(preflight, "preflightInstallation", async () => ({ ready: true, checks: [] }));
    await installRos();
    assert.strictEqual(terminalCount, 1, "A repaired system must be rescanned and permitted on retry");
  });

  it("does not continue when preflight warnings are not accepted", async () => {
    replace(preflight, "preflightInstallation", async () => ({
      ready: true,
      checks: [{ id: "held-packages", status: "warning", detail: "Intentional package holds" }],
    }));
    await installRos();
    assert.strictEqual(terminalCount, 0);
    assert.strictEqual(healthCalls, 0);
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "blocked");
  });

  it("keeps Pixi solver failures in diagnostic staging without creating the ROS target", async () => {
    const workspace = path.join(directory, "not-created");
    const diagnostics = await InstallDiagnostics.create(directory, {
      kind: "pixi", distro: "jazzy", workspace,
    }, "install");
    diagnostics.report.preflight = { ready: true, checks: [] };
    replace(preflight, "runPreflightCommand", async (command: string, args: string[]) => {
      assert.strictEqual(command, "pixi");
      assert.strictEqual(args[0], "lock");
      assert.ok(args[2].startsWith(diagnostics.directory));
      return { exitCode: 1, stdout: "", stderr: "Unsupported virtual package" };
    });
    await assert.rejects(preflightPixiEnvironment(ROS2_DISTROS.find((d) => d.name === "jazzy"), diagnostics), /dependency preflight failed/);
    assert.strictEqual(fs.existsSync(workspace), false);
    assert.strictEqual(diagnostics.report.preflight.ready, false);
    assert.strictEqual(diagnostics.report.status, "blocked");
  });

  it("refuses a target replaced with a directory link after preflight", async () => {
    const linkedDirectory = path.join(directory, "existing-directory");
    const workspace = path.join(directory, "target");
    await fs.promises.mkdir(linkedDirectory);
    await fs.promises.symlink(linkedDirectory, workspace, "junction");
    await assert.rejects(createPixiTarget(path.join(directory, "unused-manifest"), workspace), /non-symlink directory/);
    assert.deepStrictEqual(await fs.promises.readdir(linkedDirectory), []);
  });

  it("copies only a successfully staged manifest and lockfile into a fresh target", async () => {
    const staging = path.join(directory, "staging");
    const workspace = path.join(directory, "fresh-target");
    await fs.promises.mkdir(staging);
    const stagedManifest = path.join(staging, "pixi.toml");
    await fs.promises.writeFile(stagedManifest, "selected manifest");
    await fs.promises.writeFile(path.join(staging, "pixi.lock"), "resolved lock");
    const manifest = await createPixiTarget(stagedManifest, workspace);
    assert.strictEqual(await fs.promises.readFile(manifest, "utf8"), "selected manifest");
    assert.strictEqual(await fs.promises.readFile(path.join(workspace, "pixi.lock"), "utf8"), "resolved lock");
    await assert.rejects(createPixiTarget(stagedManifest, workspace), /empty, non-symlink/);
    assert.strictEqual(await fs.promises.readFile(manifest, "utf8"), "selected manifest");
  });

  it("does not announce success when package installation succeeds but runtime validation fails", async () => {
    healthPasses = false;
    await installRos();
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "failed");
    assert.strictEqual(report.exitCode, 0);
    assert.strictEqual(report.health.healthy, false);
    assert.strictEqual(infoMessages.length, 0);
    assert.match(errorMessages[0], /runtime validation failed/);
  });

  it("does not validate a package-manager failure and permits a later retry", async () => {
    terminalExit = 23;
    await installRos();
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "failed");
    assert.strictEqual(report.exitCode, 23);
    assert.strictEqual(healthCalls, 0);
    terminalExit = 0;
    await installRos();
    assert.strictEqual(healthCalls, 1);
  });

  it("records terminal interruption without treating it as success", async () => {
    terminalExit = undefined;
    await installRos();
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "interrupted");
    assert.strictEqual(healthCalls, 0);
    assert.strictEqual(infoMessages.length, 0);
  });

  it("returns the same structured health result from the noninteractive entrypoint", async () => {
    const result = await checkRosInstallation({
      kind: "pixi", distro: "jazzy", workspace: directory,
    });
    assert.strictEqual(result.healthy, true);
    assert.strictEqual(result.target.kind, "pixi");
    assert.strictEqual(healthCalls, 1);
    assert.strictEqual(infoMessages.length, 0);
    assert.strictEqual(errorMessages.length, 0);
  });

  it("prevents installation or a second health check from overlapping a pending health check", async () => {
    let signalStarted: () => void;
    let release: (report: health.HealthReport) => void;
    const started = new Promise<void>((resolve) => { signalStarted = resolve; });
    const validation = new Promise<health.HealthReport>((resolve) => { release = resolve; });
    const target: health.HealthTarget = { kind: "pixi", distro: "jazzy", workspace: directory };
    replace(health, "validateInstallation", () => {
      signalStarted();
      return validation;
    });
    const pending = checkRosInstallation(target);
    await started;
    try {
      await installRos();
      assert.strictEqual(terminalCount, 0);
      await assert.rejects(checkRosInstallation(target), /active ROS installation or health check/);
    } finally {
      release({ target, healthy: true, checks: [] });
      await pending;
    }
  });

  it("writes Windows PowerShell scripts with a BOM and runs a directly exiting process", async () => {
    expectedShell = "powershell.exe";
    const diagnostics = await InstallDiagnostics.create(directory, {
      kind: "pixi", distro: "jazzy", workspace: directory,
    }, "install");
    const distro = ROS2_DISTROS.find((entry) => entry.name === "jazzy");
    assert.strictEqual(await runInstallTerminal(distro, diagnostics, "exit 0\r\n", true), 0);
    const script = await fs.promises.readFile(diagnostics.report.artifacts.script);
    assert.deepStrictEqual([...script.subarray(0, 3)], [0xef, 0xbb, 0xbf]);
  });
});

describe("Registered ROS installation health command", () => {
  it("returns a structured failure from the packaged command for an absent installation", async () => {
    const result = await vscode.commands.executeCommand<health.HealthReport>("ROS2.checkInstallation", {
      kind: "setup", distro: "jazzy",
      setupScript: path.join(os.tmpdir(), `rde-missing-${Date.now()}`, "setup.bash"),
    });
    assert.ok(result, "The command must return its health report, not swallow the callback result");
    assert.strictEqual(result.healthy, false);
    assert.strictEqual(result.checks[0].id, "activation");
  });
});
