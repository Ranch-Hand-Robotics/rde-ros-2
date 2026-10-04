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
import * as vscodeUtils from "../../src/vscode-utils";
import * as rosUtils from "../../src/ros/utils";
import * as rosBuildUtils from "../../src/ros/build-env-utils";
import * as ros from "../../src/ros/ros";
import * as buildTool from "../../src/build-tool/build-tool";
import * as rosShell from "../../src/build-tool/ros-shell";
import * as debugManager from "../../src/debugger/manager";
import * as installer from "../../src/ros/installer/install-ros";
import * as pixiLocation from "../../src/ros/installer/pixi-location";
import { installRos, checkRosInstallation, runInstallTerminal, preflightPixiEnvironment, createPixiTarget, confirmPixiBootstrap, ROS2_DISTROS } from "../../src/ros/installer/install-ros";
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
  let generatedScript: Buffer;
  let configurationUpdates: unknown[][];

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
    configurationUpdates = [];
    replace(process, "platform", "linux");
    replace(vscode.workspace, "isTrusted", true);
    replace(extension, "outputChannel", { appendLine: () => undefined, show: () => undefined });
    replace(extension, "extPath", path.resolve(__dirname, "../../.."));
    replace(extension, "rosDistributionsProvider", null);
    replace(extension, "activateEnvironment", async (_context: vscode.ExtensionContext) => {});
    replace(vscodeUtils, "getExtensionConfiguration", () => ({
      update: async (...args: unknown[]) => { configurationUpdates.push(args); },
    }));
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
    let ended: ((event: vscode.TaskProcessEndEvent) => void) | undefined;
    replace(vscode.tasks, "onDidEndTaskProcess", (listener: typeof ended) => {
      ended = listener;
      return { dispose: () => { ended = undefined; } };
    });
    replace(vscode.tasks, "onDidEndTask", () => ({ dispose: () => {} }));
    replace(vscode.tasks, "executeTask", async (task: vscode.Task) => {
      terminalCount++;
      const options = (task.execution as vscode.ShellExecution).options;
      assert.strictEqual(options.executable, expectedShell);
      assert.ok(Array.isArray(options.shellArgs));
      assert.strictEqual(task.presentationOptions.close, false);
      if (!vscode.workspace.workspaceFolders?.length) {
        assert.strictEqual(task.scope, vscode.TaskScope.Global);
        assert.strictEqual(options.cwd, os.homedir());
      }
      const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
      generatedScript = await fs.promises.readFile(report.artifacts.script);
      const execution = { task, terminate: () => {} };
      assert.ok(ended, "Must subscribe before starting the task");
      ended({ execution, exitCode: terminalExit });
      return execution;
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

  it("continues through preflight warnings and records them in the install log", async () => {
    replace(preflight, "preflightInstallation", async () => ({
      ready: true,
      checks: [{ id: "held-packages", status: "warning", detail: "Intentional package holds", remediation: "Review held packages." }],
    }));
    await installRos();
    assert.strictEqual(terminalCount, 1);
    assert.strictEqual(healthCalls, 1);
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "passed");
    assert.strictEqual(report.preflight.checks[0].status, "warning");
    const log = await fs.promises.readFile(report.artifacts.log, "utf8");
    assert.match(log, /held-packages: warning: Intentional package holds/);
    assert.match(log, /Action: Review held packages\./);
  });

  it("lets the user stop when recommended Windows settings are disabled", async () => {
    replace(preflight, "preflightInstallation", async () => ({
      ready: true,
      checks: [
        { id: "windows-developer-mode", status: "warning", detail: "Developer Mode is disabled." },
        { id: "windows-long-paths", status: "warning", detail: "Long paths are disabled." },
      ],
    }));
    let prompt: unknown[] = [];
    replace(vscode.window, "showWarningMessage", async (...args: unknown[]) => {
      prompt = args;
      return "Stop";
    });

    await installRos();
    assert.deepStrictEqual(prompt.slice(2), ["Continue Now", "Stop"]);
    assert.strictEqual(terminalCount, 0);
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "blocked");
    assert.match(await fs.promises.readFile(report.artifacts.log, "utf8"), /Windows settings recommendation choice: Stop/);
  });

  it("continues when the user accepts the Windows settings recommendation", async () => {
    replace(preflight, "preflightInstallation", async () => ({
      ready: true,
      checks: [{ id: "windows-developer-mode", status: "warning", detail: "Developer Mode is disabled." }],
    }));
    let prompt: unknown[] = [];
    replace(vscode.window, "showWarningMessage", async (...args: unknown[]) => {
      prompt = args;
      return "Continue Now";
    });

    await installRos();
    assert.deepStrictEqual(prompt.slice(2), ["Continue Now", "Stop"]);
    assert.strictEqual(terminalCount, 1);
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "passed");
    assert.match(await fs.promises.readFile(report.artifacts.log, "utf8"), /Windows settings recommendation choice: Continue Now/);
  });

  it("asks for a Pixi install folder and uses and caches the selected root", async () => {
    replace(process, "platform", "win32");
    const selectedRoot = path.join(directory, "custom-pixi-root");
    let dialog: vscode.OpenDialogOptions | undefined;
    let cachedRoot: string | undefined;
    replace(vscode.window, "showOpenDialog", async (options: vscode.OpenDialogOptions) => {
      dialog = options;
      return [vscode.Uri.file(selectedRoot)];
    });
    replace(pixiLocation, "cachePixiInstallRoot", async (root: string) => { cachedRoot = root; });
    replace(preflight, "preflightInstallation", async target => {
      assert.strictEqual(target.kind, "pixi");
      assert.strictEqual(target.workspace, path.join(selectedRoot, "jazzy"));
      return { ready: false, checks: [{ id: "test-stop", status: "blocked", detail: "Stop before install." }] };
    });

    await installRos();
    assert.strictEqual(dialog?.canSelectFolders, true);
    assert.strictEqual(dialog?.canSelectFiles, false);
    assert.match(dialog?.title ?? "", /ROS 2 jazzy/);
    assert.strictEqual(path.normalize(cachedRoot ?? "").toLowerCase(), path.normalize(selectedRoot).toLowerCase());
    assert.strictEqual(terminalCount, 0);
  });

  it("cancels before preflight when no Pixi install folder is selected", async () => {
    replace(process, "platform", "win32");
    reportPath = "not-created";
    let preflightCalled = false;
    replace(vscode.window, "showOpenDialog", async () => undefined);
    replace(preflight, "preflightInstallation", async () => {
      preflightCalled = true;
      return { ready: true, checks: [] };
    });

    await installRos();
    assert.strictEqual(preflightCalled, false);
    assert.strictEqual(reportPath, "not-created");
    assert.strictEqual(terminalCount, 0);
  });

  it("asks before removing a recognized incomplete Pixi target and leaves it intact on Stop", async () => {
    replace(process, "platform", "win32");
    replace(vscode.window, "showOpenDialog", async () => [vscode.Uri.file(directory)]);
    replace(pixiLocation, "cachePixiInstallRoot", async () => {});
    replace(vscodeUtils, "getExtensionConfiguration", () => ({
      inspect: () => ({ globalValue: directory }),
      update: async (...args: unknown[]) => { configurationUpdates.push(args); },
    }));
    replace(preflight, "preflightInstallation", async () => ({
      ready: true,
      checks: [{ id: "target", status: "warning", detail: "The installation target appears incomplete." }],
    }));
    const workspace = path.join(directory, "jazzy");
    await fs.promises.mkdir(workspace);
    let prompt: unknown[] = [];
    replace(vscode.window, "showWarningMessage", async (...args: unknown[]) => {
      prompt = args;
      return "Stop";
    });

    await installRos();
    assert.deepStrictEqual(prompt.slice(2), ["Remove and Start Over", "Stop"]);
    assert.match(prompt[0] as string, /appears to be incomplete/);
    assert.strictEqual(fs.existsSync(workspace), true);
    assert.strictEqual(terminalCount, 0);
  });

  it("installs ROS in an empty window using a global task rooted at the user home", async () => {
    replace(vscode.workspace, "workspaceFolders", undefined);
    replace(vscode.workspace, "workspaceFile", undefined);
    await installRos();
    const report = JSON.parse(await fs.promises.readFile(reportPath, "utf8"));
    assert.strictEqual(report.status, "passed");
    assert.strictEqual(terminalCount, 1);
    assert.strictEqual(healthCalls, 1);
    assert.deepStrictEqual(configurationUpdates.map(update => update[2]), [
      vscode.ConfigurationTarget.Global, vscode.ConfigurationTarget.Global,
    ]);
  });

  it("offers to open Prefix.dev from the Pixi installation prompt and asks for consent again", async () => {
    replace(process, "platform", "win32");
    const promptCalls: unknown[][] = [];
    const choices = ["Visit Prefix.dev", "Yes"];
    let openedUrl: string | undefined;
    replace(vscode.window, "showWarningMessage", async (...args: unknown[]) => {
      promptCalls.push(args);
      return choices.shift();
    });
    replace(vscode.env, "openExternal", async (uri: vscode.Uri) => {
      openedUrl = uri.toString();
      return true;
    });

    assert.strictEqual(await confirmPixiBootstrap(), true);
    assert.strictEqual(openedUrl, "https://prefix.dev/");
    assert.strictEqual(promptCalls.length, 2);
    assert.match(promptCalls[0][0] as string, /Pixi, the package manager by Prefix\.dev/);
    assert.deepStrictEqual(promptCalls[0].slice(2), ["Yes", "No", "Visit Prefix.dev"]);
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
    reportPath = diagnostics.reportPath;
    const distro = ROS2_DISTROS.find((entry) => entry.name === "jazzy");
    assert.strictEqual(await runInstallTerminal(distro, diagnostics, "exit 0\r\n", true), 0);
    assert.deepStrictEqual([...generatedScript.subarray(0, 3)], [0xef, 0xbb, 0xbf]);
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
