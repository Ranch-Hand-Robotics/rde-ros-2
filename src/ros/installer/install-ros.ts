// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as os from "os";
import { Worker } from "worker_threads";

import * as vscode_utils from "../../vscode-utils";
import * as extension from "../../extension";
import type { WorkerRequest, WorkerResponse } from "./install-ros-worker";
import { HealthReport, HealthTarget, validateInstallation } from "./health-check";
import { InstallDiagnostics, bashInstallScript, powershellInstallScript, installationManifest, ScriptStep } from "./install-diagnostics";
import { preflightInstallation, runPreflightCommand } from "./install-preflight";
import { PreflightCheck } from "./preflight-types";

/**
 * ROS 2 distribution information
 */
export interface RosDistro {
  name: string;
  displayName: string;
  isLTS: boolean;
  releaseDate: string;
  eolDate?: string;
  supportedPlatforms: string[];
}

/**
 * Supported ROS 2 distributions
 */
export const ROS2_DISTROS: RosDistro[] = [
  {
    name: "rolling",
    displayName: "Rolling Ridley",
    isLTS: false,
    releaseDate: "Continuous",
    supportedPlatforms: ["linux", "win32", "darwin"],
  },
  {
    name: "lyrical",
    displayName: "Lyrical Luth",
    isLTS: false,
    releaseDate: "May 2026",
    supportedPlatforms: ["linux", "win32", "darwin"],
  },
  {
    name: "kilted",
    displayName: "Kilted Kaiju",
    isLTS: false,
    releaseDate: "May 2024",
    supportedPlatforms: ["linux", "win32", "darwin"],
  },
  {
    name: "jazzy",
    displayName: "Jazzy Jalisco",
    isLTS: true,
    releaseDate: "May 2024",
    supportedPlatforms: ["linux", "win32", "darwin"],
  },
  {
    name: "humble",
    displayName: "Humble Hawksbill",
    isLTS: true,
    releaseDate: "May 2022",
    supportedPlatforms: ["linux", "win32", "darwin"],
  },
];

/**
 * Workspace setting key for ROS installation preference
 */
const INSTALL_ROS_PREFERENCE_KEY = "neverInstallRos";

// ---------------------------------------------------------------------------
// Installation state machine
// ---------------------------------------------------------------------------

/** Tracks the lifecycle of a ROS 2 installation attempt. */
type InstallationState = "idle" | "installing" | "complete" | "failed";

/**
 * Singleton that gates concurrent install attempts and tracks installation
 * state so the UI can accurately reflect what is happening.
 *
 * State transitions:
 *   idle ──► installing ──► complete
 *                       └──► failed
 *   complete/failed ──► installing  (user retries)
 */
class InstallationManager {
  private static _instance: InstallationManager;
  private _state: InstallationState = "idle";

  private constructor() {}

  static getInstance(): InstallationManager {
    if (!InstallationManager._instance) {
      InstallationManager._instance = new InstallationManager();
    }
    return InstallationManager._instance;
  }

  get state(): InstallationState {
    return this._state;
  }

  get isInstalling(): boolean {
    return this._state === "installing";
  }

  /**
   * Begin an installation. Rejects if one is already in progress.
   * The caller marks completion only after the terminal and runtime validation finish.
   */
  begin(): boolean {
    if (this._state === "installing") {
      return false;
    }
    this._state = "installing";
    return true;
  }

  markComplete(): void {
    this._state = "complete";
  }

  markFailed(): void {
    this._state = "failed";
  }
}

// ---------------------------------------------------------------------------
// Worker wrapper
// ---------------------------------------------------------------------------

/**
 * Thin wrapper around a Node.js Worker thread that exposes typed promise-based
 * methods for the two subprocess operations (pixi detection and pixi install).
 * All VS Code API calls remain on the extension host main thread.
 */
class RosInstallWorker {
  private readonly _worker: Worker;
  private _failure: Error | undefined;

  constructor() {
    // The worker bundle is placed beside extension.js in the dist/ folder.
    const workerPath = path.join(extension.extPath, "dist", "install-ros-worker.js");
    this._worker = new Worker(workerPath);

    // Propagate unhandled worker errors to the extension output channel.
    this._worker.on("error", (err) => {
      this._failure = err;
      extension.outputChannel.appendLine(`[install-ros-worker] Unhandled error: ${err.message}`);
    });
    this._worker.on("exit", (code) => {
      this._failure = this._failure ?? new Error(`Installer worker exited (code ${code}).`);
    });
  }

  /** Returns true when pixi is found on PATH. */
  checkPixi(): Promise<boolean> {
    return this.request({ type: "check_pixi" }).then((response) =>
      response.type === "pixi_available" && response.available
    );
  }

  /**
   * Installs Pixi on the given platform.
   * Streams stdout/stderr via `onLog` while running.
   */
  installPixi(platform: string, onLog: (text: string) => void): Promise<void> {
    return this.request({ type: "install_pixi", platform }, onLog).then(() => undefined);
  }

  private request(request: WorkerRequest, onLog?: (text: string) => void): Promise<WorkerResponse> {
    if (this._failure) {
      return Promise.reject(this._failure);
    }
    return new Promise((resolve, reject) => {
      const cleanup = () => {
        this._worker.off("message", handler);
        this._worker.off("error", failed);
        this._worker.off("exit", exited);
      };
      const failed = (error: Error) => {
        cleanup();
        reject(error);
      };
      const exited = (code: number) => failed(new Error(`Installer worker exited before replying (code ${code}).`));
      const handler = (msg: WorkerResponse) => {
        if (msg.type === "log") {
          onLog?.(msg.text);
        } else if (msg.type === "error") {
          failed(new Error(msg.message));
        } else {
          cleanup();
          resolve(msg);
        }
      };
      this._worker.on("message", handler);
      this._worker.once("error", failed);
      this._worker.once("exit", exited);
      this._send(request);
    });
  }

  terminate(): void {
    this._worker.terminate();
  }

  private _send(req: WorkerRequest): void {
    this._worker.postMessage(req);
  }
}

// ---------------------------------------------------------------------------
// Script helpers
// ---------------------------------------------------------------------------

/**
 * Generates a distro-specific pixi.toml in the given workspace directory from
 * the RoboStack template in assets/ros/robostack.toml.
 */
async function preparePixiManifest(distro: RosDistro, diagnostics: InstallDiagnostics): Promise<string> {
  const templatePath = path.join(extension.extPath, "assets", "ros", "robostack.toml");
  const template = await fs.promises.readFile(templatePath, "utf-8");
  const resolved = installationManifest(template, distro.name);
  const staging = path.join(diagnostics.directory, "pixi-plan");
  await fs.promises.mkdir(staging, { mode: 0o700 });
  const manifestPath = path.join(staging, "pixi.toml");
  await fs.promises.writeFile(manifestPath, resolved, { mode: 0o600, flag: "wx" });
  return manifestPath;
}

export async function preflightPixiEnvironment(distro: RosDistro, diagnostics: InstallDiagnostics): Promise<string> {
  const stagedManifest = await preparePixiManifest(distro, diagnostics);
  await diagnostics.log("RDE_STEP_START:pixi-solver\n");
  const plan = await runPreflightCommand("pixi", ["lock", "--manifest-path", stagedManifest], 120000);
  await diagnostics.log(plan.stdout + "\n" + plan.stderr + "\n");
  let solved = !plan.error && plan.exitCode === 0;
  let error = plan.error || plan.stderr || plan.stdout;
  if (solved) {
    try {
      const lock = await fs.promises.stat(path.join(path.dirname(stagedManifest), "pixi.lock"));
      if (!lock.isFile() || lock.size === 0) {
        throw new Error("Pixi did not produce a nonempty lockfile.");
      }
    } catch (failure) {
      solved = false;
      error = String(failure);
    }
  }
  const check: PreflightCheck = {
    id: "pixi-solver", status: solved ? "passed" : "blocked",
    detail: solved ? "Pixi resolved the requested distro for this host without installing packages."
      : `Pixi could not solve the proposed environment: ${error}`,
    remediation: solved ? undefined : "Resolve the reported channel, platform, network, system-requirement or dependency problem, then retry. The ROS workspace has not been written.",
  };
  diagnostics.report.preflight.checks.push(check);
  diagnostics.report.preflight.ready = solved;
  await diagnostics.log(solved ? "RDE_STEP_OK:pixi-solver\n" : "RDE_STEP_FAILED:pixi-solver:1\n");
  await diagnostics.save();
  if (!solved) {
    diagnostics.report.status = "blocked";
    throw new Error("Installation aborted: Pixi dependency preflight failed. No ROS environment was created; Pixi bootstrap or metadata caches may remain.");
  }
  return stagedManifest;
}

export async function createPixiTarget(stagedManifest: string, workspace: string): Promise<string> {
  await fs.promises.mkdir(workspace, { recursive: true });
  const target = await fs.promises.lstat(workspace);
  if (target.isSymbolicLink() || !target.isDirectory() || (await fs.promises.readdir(workspace)).length > 0) {
    throw new Error("The Pixi target changed after preflight: it must be an empty, non-symlink directory. No existing files were overwritten.");
  }
  const manifestPath = path.join(workspace, "pixi.toml");
  await fs.promises.copyFile(stagedManifest, manifestPath, fs.constants.COPYFILE_EXCL);
  await fs.promises.copyFile(path.join(path.dirname(stagedManifest), "pixi.lock"),
    path.join(workspace, "pixi.lock"), fs.constants.COPYFILE_EXCL);
  return manifestPath;
}

// ---------------------------------------------------------------------------

/**
 * Recursively searches for package.xml files in a directory
 * @param dirPath Directory to search
 * @param maxDepth Maximum depth to search (default: 3)
 * @param currentDepth Current depth in recursion
 */
async function findPackageXml(dirPath: string, maxDepth: number = 3, currentDepth: number = 0): Promise<boolean> {
  if (currentDepth >= maxDepth) {
    return false;
  }

  try {
    const packageXmlPath = path.join(dirPath, "package.xml");
    const exists = await fs.promises.access(packageXmlPath).then(() => true).catch(() => false);
    if (exists) {
      return true;
    }

    // Check subdirectories
    const entries = await fs.promises.readdir(dirPath, { withFileTypes: true });
    for (const entry of entries) {
      if (entry.isDirectory() && !entry.name.startsWith('.')) {
        const found = await findPackageXml(
          path.join(dirPath, entry.name),
          maxDepth,
          currentDepth + 1
        );
        if (found) {
          return true;
        }
      }
    }
  } catch (err) {
    // Ignore errors (permission denied, etc.)
  }

  return false;
}

/**
 * Checks if the current workspace is a ROS workspace by looking for package.xml files
 */
export async function isRosWorkspace(): Promise<boolean> {
  const workspaceFolders = vscode.workspace.workspaceFolders;
  if (!workspaceFolders || workspaceFolders.length === 0) {
    return false;
  }

  for (const folder of workspaceFolders) {
    // Check root directory
    const packageXmlPath = path.join(folder.uri.fsPath, "package.xml");
    const exists = await fs.promises.access(packageXmlPath).then(() => true).catch(() => false);
    if (exists) {
      return true;
    }

    // Check src directory recursively (up to 3 levels deep)
    const srcPath = path.join(folder.uri.fsPath, "src");
    const srcExists = await fs.promises.access(srcPath).then(() => true).catch(() => false);
    if (srcExists) {
      if (await findPackageXml(srcPath, 3, 0)) {
        return true;
      }
    }
  }

  return false;
}

/**
 * Checks if the user has chosen to never install ROS for this workspace
 */
export function hasUserDeclinedInstallation(): boolean {
  const config = vscode_utils.getExtensionConfiguration();
  return config.get<boolean>(INSTALL_ROS_PREFERENCE_KEY, false);
}

/**
 * Sets the user's preference to never install ROS for this workspace
 */
export async function setNeverInstallRos(value: boolean): Promise<void> {
  const config = vscode_utils.getExtensionConfiguration();
  await config.update(
    INSTALL_ROS_PREFERENCE_KEY,
    value,
    vscode.ConfigurationTarget.Workspace
  );
}

/**
 * Prompts the user to install ROS if not detected in a ROS workspace
 */
export async function promptInstallRosIfNeeded(): Promise<void> {
  // Check if we're in a ROS workspace
  if (!(await isRosWorkspace())) {
    return;
  }

  // Check if ROS is already detected
  if (extension.env?.ROS_DISTRO !== undefined) {
    return;
  }

  // Check if user has declined installation for this workspace
  if (hasUserDeclinedInstallation()) {
    return;
  }

  // Prompt the user
  const choice = await vscode.window.showInformationMessage(
    "ROS 2 is not detected on this system, but this appears to be a ROS workspace. Would you like to install ROS 2?",
    "Yes",
    "No",
    "Never for this workspace"
  );

  if (choice === "Yes") {
    await installRos();
  } else if (choice === "Never for this workspace") {
    await setNeverInstallRos(true);
    vscode.window.showInformationMessage(
      "ROS 2 installation will not be prompted again for this workspace. You can change this in workspace settings."
    );
  }
}

/**
 * Validates that a distro name is safe for use in shell commands
 * @param distro The distro object to validate
 * @returns true if the distro name is valid (lowercase letters only)
 */
function validateDistroName(distro: RosDistro): boolean {
  return /^[a-z]+$/.test(distro.name);
}

/**
 * Main function to install ROS 2.
 * Guards against concurrent invocations via {@link InstallationManager}.
 */
export async function installRos(): Promise<void> {
  if (!vscode.workspace.isTrusted) {
    throw new Error("Trust this workspace before installing ROS.");
  }
  const manager = InstallationManager.getInstance();

  if (!manager.begin()) {
    vscode.window.showWarningMessage(
      "A ROS 2 installation or health check is already in progress. Please wait for it to complete."
    );
    return;
  }

  let diagnostics: InstallDiagnostics | undefined;
  try {
    // Ask user to select a distro
    const distro = await selectRosDistro();
    if (!distro) {
      manager.markFailed();
      return;
    }

    // Validate distro name for security
    if (!validateDistroName(distro)) {
      throw new Error(`Invalid distro name: ${distro.name}`);
    }

    extension.outputChannel.appendLine(`User selected ROS 2 distro: ${distro.name}`);

    let target: HealthTarget;
    if (process.platform === "linux") {
      target = { kind: "setup", distro: distro.name, setupScript: `/opt/ros/${distro.name}/setup.bash` };
    } else if (process.platform === "win32" || process.platform === "darwin") {
      target = { kind: "pixi", distro: distro.name, workspace: path.join(getPixiRoot(), distro.name) };
    } else {
      throw new Error(`ROS 2 installation is not supported on platform: ${process.platform}`);
    }
    diagnostics = await createDiagnostics(target, "install");
    const ready = await runPreflight(diagnostics);
    if (!ready) {
      manager.markFailed();
      return;
    }
    const exitCode = target.kind === "setup"
      ? await installRosLinux(distro, diagnostics)
      : await installRosPixi(distro, diagnostics, target.workspace);
    diagnostics.report.exitCode = exitCode;
    if (exitCode !== 0) {
      if (exitCode === undefined) {
        diagnostics.report.status = "interrupted";
      }
      throw new Error(exitCode === undefined
        ? "Installation was interrupted or the terminal closed without an exit code."
        : `Installer exited with code ${exitCode}.`);
    }
    const health = await runHealthChecks(diagnostics);
    if (!health.healthy) {
      throw new Error("Packages installed, but ROS runtime validation failed. The installation has not been rolled back.");
    }
    await diagnostics.finish("passed");
    manager.markComplete();
    extension.rosDistributionsProvider?.refresh();
    const choice = await vscode.window.showInformationMessage(
      "ROS 2 installation and runtime validation passed. Reload to detect the installation.",
      "Reload Window", "View Report"
    );
    if (choice === "Reload Window") {
      await vscode.commands.executeCommand("workbench.action.reloadWindow");
    } else if (choice === "View Report") {
      await showReport(diagnostics);
    }
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(`Error during ROS installation: ${errorMessage}`);
    manager.markFailed();
    if (diagnostics) {
      await diagnostics.finish(["interrupted", "blocked"].includes(diagnostics.report.status)
        ? diagnostics.report.status : "failed", errorMessage);
      await showFailure(diagnostics, errorMessage);
    } else {
      await vscode.window.showErrorMessage(`Failed to install ROS 2: ${errorMessage}`);
    }
  }
}

/**
 * Prompts user to select a ROS 2 distro
 */
async function selectRosDistro(): Promise<RosDistro | undefined> {
  const supportedDistros = ROS2_DISTROS.filter((distro) =>
    distro.supportedPlatforms.includes(process.platform)
  );

  if (supportedDistros.length === 0) {
    vscode.window.showErrorMessage(
      `ROS 2 installation is not supported on platform: ${process.platform}`
    );
    return undefined;
  }

  const items = supportedDistros.map((distro) => {
    const ltsLabel = distro.isLTS ? " (LTS)" : "";
    const label = `${distro.displayName}${ltsLabel}`;
    const description = `Released: ${distro.releaseDate}`;

    return {
      label,
      description,
      distro,
    };
  });

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: "Select a ROS 2 distribution to install",
    ignoreFocusOut: true,
  });

  return selected?.distro;
}

/**
 * Checks if Pixi is installed on the system via the worker thread.
 */
async function isPixiInstalled(worker: RosInstallWorker): Promise<boolean> {
  return worker.checkPixi();
}

/**
 * Prompts the user and installs Pixi via the worker thread.
 * Streams subprocess output to the extension output channel.
 * Fails explicitly if bootstrap is declined or Pixi remains unavailable.
 */
async function installPixiViaWorker(worker: RosInstallWorker, diagnostics: InstallDiagnostics): Promise<void> {
  const method = process.platform === "win32" ? "Windows Package Manager (winget)" : "the installer from pixi.sh";
  const choice = await vscode.window.showWarningMessage(
    "Pixi package manager is required to install ROS 2 on this platform but is not currently installed. " +
    `This will install Pixi using ${method}. ` +
    (process.platform === "win32" ? "Proceeding accepts the winget source and package agreements. " : "") +
    "Would you like to proceed?",
    { modal: true },
    "Yes",
    "No"
  );

  if (choice !== "Yes") {
    throw new Error("Pixi installation was declined. No ROS installation was started.");
  }

  extension.outputChannel.appendLine("Installing Pixi...");
  extension.outputChannel.show();

  let logWrites = Promise.resolve();
  let logError: Error | undefined;
  await diagnostics.log("RDE_STEP_START:pixi-bootstrap\n");
  try {
    await worker.installPixi(process.platform, (text) => {
      extension.outputChannel.append(text);
      logWrites = logWrites.then(() => diagnostics.log(text)).catch((error: Error) => {
        logError = error;
      });
    });
    await logWrites;
    if (logError) {
      throw logError;
    }
    await diagnostics.log("\nRDE_STEP_OK:pixi-bootstrap\n");
    if (!(await worker.checkPixi())) {
      throw new Error("Pixi bootstrap completed but Pixi is not on the extension host PATH. Restart VS Code and retry.");
    }
  } catch (error) {
    await logWrites;
    await diagnostics.log(`\nRDE_STEP_FAILED:pixi-bootstrap:1\n${String(error)}\n`);
    throw error;
  }
}

/**
 * Installs ROS 2 on Linux using APT
 */
async function installRosLinux(distro: RosDistro, diagnostics: InstallDiagnostics): Promise<number | undefined> {
  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} on Linux using APT...`);
  extension.outputChannel.show();

  const steps: ScriptStep[] = [
    { id: "platform-preflight", commands: [
      ". /etc/os-release",
      `if [ "$ID" != ubuntu ]; then echo "APT installation requires Ubuntu, including Ubuntu in WSL or on Jetson."; false; fi`,
      "printf 'Ubuntu=%s architecture=%s kernel=%s\\n' \"$VERSION_ID\" \"$(dpkg --print-architecture)\" \"$(uname -r)\"",
      "df -h / /tmp",
    ] },
    { id: "locales", commands: [
      "sudo apt-get update",
      "sudo apt-get install --no-remove -y locales",
      "sudo locale-gen en_US en_US.UTF-8",
      "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8",
      "export LANG=en_US.UTF-8",
    ] },
    { id: "repository-prerequisites", commands: [
      "sudo apt-get install --no-remove -y software-properties-common curl",
      "sudo add-apt-repository -y universe",
    ] },
    { id: "ros-repository", commands: [
      "sudo curl -fSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg",
      `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
      "sudo apt-get update",
    ] },
    { id: "package-availability", commands: [
      `apt-cache policy ros-${distro.name}-desktop`,
      `apt-cache show ros-${distro.name}-desktop > /dev/null`,
    ] },
    { id: "ros-packages", commands: [`sudo apt-get install --no-remove -y ros-${distro.name}-desktop`] },
  ];
  return runInstallTerminal(distro, diagnostics, bashInstallScript(steps), false);
}

/**
 * Installs ROS 2 using Pixi on Windows or macOS.
 * Subprocess operations (pixi detection and pixi self-install) run in a
 * dedicated worker thread so the extension host main thread is not blocked.
 */
async function installRosPixi(distro: RosDistro, diagnostics: InstallDiagnostics, distroWorkspace: string): Promise<number | undefined> {
  const worker = new RosInstallWorker();
  try {
    // Check if Pixi is installed (runs in worker thread)
    await diagnostics.log("RDE_STEP_START:pixi-detection\n");
    const pixiInstalled = await isPixiInstalled(worker);
    await diagnostics.log(`Pixi on PATH: ${pixiInstalled}\nRDE_STEP_OK:pixi-detection\n`);

    if (!pixiInstalled) {
      await installPixiViaWorker(worker, diagnostics);
    }

    const stagedManifest = await preflightPixiEnvironment(distro, diagnostics);

    extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} using Pixi...`);
    extension.outputChannel.show();

    await diagnostics.log("RDE_STEP_START:pixi-manifest\n");
    const manifestPath = await createPixiTarget(stagedManifest, distroWorkspace);
    extension.outputChannel.appendLine(`Pixi manifest: ${manifestPath}`);

    await diagnostics.log("RDE_STEP_OK:pixi-manifest\n");
    let script: string;

    if (process.platform === "win32") {
      const quotedManifest = "'" + manifestPath.replace(/'/g, "''") + "'";
      script = powershellInstallScript([
        { id: "pixi-preflight", commands: [
          "& pixi --version",
          "if ($LASTEXITCODE -ne 0) { throw \"Pixi preflight exited $LASTEXITCODE\" }",
          `Get-Content -LiteralPath ${quotedManifest}`,
        ] },
        { id: "ros-packages", commands: [
          `& pixi install --locked --manifest-path ${quotedManifest} -e ${distro.name}`,
          "if ($LASTEXITCODE -ne 0) { throw \"Pixi install exited $LASTEXITCODE\" }",
        ] },
      ], diagnostics.logPath);
    } else {
      const quotedManifest = "'" + manifestPath.replace(/'/g, "'\\''") + "'";
      script = bashInstallScript([
        { id: "pixi-preflight", commands: ["pixi --version", `cat ${quotedManifest}`] },
        { id: "ros-packages", commands: [`pixi install --locked --manifest-path ${quotedManifest} -e ${distro.name}`] },
      ]);
    }

    return await runInstallTerminal(distro, diagnostics, script, process.platform === "win32");
  } finally {
    worker.terminate();
  }
}

export async function runInstallTerminal(
  distro: RosDistro,
  diagnostics: InstallDiagnostics,
  script: string,
  windows: boolean
): Promise<number | undefined> {
  const scriptPath = path.join(diagnostics.directory, windows ? "install.ps1" : "install.sh");
  // Windows PowerShell 5.1 otherwise reads non-ASCII paths using the system ANSI code page.
  await fs.promises.writeFile(scriptPath, windows ? "\uFEFF" + script : script, { mode: 0o700 });
  diagnostics.report.artifacts.script = scriptPath;
  await diagnostics.save();
  extension.outputChannel.appendLine(`Installer script: ${scriptPath}\nInstaller log: ${diagnostics.logPath}`);
  // Run the script as the terminal process, not as input to a persistent shell.
  // The Windows terminal now closes when PowerShell exits, rather than leaving cmd.exe running.
  const terminal = vscode.window.createTerminal({
    name: `ROS 2 ${distro.displayName} Installation`,
    shellPath: windows ? "powershell.exe" : "/bin/bash",
    shellArgs: windows
      ? ["-NoProfile", "-ExecutionPolicy", "Bypass", "-File", scriptPath]
      : ["--noprofile", "--norc", "-c",
        'bash "$1" 2>&1 | tee -a "$2"; codes=("${PIPESTATUS[@]}"); if [ "${codes[0]}" -ne 0 ]; then exit "${codes[0]}"; fi; exit "${codes[1]}"',
        "ros-install", scriptPath, diagnostics.logPath],
  });
  const completion = new Promise<number | undefined>((resolve) => {
    const disposable = vscode.window.onDidCloseTerminal((closed) => {
      if (closed === terminal) {
        disposable.dispose();
        resolve(closed.exitStatus?.code);
      }
    });
  });
  terminal.show();
  return completion;
}

/**
 * Offers Copilot help for diagnosing installation issues
 */
async function offerCopilotHelp(diagnostics: InstallDiagnostics): Promise<void> {
  try {
    await diagnostics.readLog();
    const selected = await vscode.window.showQuickPick([
      { label: "Whole installation", id: undefined as string | undefined },
      ...diagnostics.report.steps.map((step) => ({ label: `${step.id}: ${step.status}`, id: step.id })),
      ...(diagnostics.report.preflight?.checks ?? []).map((check) => ({ label: `${check.id}: ${check.status}`, id: check.id })),
      ...(diagnostics.report.health?.checks ?? []).map((check) => ({ label: `${check.id}: ${check.status}`, id: check.id })),
    ], { placeHolder: "Choose an installation or health step to diagnose" });
    if (!selected) {
      return;
    }
    const prompt = await diagnostics.troubleshootingPrompt(selected.id);
    const document = await vscode.workspace.openTextDocument({ language: "markdown", content: prompt });
    await vscode.window.showTextDocument(document, { preview: false });
    const choice = await vscode.window.showWarningMessage(
      "Review and edit this diagnostic draft before sharing. It includes local paths and log output; automatic redaction is best-effort. Nothing has been sent to AI.",
      "Copy Reviewed Draft", "Copy and Open Copilot"
    );
    if (!choice) {
      return;
    }
    await vscode.env.clipboard.writeText(document.getText());
    if (choice === "Copy Reviewed Draft") {
      return;
    }
    await vscode.commands.executeCommand("workbench.action.chat.open");
    vscode.window.showInformationMessage(
      "Paste the reviewed diagnostic draft into Copilot Chat. No repair commands are executed automatically."
    );
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(
      `Error opening Copilot help: ${errorMessage}`
    );
    vscode.window.showErrorMessage(
      "Failed to open Copilot Chat. Please check the output channel for error details."
    );
    extension.outputChannel.show();
  }
}

let latestDiagnostics: InstallDiagnostics | undefined;
const LAST_REPORT_KEY = "rosInstallationReport";

function getPixiRoot(): string {
  const config = vscode_utils.getExtensionConfiguration();
  const setting = config.inspect<string>("pixiRoot");
  return setting?.workspaceFolderValue ?? setting?.workspaceValue ?? setting?.globalValue ??
    (process.platform === "win32" ? "c:\\pixi_ws" : path.join(os.homedir(), "pixi_ws"));
}

async function createDiagnostics(target: HealthTarget, operation: "install" | "health"): Promise<InstallDiagnostics> {
  if (!extension.extensionContext) {
    throw new Error("The extension must be activated before running installation diagnostics.");
  }
  const diagnostics = await InstallDiagnostics.create(
    extension.extensionContext.globalStorageUri.fsPath, target, operation, vscode.env.remoteName
  );
  latestDiagnostics = diagnostics;
  await extension.extensionContext.globalState.update(LAST_REPORT_KEY, diagnostics.reportPath);
  extension.outputChannel.appendLine(`ROS ${operation} report: ${diagnostics.reportPath}`);
  return diagnostics;
}

async function runPreflight(diagnostics: InstallDiagnostics): Promise<boolean> {
  await diagnostics.log("RDE_STEP_START:preflight\n");
  const report = await vscode.window.withProgress({
    location: vscode.ProgressLocation.Notification,
    title: `Checking readiness to install ROS 2 ${diagnostics.report.target.distro}`,
    cancellable: false,
  }, () => preflightInstallation(diagnostics.report.target));
  diagnostics.report.preflight = report;
  for (const check of report.checks) {
    const detail = `${check.id}: ${check.status}: ${check.detail}` +
      (check.remediation ? `\nAction: ${check.remediation}` : "");
    extension.outputChannel.appendLine(detail);
    await diagnostics.log(detail + "\n");
  }
  await diagnostics.log(report.ready ? "RDE_STEP_OK:preflight\n" : "RDE_STEP_FAILED:preflight:1\n");
  await diagnostics.save();
  if (!report.ready) {
    const blockers = report.checks.filter((check) => check.status === "blocked");
    const message = `Installation aborted by preflight: ${blockers.map((check) => check.id).join(", ")}. Resolve the reported blockers and retry.`;
    diagnostics.report.recovery = ["Preflight aborted before Pixi bootstrap, target writes, repository changes or package installation. No system repair was attempted."];
    await diagnostics.finish("blocked", message);
    await showFailure(diagnostics, message);
    return false;
  }
  const warnings = report.checks.filter((check) => check.status === "warning");
  if (warnings.length > 0) {
    const choice = await vscode.window.showWarningMessage(
      `Preflight found warnings: ${warnings.map((check) => check.id).join(", ")}. Review the Output channel or report before proceeding.`,
      { modal: true }, "Proceed with Installation", "View Report"
    );
    if (choice !== "Proceed with Installation") {
      diagnostics.report.recovery = ["Installation was not started. Preflight warnings were not accepted."];
      await diagnostics.finish("blocked", "Installation cancelled before accepting preflight warnings.");
      if (choice === "View Report") {
        await showReport(diagnostics);
      }
      return false;
    }
  }
  return true;
}

async function runHealthChecks(diagnostics: InstallDiagnostics): Promise<HealthReport> {
  await diagnostics.log("RDE_STEP_START:runtime-validation\n");
  const health = await vscode.window.withProgress({
    location: vscode.ProgressLocation.Notification,
    title: `Validating ROS 2 ${diagnostics.report.target.distro}`,
    cancellable: false,
  }, () => validateInstallation(
    diagnostics.report.target, path.join(extension.extPath, "assets", "scripts", "ros_install_health.py")
  ));
  diagnostics.report.health = health;
  for (const check of health.checks) {
    const line = `${check.id}: ${check.status}: ${check.detail}`;
    extension.outputChannel.appendLine(line);
    await diagnostics.log(line + "\n");
  }
  await diagnostics.log(health.healthy ? "RDE_STEP_OK:runtime-validation\n" : "RDE_STEP_FAILED:runtime-validation:1\n");
  return health;
}

async function showReport(diagnostics: InstallDiagnostics): Promise<void> {
  await diagnostics.readLog();
  await diagnostics.save();
  await vscode.window.showTextDocument(await vscode.workspace.openTextDocument(diagnostics.reportPath));
}

async function showFailure(diagnostics: InstallDiagnostics, message: string): Promise<void> {
  const choice = await vscode.window.showErrorMessage(
    `${message} Report saved. ${diagnostics.report.operation === "install"
      ? "No automatic rollback was performed." : "No package repair was attempted."}`,
    "View Report", "Diagnose Step"
  );
  if (choice === "View Report") {
    await showReport(diagnostics);
  } else if (choice === "Diagnose Step") {
    await offerCopilotHelp(diagnostics);
  }
}

/** Reopens diagnostics after an extension-host reload, without executing anything. */
export async function showInstallationReport(): Promise<void> {
  if (!latestDiagnostics) {
    const reportPath = extension.extensionContext?.globalState.get<string>(LAST_REPORT_KEY);
    if (!reportPath) {
      await vscode.window.showInformationMessage("No ROS installation or health report has been recorded.");
      return;
    }
    latestDiagnostics = await InstallDiagnostics.load(reportPath);
  }
  await showReport(latestDiagnostics);
  const choice = await vscode.window.showInformationMessage(
    latestDiagnostics.report.status === "running"
      ? "This run has no recorded completion. It may still be running or may have been interrupted by a reload."
      : "Installation diagnostics",
    "Diagnose Step", "Open Log"
  );
  if (choice === "Diagnose Step") {
    await offerCopilotHelp(latestDiagnostics);
  } else if (choice === "Open Log") {
    await vscode.window.showTextDocument(await vscode.workspace.openTextDocument(latestDiagnostics.logPath));
  }
}

/** Command entrypoint; passing a target skips selection and returns the structured health result. */
export async function checkRosInstallation(target?: HealthTarget): Promise<HealthReport | undefined> {
  if (!vscode.workspace.isTrusted) {
    throw new Error("Trust this workspace before executing ROS installation health checks.");
  }
  const manager = InstallationManager.getInstance();
  if (!manager.begin()) {
    throw new Error("Wait for the active ROS installation or health check to finish.");
  }
  try {
    const report = await checkSelectedRosInstallation(target);
    if (report?.healthy) {
      manager.markComplete();
    } else {
      manager.markFailed();
    }
    return report;
  } finally {
    if (manager.isInstalling) {
      manager.markFailed();
    }
  }
}

async function checkSelectedRosInstallation(target?: HealthTarget): Promise<HealthReport | undefined> {
  const interactive = target === undefined;
  if (!target) {
    const distro = await selectRosDistro();
    if (!distro) {
      return undefined;
    }
    const configured = vscode_utils.getRosSetupScript();
    const choices: { label: string; value: string }[] = [
      { label: "Installer default location", value: "default" },
      { label: "Choose a Pixi manifest (pixi.toml)", value: "pixi" },
      { label: "Choose a ROS setup script", value: "setup" },
    ];
    if (configured) {
      choices.unshift({ label: `Configured setup: ${configured}`, value: "configured" });
    }
    const choice = await vscode.window.showQuickPick(choices, { placeHolder: "Choose the installation to check" });
    if (!choice) {
      return undefined;
    }
    if (choice.value === "default") {
      target = process.platform === "linux"
        ? { kind: "setup", distro: distro.name, setupScript: `/opt/ros/${distro.name}/setup.bash` }
        : { kind: "pixi", distro: distro.name, workspace: path.join(getPixiRoot(), distro.name) };
    } else if (choice.value === "configured") {
      target = { kind: "setup", distro: distro.name, setupScript: configured };
    } else {
      const selected = await vscode.window.showOpenDialog({
        canSelectMany: false, canSelectFiles: true, canSelectFolders: false,
        title: choice.value === "pixi" ? "Select pixi.toml" : "Select ROS setup script",
      });
      if (!selected?.length) {
        return undefined;
      }
      if (choice.value === "pixi" && path.basename(selected[0].fsPath) !== "pixi.toml") {
        throw new Error("Select a file named pixi.toml.");
      }
      target = choice.value === "pixi"
        ? { kind: "pixi", distro: distro.name, workspace: path.dirname(selected[0].fsPath) }
        : { kind: "setup", distro: distro.name, setupScript: selected[0].fsPath };
    }
  }
  const diagnostics = await createDiagnostics(target, "health");
  try {
    const health = await runHealthChecks(diagnostics);
    await diagnostics.finish(health.healthy ? "passed" : "failed");
    if (health.healthy && interactive) {
      const choice = await vscode.window.showInformationMessage("ROS 2 runtime health checks passed.", "View Report");
      if (choice === "View Report") {
        await showReport(diagnostics);
      }
    } else if (!health.healthy && interactive) {
      await showFailure(diagnostics, "ROS 2 runtime health checks failed.");
    }
    return health;
  } catch (error) {
    await diagnostics.finish("failed", String(error));
    throw error;
  }
}
