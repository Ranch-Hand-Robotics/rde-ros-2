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
import { macInstallScript, macOSVersion, macPrerequisiteIssue, requestMacCommandLineTools, pixiManifest, pixiPlatform, pixiSetupScript, quoteShell } from "./pixi";
import { HealthReport, HealthTarget, runHealthProcess, validateInstallation } from "./health-check";
import { InstallDiagnostics, bashInstallScript, powershellInstallScript, ScriptStep } from "./install-diagnostics";
import { preflightInstallation, removeIncompletePixiTarget, runPreflightCommand } from "./install-preflight";
import { cachePixiInstallRoot, getPixiInstallRoot, selectPixiInstallRoot } from "./pixi-location";
import { PreflightCheck } from "./preflight-types";
import { activateWindowsToolchain, WINDOWS_BUILD_TOOLS_COMMAND } from "../windows-toolchain";


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

  /** Returns the working Pixi executable, including supported locations outside PATH. */
  async checkPixi(): Promise<string | undefined> {
    const response = await this.request({ type: "check_pixi" });
    return response.type === "pixi_available" ? response.executable : undefined;
  }

  /**
   * Installs Pixi on the given platform.
   * Streams stdout/stderr via `onLog` while running.
   */
  async installPixi(platform: string, onLog: (text: string) => void): Promise<void> {
    await this.request({ type: "install_pixi", platform }, onLog);
  }

  terminate(): void {
    this._worker.terminate();
  }

  private request(req: WorkerRequest, onLog: (text: string) => void = () => {}): Promise<WorkerResponse> {
    if (this._failure) {
      return Promise.reject(this._failure);
    }
    return new Promise((resolve, reject) => {
      const cleanup = () => {
        clearTimeout(timer);
        this._worker.off("message", onMessage);
        this._worker.off("error", onError);
        this._worker.off("exit", onExit);
      };
      const onError = (error: Error) => { cleanup(); reject(error); };
      const onExit = (code: number) => onError(new Error(`Pixi worker exited (${code})`));
      const onMessage = (message: WorkerResponse) => {
        if (message.type === "log") {
          onLog(message.text);
        } else if (message.type === "error") {
          onError(new Error(message.message));
        } else {
          cleanup();
          resolve(message);
        }
      };
      const timer = setTimeout(() => onError(new Error("Pixi operation timed out")), 15 * 60 * 1000);
      this._worker.on("message", onMessage);
      this._worker.once("error", onError);
      this._worker.once("exit", onExit);
      try {
        this._worker.postMessage(req);
      } catch (error) {
        onError(error instanceof Error ? error : new Error(String(error)));
      }
    });
  }
}

// ---------------------------------------------------------------------------
// Script helpers
// ---------------------------------------------------------------------------

/**
 * Stages a distro-specific manifest with the current host's platform requirements.
 */
async function preparePixiManifest(distro: RosDistro, diagnostics: InstallDiagnostics): Promise<string> {
  const macos = process.platform === "darwin" ? await macOSVersion() : undefined;
  const resolved = pixiManifest(distro.name, pixiPlatform(), macos);
  const staging = path.join(diagnostics.directory, "pixi-plan");
  await fs.promises.mkdir(staging, { mode: 0o700 });
  const manifestPath = path.join(staging, "pixi.toml");
  await fs.promises.writeFile(manifestPath, resolved, { mode: 0o600, flag: "wx" });
  return manifestPath;
}

export async function preflightPixiEnvironment(
  distro: RosDistro, diagnostics: InstallDiagnostics, env?: NodeJS.ProcessEnv
): Promise<string> {
  const stagedManifest = await preparePixiManifest(distro, diagnostics);
  await diagnostics.log("RDE_STEP_START:pixi-solver\n");
  const args = ["lock", "--manifest-path", stagedManifest, "--quiet"];
  const pixiExecutable = diagnostics.report.target.kind === "pixi" ? diagnostics.report.target.pixiExecutable : undefined;
  const plan = env
    ? pixiExecutable
      ? await runHealthProcess({ command: pixiExecutable, args, cwd: path.dirname(stagedManifest), env }, 120000, 8 * 1024 * 1024)
      : { stdout: "", stderr: "", exitCode: null, error: "No detected Pixi executable was recorded for the solver." }
    : await runPreflightCommand(pixiExecutable ?? "pixi", args, 120000);
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
    let pixiRoot: string | undefined;
    if (process.platform === "linux") {
      target = { kind: "setup", distro: distro.name, setupScript: `/opt/ros/${distro.name}/setup.bash` };
    } else if (process.platform === "win32" || process.platform === "darwin") {
      pixiRoot = await selectPixiInstallRoot(distro.name);
      if (!pixiRoot) {
        extension.outputChannel.appendLine("Pixi install location selection was cancelled; no installation was started.");
        manager.markFailed();
        return;
      }
      target = { kind: "pixi", distro: distro.name, workspace: path.join(pixiRoot, distro.name) };
    } else {
      throw new Error(`ROS 2 installation is not supported on platform: ${process.platform}`);
    }
    diagnostics = await createDiagnostics(target, "install");
    if (pixiRoot) {
      try {
        await cachePixiInstallRoot(pixiRoot);
        await diagnostics.log(`Cached Pixi install root for this VS Code machine: ${pixiRoot}\n`);
      } catch (error) {
        extension.outputChannel.appendLine(`Could not cache the Pixi install root in settings: ${error}`);
        await diagnostics.log(`Warning: could not cache the Pixi install root in settings: ${error}\n`);
      }
    }
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
    let setupPath: string;
    if (target.kind === "setup") {
      setupPath = target.setupScript;
    } else if (process.platform === "darwin") {
      setupPath = path.join(target.workspace, "setup.bash");
      await fs.promises.rename(path.join(target.workspace, ".setup.bash"), setupPath);
    } else {
      const library = path.join(target.workspace, ".pixi", "envs", target.distro, "Library");
      setupPath = path.join(library, "local_setup.bat");
      if (!fs.existsSync(setupPath)) {
        setupPath = path.join(library, "setup.bat");
      }
    }
    const scope = vscode.workspace.workspaceFolders?.length || vscode.workspace.workspaceFile
      ? vscode.ConfigurationTarget.Workspace : vscode.ConfigurationTarget.Global;
    const config = vscode_utils.getExtensionConfiguration();
    await config.update("rosSetupScript", setupPath, scope);
    await config.update("distro", target.distro, scope);
    await diagnostics.finish("passed");
    manager.markComplete();
    extension.rosDistributionsProvider?.refresh();
    try {
      await extension.refreshRosEnvironment(target.distro);
    } catch (error) {
      extension.outputChannel.appendLine(`ROS environment refresh failed: ${error}`);
      await vscode.window.showWarningMessage(`ROS 2 installation and runtime validation passed, but refreshing the environment failed: ${error}`);
      return;
    }
    await vscode.window.showInformationMessage("ROS 2 installation and runtime validation passed. The ROS environment has been refreshed.");
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
async function isPixiInstalled(worker: RosInstallWorker): Promise<string | undefined> {
  return worker.checkPixi();
}

/** Confirms the Pixi installation and offers a link to its publisher. */
export async function confirmPixiBootstrap(): Promise<boolean> {
  const method = process.platform === "win32" ? "Windows Package Manager (winget)" : "the installer from pixi.sh";
  const message =
    "Pixi, the package manager by Prefix.dev, is required to install ROS 2 on this platform but is not currently installed. " +
    `This will install Pixi using ${method}. ` +
    (process.platform === "win32" ? "Proceeding accepts the winget source and package agreements. " : "") +
    "Would you like to proceed?";
  let choice: string | undefined;
  do {
    choice = await vscode.window.showWarningMessage(
      message, { modal: true }, "Yes", "No", "Visit Prefix.dev"
    );
    if (choice === "Visit Prefix.dev") {
      try {
        if (!await vscode.env.openExternal(vscode.Uri.parse("https://prefix.dev/"))) {
          extension.outputChannel.appendLine("VS Code could not open https://prefix.dev/.");
        }
      } catch (error) {
        extension.outputChannel.appendLine(`Could not open https://prefix.dev/: ${error}`);
      }
    }
  } while (choice === "Visit Prefix.dev");

  return choice === "Yes";
}

/**
 * Installs Pixi via the worker thread.
 * Streams subprocess output to the extension output channel.
 * Fails explicitly if bootstrap is declined or Pixi remains unavailable.
 */
async function installPixiViaWorker(worker: RosInstallWorker, diagnostics: InstallDiagnostics): Promise<void> {
  if (!(await confirmPixiBootstrap())) {
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
      throw new Error("Pixi bootstrap completed but no working Pixi executable was found. Check the installation and retry.");
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
 * Activates an existing compiler; installing or repairing Build Tools is always manual.
 */
export async function ensureWindowsBuildTools(
  diagnostics: InstallDiagnostics, env: NodeJS.ProcessEnv = process.env
): Promise<NodeJS.ProcessEnv> {
  await diagnostics.log("RDE_STEP_START:windows-compiler\n");
  diagnostics.report.preflight ??= { ready: true, checks: [] };
  let activated: NodeJS.ProcessEnv;
  try {
    activated = await activateWindowsToolchain(env, {
      cwd: diagnostics.directory,
      onOutput: message => extension.outputChannel.appendLine(message),
    });
  } catch (error) {
    const detail = error instanceof Error ? error.message : String(error);
    const remediation = "For a new installation, review the copied command and run it in Administrator PowerShell, then retry ROS 2 installation. " +
      "If Visual Studio or Build Tools is already installed but incomplete, open Visual Studio Installer > Modify > Desktop development with C++, " +
      "and select MSVC v143 (Visual Studio 2022) and a Windows SDK. Running winget install will not add missing components to an existing installation.";
    diagnostics.report.status = "blocked";
    diagnostics.report.preflight.ready = false;
    diagnostics.report.preflight.checks.push({ id: "windows-compiler", status: "blocked", detail, remediation });
    diagnostics.report.recovery = ["Compiler activation failed before Pixi operations or ROS target creation. No Build Tools installation or system repair was attempted.", remediation];
    await diagnostics.log(`RDE_STEP_FAILED:windows-compiler:1\n${detail}\n${remediation}\n`);
    await diagnostics.save();
    const choice = await vscode.window.showWarningMessage(
      `Windows C++ toolchain is not ready: ${detail}\n\n` +
      "Visual Studio 2022 Build Tools requires administrator privileges and a large download, including MSVC and the Windows SDK. " +
      "Running the copied command accepts the winget source and package agreements; review and consent to them before running it. " +
      "The extension will not launch an elevated installer or install Build Tools automatically.\n\n" + remediation,
      { modal: true }, "Copy Install Command", "Cancel"
    );
    if (choice === "Copy Install Command") {
      await vscode.env.clipboard.writeText(WINDOWS_BUILD_TOOLS_COMMAND);
    }
    await diagnostics.log(`Build Tools command ${choice === "Copy Install Command" ? "copied; not executed" : "not copied; installation cancelled"}.\n`);
    throw new Error(`ROS 2 installation blocked: ${detail} ${choice === "Copy Install Command" ? "Install command copied. " : "Build Tools setup was not started. "}${remediation}`);
  }
  diagnostics.report.preflight.checks.push({
    id: "windows-compiler", status: "passed", detail: "Activated a working Windows MSVC and SDK toolchain for Pixi.",
  });
  await diagnostics.log("RDE_STEP_OK:windows-compiler\n");
  await diagnostics.save();
  return activated;
}

/**
 * Installs ROS 2 using Pixi on Windows or macOS.
 * Subprocess operations (pixi detection and pixi self-install) run in a
 * dedicated worker thread so the extension host main thread is not blocked.
 */
async function installRosPixi(distro: RosDistro, diagnostics: InstallDiagnostics, distroWorkspace: string): Promise<number | undefined> {
  pixiPlatform();
  const env = process.platform === "win32" ? await ensureWindowsBuildTools(diagnostics) : undefined;
  if (process.platform === "darwin") {
    const issue = await macPrerequisiteIssue();
    if (issue) {
      const choice = await vscode.window.showWarningMessage(issue, { modal: true }, "Install Apple Tools");
      if (choice === "Install Apple Tools") { await requestMacCommandLineTools(); }
      throw new Error(issue);
    }
  }
  const worker = new RosInstallWorker();
  try {
    // Check if Pixi is installed (runs in worker thread)
    await diagnostics.log("RDE_STEP_START:pixi-detection\n");
    let pixiExecutable = await isPixiInstalled(worker);
    await diagnostics.log(`Pixi executable: ${pixiExecutable ?? "not found"}\nRDE_STEP_OK:pixi-detection\n`);

    if (!pixiExecutable) {
      await installPixiViaWorker(worker, diagnostics);
      pixiExecutable = await isPixiInstalled(worker);
      if (!pixiExecutable) { throw new Error("No working Pixi executable found after bootstrap."); }
    }
    if (diagnostics.report.target.kind === "pixi") {
      diagnostics.report.target.pixiExecutable = pixiExecutable;
    }

    const stagedManifest = await preflightPixiEnvironment(distro, diagnostics, env);

    extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} using Pixi...`);
    extension.outputChannel.show();

    await diagnostics.log("RDE_STEP_START:pixi-manifest\n");
    const manifestPath = await createPixiTarget(stagedManifest, distroWorkspace);
    extension.outputChannel.appendLine(`Pixi manifest: ${manifestPath}`);
    const setupPath = process.platform === "darwin" ? path.join(distroWorkspace, "setup.bash") : undefined;
    const pendingSetupPath = path.join(distroWorkspace, ".setup.bash");
    if (setupPath) {
      await fs.promises.writeFile(pendingSetupPath, pixiSetupScript(pixiExecutable, manifestPath, distro.name), { mode: 0o600 });
    }

    await diagnostics.log("RDE_STEP_OK:pixi-manifest\n");
    let script: string;

    if (process.platform === "win32") {
      const quotedManifest = "'" + manifestPath.replace(/'/g, "''") + "'";
      const quotedPixi = "'" + pixiExecutable.replace(/'/g, "''") + "'";
      script = powershellInstallScript([
        { id: "pixi-preflight", commands: [
          `& ${quotedPixi} --version`,
          "if ($LASTEXITCODE -ne 0) { throw \"Pixi preflight exited $LASTEXITCODE\" }",
          `Get-Content -LiteralPath ${quotedManifest}`,
        ] },
        { id: "ros-packages", commands: [
          `& ${quotedPixi} install --locked --manifest-path ${quotedManifest} -e ${distro.name}`,
          "if ($LASTEXITCODE -ne 0) { throw \"Pixi install exited $LASTEXITCODE\" }",
        ] },
      ], diagnostics.logPath);
    } else {
      script = bashInstallScript([
        { id: "ros-packages", commands: ["set +u", macInstallScript(pixiExecutable, distroWorkspace, distro.name, pendingSetupPath, { locked: true, publishSetup: false })] },
      ]);
    }

    return await runInstallTerminal(distro, diagnostics, script, process.platform === "win32", env);
  } finally {
    worker.terminate();
  }
}

export async function runInstallTerminal(
  distro: RosDistro,
  diagnostics: InstallDiagnostics,
  script: string,
  windows: boolean,
  env?: NodeJS.ProcessEnv
): Promise<number | undefined> {
  const scriptPath = path.join(diagnostics.directory, windows ? "install.ps1" : "install.sh");
  // Windows PowerShell 5.1 otherwise reads non-ASCII paths using the system ANSI code page.
  await fs.promises.writeFile(scriptPath, windows ? "\uFEFF" + script : script, { mode: 0o700 });
  diagnostics.report.artifacts.script = scriptPath;
  await diagnostics.save();
  extension.outputChannel.appendLine(`Installer script: ${scriptPath}\nInstaller log: ${diagnostics.logPath}`);
  const command = windows
    ? `& '${scriptPath.replace(/'/g, "''")}'; exit $LASTEXITCODE`
    : `bash ${quoteShell(scriptPath)} 2>&1 | tee -a ${quoteShell(diagnostics.logPath)}; codes=("\${PIPESTATUS[@]}"); if [ "\${codes[0]}" -ne 0 ]; then exit "\${codes[0]}"; fi; exit "\${codes[1]}"`;
  try {
    return await runInstallationTask(command, {
      executable: windows ? "powershell.exe" : "/bin/bash",
      shellArgs: windows ? ["-NoProfile", "-ExecutionPolicy", "Bypass", "-Command"] : ["--noprofile", "--norc", "-c"],
      cwd: vscode.workspace.workspaceFolders?.[0]?.uri.fsPath ?? os.homedir(),
      env,
    }, distro, scriptPath);
  } finally {
    delete diagnostics.report.artifacts.script;
    await diagnostics.save();
  }
}

export async function runInstallationTask(
  command: string,
  options: vscode.ShellExecutionOptions,
  distro: RosDistro,
  temporaryScriptPath?: string
): Promise<number | undefined> {
  const cleanup = async () => {
    if (temporaryScriptPath) {
      try {
        await fs.promises.unlink(temporaryScriptPath);
      } catch (error) {
        if (error.code !== "ENOENT") {
          extension.outputChannel?.appendLine(`Could not remove temporary installer script ${temporaryScriptPath}: ${error}`);
        }
      }
    }
  };
  try {
    const task = new vscode.Task(
      { type: "shell", id: `${distro.name}-${Date.now()}` },
      vscode.workspace.workspaceFolders?.[0] ?? vscode.TaskScope.Global,
      `ROS 2 ${distro.displayName} Installation`, "ROS 2",
      new vscode.ShellExecution(command, options), []
    );
    task.presentationOptions = {
      reveal: vscode.TaskRevealKind.Always, panel: vscode.TaskPanelKind.New,
      close: false, clear: false, showReuseMessage: false,
    };
    return await new Promise<number | undefined>((resolve, reject) => {
      let execution: vscode.TaskExecution | undefined;
      let finished = false;
      const finish = (ended: vscode.TaskExecution, code?: number) => {
        if (!finished && (ended === execution || ended.task === task)) {
          finished = true;
          processListener.dispose();
          endListener.dispose();
          resolve(code);
        }
      };
      const processListener = vscode.tasks.onDidEndTaskProcess(event => finish(event.execution, event.exitCode));
      const endListener = vscode.tasks.onDidEndTask(event => finish(event.execution));
      vscode.tasks.executeTask(task).then(started => { execution = started; }, error => {
        processListener.dispose();
        endListener.dispose();
        reject(error);
      });
    });
  } finally {
    await cleanup();
  }
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

  const windowsRecommendations = report.checks.filter((check) =>
    check.status === "warning" && ["windows-developer-mode", "windows-long-paths"].includes(check.id)
  );
  if (windowsRecommendations.length > 0) {
    const advice = windowsRecommendations.map((check) => `${check.detail} ${check.remediation ?? ""}`).join("\n\n");
    const choice = await vscode.window.showWarningMessage(
      `Windows recommends enabling Developer Mode and Win32 long-path support before installing ROS 2.\n\n${advice}\n\nContinue with installation now?`,
      { modal: true }, "Continue Now", "Stop"
    );
    await diagnostics.log(`Windows settings recommendation choice: ${choice ?? "Stop"}.\n`);
    if (choice !== "Continue Now") {
      diagnostics.report.recovery = ["Installation stopped before Pixi bootstrap, target writes, repository changes or package installation. No Windows settings were changed."];
      await diagnostics.finish("blocked", "Installation stopped at the Windows settings recommendation.");
      return false;
    }
  }

  const incompleteTarget = report.checks.find((check) => check.id === "target" && check.status === "warning");
  if (incompleteTarget && diagnostics.report.target.kind === "pixi") {
    const target = diagnostics.report.target;
    const choice = await vscode.window.showWarningMessage(
      `The Pixi target at ${target.workspace} appears to be incomplete: it contains only the generated pixi.toml and pixi.lock, with no installed environment. Remove it and start over?`,
      { modal: true }, "Remove and Start Over", "Stop"
    );
    await diagnostics.log(`Incomplete Pixi target recovery choice: ${choice ?? "Stop"}.\n`);
    if (choice !== "Remove and Start Over") {
      diagnostics.report.recovery = [`The incomplete target was kept at ${target.workspace}. No installation changes were made.`];
      await diagnostics.finish("blocked", "Installation stopped; the incomplete Pixi target was kept.");
      return false;
    }
    try {
      await removeIncompletePixiTarget(target.workspace, target.distro);
    } catch (error) {
      const message = `Could not safely remove the incomplete Pixi target: ${String(error)}`;
      incompleteTarget.status = "blocked";
      incompleteTarget.detail = message;
      incompleteTarget.remediation = "Inspect the target and any reported quarantine path manually. No unrelated files were intentionally removed.";
      report.ready = false;
      diagnostics.report.recovery = ["The incomplete Pixi target could not be safely reset. No package installation was started."];
      await diagnostics.log(`target: blocked: ${message}\n`);
      await diagnostics.save();
      await diagnostics.finish("blocked", message);
      await showFailure(diagnostics, message);
      return false;
    }
    incompleteTarget.status = "passed";
    incompleteTarget.detail = `Removed the confirmed incomplete Pixi manifest and lockfile from ${target.workspace}; a clean target will be created.`;
    delete incompleteTarget.remediation;
    await diagnostics.log(`${incompleteTarget.id}: passed: ${incompleteTarget.detail}\n`);
    await diagnostics.save();
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
        : { kind: "pixi", distro: distro.name, workspace: path.join(getPixiInstallRoot(), distro.name) };
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
