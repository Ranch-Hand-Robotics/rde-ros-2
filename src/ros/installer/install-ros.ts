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

/**
 * Maximum length for environment variable values in diagnostic output
 */
const MAX_ENV_VALUE_LENGTH = 500;
const MAX_INSTALL_LOG_CHARS = 15000;

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
   * The caller is responsible for calling `markComplete` or `markFailed`
   * when the installation terminal closes.
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

  constructor() {
    // The worker bundle is placed beside extension.js in the dist/ folder.
    const workerPath = path.join(extension.extPath, "dist", "install-ros-worker.js");
    this._worker = new Worker(workerPath);

    // Propagate unhandled worker errors to the extension output channel.
    this._worker.on("error", (err) => {
      extension.outputChannel.appendLine(`[install-ros-worker] Unhandled error: ${err.message}`);
    });
  }

  /** Returns true when pixi is found on PATH. */
  checkPixi(): Promise<boolean> {
    return new Promise<boolean>((resolve, reject) => {
      const handler = (msg: WorkerResponse) => {
        if (msg.type === "pixi_available") {
          this._worker.off("message", handler);
          resolve(msg.available);
        } else if (msg.type === "error") {
          this._worker.off("message", handler);
          reject(new Error(msg.message));
        }
      };
      this._worker.on("message", handler);
      this._send({ type: "check_pixi" });
    });
  }

  /**
   * Installs Pixi on the given platform.
   * Streams stdout/stderr via `onLog` while running.
   */
  installPixi(platform: string, onLog: (text: string) => void): Promise<void> {
    return new Promise<void>((resolve, reject) => {
      const handler = (msg: WorkerResponse) => {
        if (msg.type === "log") {
          onLog(msg.text);
        } else if (msg.type === "complete") {
          this._worker.off("message", handler);
          resolve();
        } else if (msg.type === "error") {
          this._worker.off("message", handler);
          reject(new Error(msg.message));
        }
      };
      this._worker.on("message", handler);
      this._send({ type: "install_pixi", platform });
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
 * Writes `content` to a uniquely-named temp file and returns its path.
 * Uses a timestamp+random suffix to avoid collisions across concurrent calls.
 */
async function writeInstallScript(content: string, extension: string): Promise<string> {
  const name = `ros2-install-${Date.now()}-${Math.floor(Math.random() * 0xffff).toString(16)}${extension}`;
  const scriptPath = path.join(os.tmpdir(), name);
  await fs.promises.writeFile(scriptPath, content, { encoding: "utf-8", mode: 0o700 });
  return scriptPath;
}

/** Creates a unique temp file path for installer logs. */
function createInstallLogPath(scope: string): string {
  const name = `ros2-install-${scope}-${Date.now()}-${Math.floor(Math.random() * 0xffff).toString(16)}.log`;
  return path.join(os.tmpdir(), name);
}

/**
 * Reads a captured install log and returns a bounded snippet for Copilot help.
 */
async function readInstallLogSnippet(installLogPath?: string): Promise<string> {
  if (!installLogPath) {
    return "No install log file was captured for this run.";
  }

  try {
    const full = await fs.promises.readFile(installLogPath, "utf-8");
    if (full.length <= MAX_INSTALL_LOG_CHARS) {
      return full;
    }

    const tail = full.slice(-MAX_INSTALL_LOG_CHARS);
    return `[Log truncated to last ${MAX_INSTALL_LOG_CHARS} characters]\n${tail}`;
  } catch (err) {
    return `Failed to read install log at ${installLogPath}: ${err}`;
  }
}

/**
 * Generates a distro-specific pixi.toml in the given workspace directory from
 * the RoboStack template in assets/ros/robostack.toml.
 */
async function createPixiManifest(distro: RosDistro, workspaceDir: string): Promise<string> {
  const templatePath = path.join(extension.extPath, "assets", "ros", "robostack.toml");
  let template = "";

  try {
    template = await fs.promises.readFile(templatePath, "utf-8");
  } catch (err) {
    extension.outputChannel.appendLine(`Warning: failed to read RoboStack template (${templatePath}): ${err}`);
  }

  if (!template.trim()) {
    template = [
      "[workspace]",
      "name = \"ros2-workspace\"",
      "channels = [\"conda-forge\", \"robostack-staging\"]",
      "platforms = [\"win-64\", \"linux-64\", \"osx-64\", \"osx-arm64\"]",
      "",
      "[dependencies]",
      "ros-__ROS_DISTRO__-desktop = \"*\"",
      "",
    ].join("\n");
  }

  const resolved = template.replace(/__ROS_DISTRO__/g, distro.name);
  const manifestPath = path.join(workspaceDir, "pixi.toml");
  await fs.promises.writeFile(manifestPath, resolved, "utf-8");
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
  const manager = InstallationManager.getInstance();

  if (!manager.begin()) {
    vscode.window.showWarningMessage(
      "A ROS 2 installation is already in progress. Please wait for it to complete."
    );
    return;
  }

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

    // Install based on platform.
    // The manager state is updated to complete/failed from monitorTerminalForErrors
    // once the installation terminal closes.
    if (process.platform === "linux") {
      await installRosLinux(distro, manager);
    } else if (process.platform === "win32" || process.platform === "darwin") {
      await installRosPixi(distro, manager);
    } else {
      vscode.window.showErrorMessage(
        `ROS 2 installation is not supported on platform: ${process.platform}`
      );
      manager.markFailed();
    }
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(`Error during ROS installation: ${errorMessage}`);
    vscode.window.showErrorMessage(`Failed to install ROS 2: ${errorMessage}`);
    manager.markFailed();
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
 * @returns true if Pixi was successfully installed, false if the user declined.
 */
async function installPixiViaWorker(worker: RosInstallWorker): Promise<boolean> {
  const choice = await vscode.window.showWarningMessage(
    "Pixi package manager is required to install ROS 2 on this platform but is not currently installed. " +
    "This will install Pixi using Windows Package Manager (winget). Would you like to proceed?",
    { modal: true },
    "Yes",
    "No"
  );

  if (choice !== "Yes") {
    return false;
  }

  extension.outputChannel.appendLine("Installing Pixi...");
  extension.outputChannel.show();

  try {
    await worker.installPixi(process.platform, (text) => {
      extension.outputChannel.append(text);
    });
    extension.outputChannel.appendLine("Pixi installed successfully");
    vscode.window.showInformationMessage(
      "Pixi has been installed successfully. You may need to restart your terminal or VS Code for changes to take effect."
    );
    return true;
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(`Failed to install Pixi: ${errorMessage}`);
    vscode.window.showErrorMessage(`Failed to install Pixi: ${errorMessage}`);
    return false;
  }
}

/**
 * Installs ROS 2 on Linux using APT
 */
async function installRosLinux(distro: RosDistro, manager: InstallationManager): Promise<void> {
  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} on Linux using APT...`);
  extension.outputChannel.show();

  // Write a bash script so every step is fail-fast and the terminal exit code
  // accurately reflects success or failure.
  const scriptContent = [
    "#!/bin/bash",
    "set -euo pipefail",
    "sudo apt update && sudo apt install -y locales",
    "sudo locale-gen en_US en_US.UTF-8",
    "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8",
    "export LANG=en_US.UTF-8",
    "sudo apt install -y software-properties-common",
    "sudo add-apt-repository universe",
    "sudo apt update && sudo apt install -y curl",
    "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg",
    `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
    "sudo apt update",
    `sudo apt install -y ros-${distro.name}-desktop`,
    `echo "ROS 2 ${distro.displayName} installation complete!"`,
    `echo "Please reload the window or restart VS Code to detect the new installation."`,
  ].join("\n");

  const scriptPath = await writeInstallScript(scriptContent, ".sh");
  const installLogPath = createInstallLogPath(`linux-${distro.name}`);
  extension.outputChannel.appendLine(`Linux install script: ${scriptPath}`);
  extension.outputChannel.appendLine(`Linux install log: ${installLogPath}`);

  // Pin the shell to /bin/bash so we always know the interpreter.
  const terminal = vscode.window.createTerminal({
    name: `ROS 2 ${distro.displayName} Installation`,
    shellPath: "/bin/bash",
  });

  terminal.show();

  // Run the script then immediately exit so the terminal exit code == script exit code.
  terminal.sendText(`bash "${scriptPath}" 2>&1 | tee "${installLogPath}"; exit \${PIPESTATUS[0]}`);

  // Monitor the terminal for completion
  monitorTerminalForErrors(terminal, distro, manager, installLogPath);
}

/**
 * Installs ROS 2 using Pixi on Windows or macOS.
 * Subprocess operations (pixi detection and pixi self-install) run in a
 * dedicated worker thread so the extension host main thread is not blocked.
 */
async function installRosPixi(distro: RosDistro, manager: InstallationManager): Promise<void> {
  const worker = new RosInstallWorker();
  try {
    // Check if Pixi is installed (runs in worker thread)
    const pixiInstalled = await isPixiInstalled(worker);

    if (!pixiInstalled) {
      const installed = await installPixiViaWorker(worker);
      if (!installed) {
        manager.markFailed();
        return;
      }
    }

    extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} using Pixi...`);
    extension.outputChannel.show();

    // Get the Pixi base root from config; each distro gets its own subdirectory.
    const config = vscode_utils.getExtensionConfiguration();
    const defaultPixiRoot =
      process.platform === "win32" ? "c:\\pixi_ws" : path.join(os.homedir(), "pixi_ws");
    const pixiRoot = config.get<string>("pixiRoot", defaultPixiRoot);
    const distroWorkspace = path.join(pixiRoot, distro.name);

    // Ensure directories exist and generate the distro-specific manifest.
    try {
      await fs.promises.mkdir(pixiRoot, { recursive: true });
      await fs.promises.mkdir(distroWorkspace, { recursive: true });
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : String(err);
      extension.outputChannel.appendLine(`Failed to create Pixi root directory "${pixiRoot}": ${errorMessage}`);
      throw err;
    }

    const manifestPath = await createPixiManifest(distro, distroWorkspace);
    extension.outputChannel.appendLine(`Pixi manifest: ${manifestPath}`);

    // Build a fail-fast script appropriate for the platform, write it to a temp
    // file, and execute it as a single command so the terminal exit code
    // accurately reflects success or failure of every step.
    let scriptPath: string;
    let runCommand: string;
    const installLogPath = createInstallLogPath(`pixi-${process.platform}-${distro.name}`);

    if (process.platform === "win32") {
      const script = [
        "@echo off",
        "setlocal enableextensions",
        `cd /d "${distroWorkspace}"`,
        "if errorlevel 1 exit 1",
        "echo ==== PIXI PREFLIGHT ====",
        `echo Target environment: ${distro.name}`,
        "pixi --version",
        "if errorlevel 1 exit 1",
        "if not exist pixi.toml (echo ERROR: pixi.toml not found in workspace & exit 1)",
        "type pixi.toml",
        "echo ==== PIXI INSTALL ====",
        `pixi install -e ${distro.name}`,
        "if errorlevel 1 exit 1",
        `echo ROS 2 ${distro.displayName} installation complete!`,
        `echo Pixi workspace created at: ${distroWorkspace}`,
      ].join("\r\n");
      scriptPath = await writeInstallScript(script, ".bat");
      // Stream output live while also capturing to log. PowerShell's
      // Tee-Object gives us real-time output and a persisted log file.
      const psScriptPath = scriptPath.replace(/'/g, "''");
      const psLogPath = installLogPath.replace(/'/g, "''");
      runCommand = `powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Continue'; & '${psScriptPath}' 2>&1 | Tee-Object -FilePath '${psLogPath}'; Write-Host 'Install log saved to: ${installLogPath}'; exit $LASTEXITCODE"`;
    } else {
      const script = [
        "#!/bin/sh",
        "set -eu",
        `cd "${distroWorkspace}"`,
        "echo '==== PIXI PREFLIGHT ===='",
        `echo 'Target environment: ${distro.name}'`,
        "pixi --version",
        "test -f pixi.toml",
        "cat pixi.toml",
        "echo '==== PIXI INSTALL ===='",
        `pixi install -e ${distro.name}`,
        `echo "ROS 2 ${distro.displayName} installation complete!"`,
        `echo "Pixi workspace created at: ${distroWorkspace}"`,
      ].join("\n");
      scriptPath = await writeInstallScript(script, ".sh");
      runCommand = `bash "${scriptPath}" 2>&1 | tee "${installLogPath}"; exit \${PIPESTATUS[0]}`;
    }

    extension.outputChannel.appendLine(`Pixi install script: ${scriptPath}`);
    extension.outputChannel.appendLine(`Pixi install log: ${installLogPath}`);

    // Pin the shell explicitly so we always know the interpreter.
    const shellPath = process.platform === "win32" ? "cmd.exe" : "/bin/bash";
    const terminal = vscode.window.createTerminal({
      name: `ROS 2 ${distro.displayName} Installation (Pixi)`,
      shellPath,
      cwd: pixiRoot,
    });

    terminal.show();
    terminal.sendText(runCommand);

    // Monitor for errors; terminal close handler updates manager state
    monitorTerminalForErrors(terminal, distro, manager, installLogPath);
  } finally {
    // Worker is only needed for pre-install subprocess checks; terminate it now
    // that the installation terminal has been launched.
    worker.terminate();
  }
}

/**
 * Monitors a terminal for errors and offers Copilot help if errors are detected.
 * Updates the {@link InstallationManager} state when the terminal closes.
 */
function monitorTerminalForErrors(
  terminal: vscode.Terminal,
  distro: RosDistro,
  manager: InstallationManager,
  installLogPath?: string
): void {
  // Set up terminal exit handler
  const disposable = vscode.window.onDidCloseTerminal((closedTerminal) => {
    if (closedTerminal === terminal) {
      disposable.dispose();

      // Check if there were errors in the output
      const exitCode = closedTerminal.exitStatus?.code;

      if (exitCode === 0) {
        manager.markComplete();
        // Terminal exited successfully
        vscode.window
          .showInformationMessage(
            "ROS 2 installation completed. Please reload the window to detect the new installation.",
            "Reload Window"
          )
          .then((choice) => {
            if (choice === "Reload Window") {
              vscode.commands.executeCommand("workbench.action.reloadWindow");
            }
          });
      } else {
        manager.markFailed();
        // Terminal did not report a successful exit; installation may have failed or been interrupted
        const message =
          exitCode !== undefined
            ? `ROS 2 installation may have encountered errors (exit code: ${exitCode}). Would you like help diagnosing the issue?`
            : "ROS 2 installation terminal was closed. The installation may not have completed successfully. Would you like help diagnosing potential issues?";

        vscode.window
          .showErrorMessage(message, "Get Copilot Help", "Dismiss")
          .then((choice) => {
            if (choice === "Get Copilot Help") {
              offerCopilotHelp(distro, installLogPath);
            }
          });
      }
    }
  });
}

/**
 * Offers Copilot help for diagnosing installation issues
 */
async function offerCopilotHelp(distro: RosDistro, installLogPath?: string): Promise<void> {
  try {
    // Check if the LM API is available
    if (!("lm" in vscode) || !vscode.lm) {
      vscode.window.showWarningMessage(
        "Copilot integration is not available in this version of VS Code. Please check the output channel for error details."
      );
      extension.outputChannel.show();
      return;
    }

    // Read the troubleshooting prompt
    const promptPath = path.join(
      extension.extPath,
      "assets",
      "prompts",
      "ros-install-troubleshooting.md"
    );

    let systemPrompt = "";
    try {
      systemPrompt = await fs.promises.readFile(promptPath, "utf-8");
    } catch (err) {
      extension.outputChannel.appendLine(
        `Warning: Could not load troubleshooting prompt from ${promptPath}: ${err}`
      );
    }

    // Gather diagnostic information
    const osInfo = `${process.platform} ${process.arch} (${os.release()})`;
    const envInfo = JSON.stringify(
      {
        ROS_DISTRO: extension.env?.ROS_DISTRO,
        ROS_VERSION: extension.env?.ROS_VERSION,
        PATH: process.env.PATH?.substring(0, MAX_ENV_VALUE_LENGTH), // Truncate to avoid huge output
        PYTHONPATH: process.env.PYTHONPATH,
        CMAKE_PREFIX_PATH: extension.env?.CMAKE_PREFIX_PATH,
      },
      null,
      2
    );
    const installLogSnippet = await readInstallLogSnippet(installLogPath);

    // Build the user prompt with diagnostic information
    const userPrompt = `I encountered an error while installing ROS 2 ${distro.displayName} (${distro.name}).

**Operating System:**
${osInfo}

**ROS 2 Distribution:**
${distro.name} (${distro.displayName})${distro.isLTS ? " - LTS" : ""}

**Environment Variables:**
\`\`\`json
${envInfo}
\`\`\`

**Extension Context:**
This issue occurred in the Robot Developer Extensions for ROS 2 VS Code extension while trying to install a ROS 2 distribution and configure it for workspace use.

**Installation Method:**
${process.platform === "linux" ? "APT package manager on Linux" : "Pixi package manager"}

**Captured Install Log Path:**
${installLogPath ?? "(none)"}

**Captured Install Log:**
\`\`\`
${installLogSnippet}
\`\`\`

Can you help diagnose what went wrong and provide steps to fix the installation?`;

    const fullPrompt = systemPrompt
      ? `${systemPrompt}\n\n---\n\n${userPrompt}`
      : userPrompt;

    // Copy the prompt to the clipboard and open Copilot Chat.
    // VS Code's "workbench.action.chat.open" command does not accept a "query" parameter,
    // so users need to paste the copied prompt into the chat manually.
    await vscode.env.clipboard.writeText(fullPrompt);
    await vscode.commands.executeCommand("workbench.action.chat.open");
    
    vscode.window.showInformationMessage(
      "A ROS 2 installation troubleshooting prompt has been copied to your clipboard. Paste it into Copilot Chat to get help."
    );

    extension.outputChannel.appendLine("Opened Copilot Chat for installation troubleshooting");
    extension.outputChannel.appendLine("Troubleshooting prompt copied to clipboard");
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
