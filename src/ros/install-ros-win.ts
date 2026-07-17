// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as os from "os";
import * as vscode_utils from "../vscode-utils";
import * as extension from "../extension";
import * as pixi from "./install-ros-pixi";
import { RosDistro } from "./install-ros";
import { monitorTerminalForErrors } from "./install-ros-terminal";

/**
 * Writes an install script to a uniquely-named temp file and returns its path.
 */
async function writeInstallScript(content: string, ext: string): Promise<string> {
  const name = `ros2-install-${Date.now()}-${Math.floor(Math.random() * 0xffff).toString(16)}${ext}`;
  const scriptPath = path.join(os.tmpdir(), name);
  await fs.promises.writeFile(scriptPath, content, { encoding: "utf-8", mode: 0o700 });
  return scriptPath;
}

function createInstallLogPath(scope: string): string {
  const name = `ros2-install-${scope}-${Date.now()}-${Math.floor(Math.random() * 0xffff).toString(16)}.log`;
  return path.join(os.tmpdir(), name);
}

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

/**
 * Installs ROS 2 on Windows using Pixi
 */
export async function installRosWindows(distro: RosDistro): Promise<void> {
  // Check if Pixi is installed
  const pixiInstalled = await pixi.isPixiInstalled();

  if (!pixiInstalled) {
    const installed = await pixi.installPixi();
    if (!installed) {
      return;
    }
  }

  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} on Windows using Pixi...`);
  extension.outputChannel.show();

  // Get or create pixi workspace directory with Windows default
  const config = vscode_utils.getExtensionConfiguration();
  const defaultPixiRoot = "c:\\pixi_ws";
  const pixiRoot = config.get<string>("pixiRoot", defaultPixiRoot);

  // Create the directory (and any missing parents) if it doesn't exist
  try {
    await fs.promises.mkdir(pixiRoot, { recursive: true });
  } catch (err) {
    const errorMessage = err instanceof Error ? err.message : String(err);
    extension.outputChannel.appendLine(
      `Failed to create Pixi root directory "${pixiRoot}": ${errorMessage}`
    );
    throw err;
  }

  const distroWorkspace = path.join(pixiRoot, distro.name);

  try {
    await fs.promises.mkdir(distroWorkspace, { recursive: true });
  } catch (err) {
    const errorMessage = err instanceof Error ? err.message : String(err);
    extension.outputChannel.appendLine(
      `Failed to create Pixi distro workspace "${distroWorkspace}": ${errorMessage}`
    );
    throw err;
  }

  const manifestPath = await createPixiManifest(distro, distroWorkspace);
  extension.outputChannel.appendLine(`Windows Pixi manifest: ${manifestPath}`);

  // Write a fail-fast batch script so every step is checked and the terminal
  // exit code accurately reflects success or failure.
  const scriptContent = [
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

  const scriptPath = await writeInstallScript(scriptContent, ".bat");
  const installLogPath = createInstallLogPath(`win-pixi-${distro.name}`);
  extension.outputChannel.appendLine(`Windows install script: ${scriptPath}`);
  extension.outputChannel.appendLine(`Windows install log: ${installLogPath}`);

  // Pin to cmd.exe so we always know the interpreter.
  const terminal = vscode.window.createTerminal({
    name: `ROS 2 ${distro.displayName} Installation (Pixi)`,
    shellPath: "cmd.exe",
    cwd: pixiRoot,
  });

  terminal.show();
  // Stream output live while also capturing to log.
  const psScriptPath = scriptPath.replace(/'/g, "''");
  const psLogPath = installLogPath.replace(/'/g, "''");
  terminal.sendText(`powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Continue'; & '${psScriptPath}' 2>&1 | Tee-Object -FilePath '${psLogPath}'; Write-Host 'Install log saved to: ${installLogPath}'; exit $LASTEXITCODE"`);

  // Monitor the terminal for completion
  monitorTerminalForErrors(terminal, distro, installLogPath);
}

/**
 * Detects if the system is Windows (always true when this module is called)
 */
export function isWindows(): boolean {
  return process.platform === "win32";
}
