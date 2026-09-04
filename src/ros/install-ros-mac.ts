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
 * Installs ROS 2 on macOS using Pixi
 */
export async function installRosMac(distro: RosDistro): Promise<void> {
  // Check if Pixi is installed
  const pixiInstalled = await pixi.isPixiInstalled();

  if (!pixiInstalled) {
    const installed = await pixi.installPixi();
    if (!installed) {
      return;
    }
  }

  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} on macOS using Pixi...`);
  extension.outputChannel.show();

  // Get or create pixi workspace directory with macOS default
  const config = vscode_utils.getExtensionConfiguration();
  const defaultPixiRoot = path.join(os.homedir(), "pixi_ws");
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

  // Create a terminal for the installation
  const terminal = vscode.window.createTerminal({
    name: `ROS 2 ${distro.displayName} Installation (Pixi)`,
    cwd: pixiRoot,
  });

  terminal.show();

  // Send Pixi commands to install ROS 2
  const commands = [
    "# Installing ROS 2 via Pixi on macOS",
    `# Target directory: ${pixiRoot}`,
    "",
    "# Initialize Pixi project if not already done",
    "pixi init --channel conda-forge --channel robostack-staging ros2-workspace",
    "cd ros2-workspace",
    "",
    "# Add ROS 2 packages",
    `pixi add ros-${distro.name}-desktop`,
    "",
    "echo 'ROS 2 installation complete!'",
    `echo 'Pixi workspace created at: ${path.join(pixiRoot, "ros2-workspace")}'`,
    "echo 'Please configure the ROS2.pixiRoot setting in VS Code to point to this workspace if needed.'",
  ];

  for (const command of commands) {
    terminal.sendText(command);
  }

  // Monitor the terminal for completion
  monitorTerminalForErrors(terminal, distro);
}

/**
 * Detects if the system is macOS (always true when this module is called)
 */
export function isMac(): boolean {
  return process.platform === "darwin";
}
