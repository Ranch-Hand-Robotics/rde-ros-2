// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as extension from "../extension";
import { RosDistro } from "./install-ros";
import { monitorTerminalForErrors } from "./install-ros-terminal";

/**
 * Installs ROS 2 on Linux using APT
 */
export async function installRosUbuntu(distro: RosDistro): Promise<void> {
  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} on Ubuntu using APT...`);
  extension.outputChannel.show();

  // Create a terminal for the installation
  const terminal = vscode.window.createTerminal({
    name: `ROS 2 ${distro.displayName} Installation`,
  });

  terminal.show();

  // Send commands to the terminal one at a time
  // Note: This requires sudo, so the terminal will be focused for the user to enter password
  const commands = [
    "# ROS 2 Installation",
    "# Please enter your password when prompted",
    "",
    "# Set locale",
    "sudo apt update && sudo apt install -y locales",
    "sudo locale-gen en_US en_US.UTF-8",
    "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8",
    "export LANG=en_US.UTF-8",
    "",
    "# Setup sources",
    "sudo apt install -y software-properties-common",
    "sudo add-apt-repository universe",
    "sudo apt update && sudo apt install -y curl",
    "",
    "# Add ROS 2 GPG key",
    "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg",
    "",
    "# Add repository to sources list",
    `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
    "",
    "# Install ROS 2 packages",
    "sudo apt update",
    `sudo apt install -y ros-${distro.name}-desktop`,
    "",
    "echo 'ROS 2 installation complete!'",
    "echo 'Please reload the window or restart VS Code to detect the new ROS 2 installation.'",
  ];

  // Send all commands as a single block for user convenience
  for (const command of commands) {
    terminal.sendText(command);
  }

  // Monitor the terminal for completion
  monitorTerminalForErrors(terminal, distro);
}

/**
 * Detects if the system is running Ubuntu
 */
export async function isUbuntu(): Promise<boolean> {
  // Simple heuristic: check if /etc/os-release exists and contains Ubuntu
  try {
    const fs = require("fs");
    const osRelease = fs.readFileSync("/etc/os-release", "utf-8");
    return /ID=ubuntu|ID_LIKE=.*ubuntu/.test(osRelease);
  } catch {
    return false;
  }
}
