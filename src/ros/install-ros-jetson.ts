// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as fs from "fs";
import * as extension from "../extension";
import { RosDistro } from "./install-ros";
import { monitorTerminalForErrors } from "./install-ros-terminal";

/**
 * Installs ROS 2 on NVIDIA Jetson devices with special handling for CUDA, OpenCV, and Python
 */
export async function installRosJetson(distro: RosDistro): Promise<void> {
  extension.outputChannel.appendLine(
    `Installing ROS 2 ${distro.name} on NVIDIA Jetson with CUDA support...`
  );
  extension.outputChannel.show();

  // Create a terminal for the installation
  const terminal = vscode.window.createTerminal({
    name: `ROS 2 ${distro.displayName} Installation (Jetson)`,
  });

  terminal.show();

  const commands = [
    "# ROS 2 Installation on NVIDIA Jetson",
    "# This installation includes CUDA, cuDNN, and optimized OpenCV support",
    "# Please enter your password when prompted",
    "",
    "# Set locale",
    "sudo apt update && sudo apt install -y locales",
    "sudo locale-gen en_US en_US.UTF-8",
    "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8",
    "export LANG=en_US.UTF-8",
    "",
    "# Verify CUDA installation",
    "echo 'Checking CUDA installation...'",
    "which nvcc || echo 'Warning: CUDA toolkit not found'",
    "nvidia-smi || echo 'Warning: NVIDIA runtime not detected'",
    "",
    "# Setup sources",
    "sudo apt install -y software-properties-common curl",
    "sudo add-apt-repository universe",
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
    "# Install Jetson-specific ROS packages (if available)",
    `sudo apt install -y ros-${distro.name}-jetson* 2>/dev/null || echo 'Jetson packages not available for this distro'`,
    "",
    "# Install OpenCV with CUDA support (optional but recommended for Jetson)",
    "echo 'Installing OpenCV dependencies...'",
    "sudo apt install -y libatlas-base-dev libjasper-dev libhdf5-dev libjasper-dev libilmbase-dev libopenexr-dev libwebp6",
    "",
    "# Install Python development headers",
    "sudo apt install -y python3-dev python3-pip",
    "",
    "# Upgrade pip and install common ML libraries",
    "pip3 install --upgrade pip",
    "pip3 install --upgrade numpy",
    "",
    "echo 'ROS 2 installation complete on Jetson!'",
    "echo 'Note: OpenCV with CUDA support may need to be compiled separately for optimal performance.'",
    "echo 'Please reload the window or restart VS Code to detect the new ROS 2 installation.'",
  ];

  for (const command of commands) {
    terminal.sendText(command);
  }

  // Monitor the terminal for completion
  monitorTerminalForErrors(terminal, distro);
}

/**
 * Detects if the system is running on NVIDIA Jetson
 */
export async function isJetson(): Promise<boolean> {
  try {
    // Check for Jetson-specific file
    const proc = await fs.promises.access("/sys/module/tegra_fuse/parameters/tegra_chip_id");
    return true;
  } catch {
    // Fallback: check for tegra in /etc/os-release or device tree
    try {
      const osRelease = await fs.promises.readFile("/etc/os-release", "utf-8");
      return osRelease.includes("tegra") || osRelease.includes("jetson");
    } catch {
      // Another fallback: check if nvidia-smi is available (CUDA/Jetson specific)
      return await hasNvidiaRuntime();
    }
  }
}

/**
 * Checks if NVIDIA runtime is available (indicates Jetson or GPU-enabled system)
 */
async function hasNvidiaRuntime(): Promise<boolean> {
  return new Promise((resolve) => {
    const child_process = require("child_process");
    const cmd = "which nvidia-smi";
    child_process.exec(cmd, (error: any) => {
      resolve(!error);
    });
  });
}
