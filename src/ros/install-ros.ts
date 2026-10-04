// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as os from "os";

import * as vscode_utils from "../vscode-utils";
import * as extension from "../extension";
import * as installWin from "./install-ros-win";
import * as installMac from "./install-ros-mac";
import * as installUbuntu from "./install-ros-ubuntu";
import * as installJetson from "./install-ros-jetson";

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
    name: "kilted",
    displayName: "Kilted Kaiju",
    isLTS: false,
    releaseDate: "May 2024",
    supportedPlatforms: ["Ubuntu 24.04", "Windows 10/11", "macOS"],
  },
  {
    name: "jazzy",
    displayName: "Jazzy Jalisco",
    isLTS: true,
    releaseDate: "May 2024",
    supportedPlatforms: ["Ubuntu 24.04", "Windows 10/11", "macOS"],
  },
  {
    name: "iron",
    displayName: "Iron Irwini",
    isLTS: false,
    releaseDate: "November 2023",
    supportedPlatforms: ["Ubuntu 22.04", "Windows 10/11", "macOS"],
  },
  {
    name: "humble",
    displayName: "Humble Hawksbill",
    isLTS: true,
    releaseDate: "May 2022",
    supportedPlatforms: ["Ubuntu 22.04", "Windows 10/11", "macOS"],
  },
];

/**
 * Workspace setting key for ROS installation preference
 */
const INSTALL_ROS_PREFERENCE_KEY = "neverInstallRos";

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
 * Main function to install ROS 2 - dispatches to appropriate platform installer
 */
export async function installRos(): Promise<void> {
  try {
    // Ask user to select a distro
    const distro = await selectRosDistro();
    if (!distro) {
      return;
    }

    // Validate distro name for security
    if (!validateDistroName(distro)) {
      throw new Error(`Invalid distro name: ${distro.name}`);
    }

    extension.outputChannel.appendLine(`User selected ROS 2 distro: ${distro.name}`);

    // Detect platform and install
    if (process.platform === "linux") {
      // Check if it's Jetson (special case of Linux)
      if (await installJetson.isJetson()) {
        await installJetson.installRosJetson(distro);
      } else if (await installUbuntu.isUbuntu()) {
        // Standard Ubuntu/Debian
        await installUbuntu.installRosUbuntu(distro);
      } else {
        throw new Error(
          "ROS 2 installation on this Linux distribution is not yet supported. Only Ubuntu and Jetson are supported."
        );
      }
    } else if (process.platform === "win32") {
      await installWin.installRosWindows(distro);
    } else if (process.platform === "darwin") {
      await installMac.installRosMac(distro);
    } else {
      vscode.window.showErrorMessage(
        `ROS 2 installation is not supported on platform: ${process.platform}`
      );
    }
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(`Error during ROS installation: ${errorMessage}`);
    vscode.window.showErrorMessage(`Failed to install ROS 2: ${errorMessage}`);
  }
}

/**
 * Prompts user to select a ROS 2 distro
 */
async function selectRosDistro(): Promise<RosDistro | undefined> {
  const items = ROS2_DISTROS.map((distro) => {
    const ltsLabel = distro.isLTS ? " (LTS)" : "";
    const label = `${distro.displayName}${ltsLabel}`;
    const description = `Released: ${distro.releaseDate}`;
    const detail = `Supported platforms: ${distro.supportedPlatforms.join(", ")}`;

    return {
      label,
      description,
      detail,
      distro,
    };
  });

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: "Select a ROS 2 distribution to install",
    ignoreFocusOut: true,
  });

  return selected?.distro;
}
