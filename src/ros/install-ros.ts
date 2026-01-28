// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as os from "os";
import * as child_process from "child_process";

import * as vscode_utils from "../vscode-utils";
import * as extension from "../extension";

/**
 * Maximum length for environment variable values in diagnostic output
 */
const MAX_ENV_VALUE_LENGTH = 500;

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
 * Main function to install ROS 2
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

    // Install based on platform
    if (process.platform === "linux") {
      await installRosLinux(distro);
    } else if (process.platform === "win32" || process.platform === "darwin") {
      await installRosPixi(distro);
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

/**
 * Checks if Pixi is installed on the system
 */
async function isPixiInstalled(): Promise<boolean> {
  return new Promise((resolve) => {
    const cmd = process.platform === "win32" ? "where pixi" : "which pixi";
    child_process.exec(cmd, (error) => {
      resolve(!error);
    });
  });
}

/**
 * Installs Pixi on Windows or macOS
 */
async function installPixi(): Promise<boolean> {
  const choice = await vscode.window.showWarningMessage(
    "Pixi package manager is required to install ROS 2 on this platform but is not currently installed. " +
    "This will download and execute an installation script from pixi.sh. Would you like to proceed?",
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
    let installCommand: string;
    if (process.platform === "win32") {
      installCommand =
        'powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"';
    } else if (process.platform === "darwin") {
      installCommand = "curl -fsSL https://pixi.sh/install.sh | sh";
    } else {
      throw new Error("Pixi installation is only supported on Windows and macOS");
    }

    await new Promise<void>((resolve, reject) => {
      const proc = child_process.exec(installCommand, (error, stdout, stderr) => {
        if (error) {
          extension.outputChannel.appendLine(`Pixi installation error: ${error.message}`);
          extension.outputChannel.appendLine(`stderr: ${stderr}`);
          reject(error);
        } else {
          extension.outputChannel.appendLine(`Pixi installed successfully`);
          extension.outputChannel.appendLine(`stdout: ${stdout}`);
          resolve();
        }
      });

      proc.stdout?.on("data", (data) => {
        extension.outputChannel.append(data.toString());
      });

      proc.stderr?.on("data", (data) => {
        extension.outputChannel.append(data.toString());
      });
    });

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
async function installRosLinux(distro: RosDistro): Promise<void> {
  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} on Linux using APT...`);
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
 * Installs ROS 2 using Pixi on Windows or macOS
 */
async function installRosPixi(distro: RosDistro): Promise<void> {
  // Check if Pixi is installed
  const pixiInstalled = await isPixiInstalled();

  if (!pixiInstalled) {
    const installed = await installPixi();
    if (!installed) {
      return;
    }
  }

  extension.outputChannel.appendLine(`Installing ROS 2 ${distro.name} using Pixi...`);
  extension.outputChannel.show();

  // Get or create pixi workspace directory with platform-aware default
  const config = vscode_utils.getExtensionConfiguration();
  const defaultPixiRoot =
    process.platform === "win32" ? "c:\\pixi_ws" : path.join(os.homedir(), "pixi_ws");
  const pixiRoot = config.get<string>("pixiRoot", defaultPixiRoot);

  // Create the directory (and any missing parents) if it doesn't exist
  try {
    await fs.promises.mkdir(pixiRoot, { recursive: true });
  } catch (err) {
    const errorMessage = err instanceof Error ? err.message : String(err);
    extension.outputChannel.appendLine(`Failed to create Pixi root directory "${pixiRoot}": ${errorMessage}`);
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
    "# Installing ROS 2 via Pixi",
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

  // Monitor for errors
  monitorTerminalForErrors(terminal, distro);
}

/**
 * Monitors a terminal for errors and offers Copilot help if errors are detected
 */
function monitorTerminalForErrors(terminal: vscode.Terminal, distro: RosDistro): void {
  // Set up terminal exit handler
  const disposable = vscode.window.onDidCloseTerminal((closedTerminal) => {
    if (closedTerminal === terminal) {
      disposable.dispose();

      // Check if there were errors in the output
      const exitCode = closedTerminal.exitStatus?.code;

      if (exitCode === 0) {
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
        // Terminal did not report a successful exit; installation may have failed or been interrupted
        const message =
          exitCode !== undefined
            ? `ROS 2 installation may have encountered errors (exit code: ${exitCode}). Would you like help diagnosing the issue?`
            : "ROS 2 installation terminal was closed. The installation may not have completed successfully. Would you like help diagnosing potential issues?";

        vscode.window
          .showErrorMessage(message, "Get Copilot Help", "Dismiss")
          .then((choice) => {
            if (choice === "Get Copilot Help") {
              offerCopilotHelp(distro);
            }
          });
      }
    }
  });
}

/**
 * Offers Copilot help for diagnosing installation issues
 */
async function offerCopilotHelp(distro: RosDistro): Promise<void> {
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

**Installation Method:**
${process.platform === "linux" ? "APT package manager on Linux" : "Pixi package manager"}

**Note:** Terminal output is not automatically captured. Please check the terminal history for error messages.

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
