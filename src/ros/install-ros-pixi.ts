// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as extension from "../extension";
import * as vscode from "vscode";

/**
 * Checks if Pixi is installed on the system
 */
export async function isPixiInstalled(): Promise<boolean> {
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
export async function installPixi(): Promise<boolean> {
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
    let installCommand: string;
    if (process.platform === "win32") {
      installCommand = "winget install prefix-dev.pixi";
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
