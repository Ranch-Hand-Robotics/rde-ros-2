// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as extension from "../extension";
import { RosDistro } from "./install-ros";

/**
 * Maximum length for environment variable values in diagnostic output
 */
const MAX_ENV_VALUE_LENGTH = 500;
const MAX_INSTALL_LOG_CHARS = 15000;

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
 * Monitors a terminal for errors and offers Copilot help if errors are detected
 */
export function monitorTerminalForErrors(
  terminal: vscode.Terminal,
  distro: RosDistro,
  installLogPath?: string
): void {
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
export async function offerCopilotHelp(distro: RosDistro, installLogPath?: string): Promise<void> {
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
    const os = require("os");
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
