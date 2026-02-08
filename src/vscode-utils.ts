// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as fs from "fs";
import { promises as fsPromises } from "fs";
import * as child_process from "child_process";
import * as vscode from "vscode";

import * as ros_utils from "./ros/utils";
import * as extension from "./extension";

import { 
    checkExternallyManagedEnvironment,
    getPackageInfo as commonGetPackageInfo,
    isLldbExtensionInstalled as commonIsLldbInstalled,
    isCppToolsExtensionInstalled as commonIsCppToolsInstalled,
    isPythonExtensionInstalled as commonIsPythonInstalled,
    isCursorEditor as commonIsCursorEditor,
    createOutputChannel as commonCreateOutputChannel,
    showOutputPanelIfConfigured,
    IPackageInfo as CommonIPackageInfo
} from "@ranchhandrobotics/rde-common";

export type IPackageInfo = CommonIPackageInfo;

export function getExtensionConfiguration(): vscode.WorkspaceConfiguration {
    const rosConfigurationName: string = "ROS2";
    return vscode.workspace.getConfiguration(rosConfigurationName);
}

/**
 * Gets the workspace folder that contains the given path.
 * Returns the workspace folder path or null if not in a workspace.
 * This function ensures workspace boundary enforcement across the extension.
 */
export function getWorkspaceFolder(dirPath: string): string | null {
    if (!vscode.workspace.workspaceFolders || vscode.workspace.workspaceFolders.length === 0) {
        return null;
    }
    
    const normalizedPath = path.normalize(dirPath);
    
    // Find the workspace folder that contains this path
    for (const folder of vscode.workspace.workspaceFolders) {
        const folderPath = path.normalize(folder.uri.fsPath);
        
        if (normalizedPath === folderPath || normalizedPath.startsWith(folderPath + path.sep)) {
            return folderPath;
        }
    }
    
    return null;
}

/**
 * Gets the ROS setup script path from user settings with Windows default for Pixi.
 * Returns the full path to the setup script that should be sourced.
 */
export function getRosSetupScript(): string {
    const config = getExtensionConfiguration();
    let rosSetupScript = config.get("rosSetupScript", "");
    
    // First, handle workspace folder variable substitution if present
    const regex = /\$\{workspaceFolder\}/g;
    if (rosSetupScript.includes("${workspaceFolder}")) {
        if (vscode.workspace.workspaceFolders && vscode.workspace.workspaceFolders.length >= 1) {
            const wsFolder = vscode.workspace.workspaceFolders[0].uri.fsPath;
            rosSetupScript = rosSetupScript.replace(regex, wsFolder);
        } else {
            // If workspace folder variable is present but no workspace is open, return empty
            return "";
        }
    }
    
    // If still empty after substitution, check for pixiRoot default
    if (!rosSetupScript) {
        // If pixiRoot is configured, use it on any platform
        const pixiRoot = config.get("pixiRoot", "");
        
        if (pixiRoot) {
            const shellInfo = ros_utils.detectUserShell();
            const setupFileName = `local_setup${shellInfo.scriptExtension}`;
            const pixiRosPath = process.platform === "win32"
                ? path.join(pixiRoot, "ros2-windows")
                : pixiRoot;
            rosSetupScript = path.join(pixiRosPath, setupFileName);
            console.log('[getRosSetupScript] Using pixi path:', rosSetupScript);
        } else {
            // No pixiRoot configured - return empty string to indicate no default
            return "";
        }
    }
    
    // Normalize path separators for current platform
    const normalized = path.normalize(rosSetupScript);
    return normalized;
}

export function createOutputChannel(): vscode.OutputChannel {
    return commonCreateOutputChannel("ROS 2");
}

/**
 * Shows the output panel if autoShowOutputChannel is enabled (default: true).
 * @param outputChannel The output channel to show
 */
export function showOutputPanel(outputChannel: vscode.OutputChannel): void {
    showOutputPanelIfConfigured(outputChannel, "ROS2", "autoShowOutputChannel");
}

/**
 * Checks if the Python environment is externally managed (PEP 668).
 * This is common in Ubuntu 24.04+ and other modern Linux distributions.
 */
export async function checkExternallyManagedEnvironmentInternal(env: any): Promise<boolean> {
    return checkExternallyManagedEnvironment(env);
}

/**
 * Check if a file or directory exists.
 */
async function exists(filePath: string): Promise<boolean> {
    try {
        await fsPromises.access(filePath);
        return true;
    } catch {
        return false;
    }
}

/**
 * Creates and ensures the extension's virtual environment exists for MCP server use only.
 * This function is specifically for MCP server requirements and should not be used elsewhere.
 */
export async function ensureMcpVirtualEnvironment(context: vscode.ExtensionContext, outputChannel?: vscode.OutputChannel, extensionPath?: string): Promise<boolean> {
    if (!extensionPath) {
        throw new Error("Extension path is required to create MCP virtual environment.");
    }
    
    let terminal: vscode.Terminal | undefined = undefined;

    // Detect system Python for creating the virtual environment
    const isExternallyManaged = await checkExternallyManagedEnvironmentInternal(process.env);
    
    if (isExternallyManaged) {
        outputChannel?.appendLine("Creating MCP-specific virtual environment for Ubuntu 24.04+ externally-managed Python.");
        
        const venvPath = path.join(extensionPath, '.venv');
        
        try {
            outputChannel?.appendLine(`Creating MCP virtual environment at: ${venvPath}`);
            
            // Create the virtual environment
            let envAvailable = await new Promise<boolean>((resolve, reject) => {
                child_process.exec(`python3 -m venv ${venvPath}`, async (err, stdout, stderr) => {
                    if (err) {
                        // Check if this is the python3-venv missing error
                        if (err && err.message.includes('failed')) {
                            const question = `MCP virtual environment creation requires python3-venv package.\n Would you lke to install it?`;

                            const selection = await vscode.window.showInformationMessage(
                                question,
                                'Install Now',
                                'Cancel'
                            );

                            if (selection === 'Install Now') {
                                // Install python3-venv using MCP terminal
                                terminal = extension.getMcpTerminal();
                                terminal.sendText("sudo apt update && sudo apt install -y python3-venv");
                                
                                vscode.window.showInformationMessage(
                                    "Installing python3-venv package. Please check the terminal for progress and restart MCP server after installation completes."
                                );
                                resolve(false);
                            } else {
                                reject(new Error(`Failed to create MCP virtual environment: ${err.message}\n${stderr}`));
                            }
                        }
                    } else {
                        outputChannel?.appendLine(`MCP virtual environment created successfully`);
                        if (stdout) outputChannel?.appendLine(stdout);

                        resolve(true);
                    }
                });
            });

            // Ensure pip is properly installed and updated in the virtual environment
            const venvPython = path.join(venvPath, 'bin', 'python3');
            if (fs.existsSync(venvPython)) {
                outputChannel?.appendLine(`Ensuring pip is available in MCP virtual environment`);

                if (!terminal) {
                    // Create a terminal if not already created
                    terminal = extension.getMcpTerminal();
                    terminal.sendText(`source ${path.join(venvPath, 'bin', 'activate')}`);
                }

                terminal.sendText(`${venvPython} -m ensurepip --upgrade`);

                // Install requirements.txt from extension assets if it exists
                const requirementsPath = path.join(extensionPath, 'assets', 'scripts', 'requirements.txt');
                if (await exists(requirementsPath)) {
                    outputChannel?.appendLine(`Installing MCP requirements from extension assets`);
                    
                    const venvPip = path.join(venvPath, 'bin', 'pip3');
                    terminal.sendText(`${venvPip} install -r "${requirementsPath}"`);
                }
            }
            
            outputChannel?.appendLine(`MCP virtual environment ready at ${venvPath}`);

            if (!envAvailable) {
                return false; // Have the user check progress.
            }

            return true;
        } catch (err) {
            const errorMessage = `Failed to create MCP virtual environment: ${err.message}`;
            outputChannel?.appendLine(errorMessage);
            vscode.window.showErrorMessage(errorMessage);
            return false;
        }
    }
    
    return true;
}

/**
 * Extracts package metadata from an extension's package.json
 */
export function getPackageInfo(extensionId: string): IPackageInfo | undefined {
    return commonGetPackageInfo(extensionId);
}

/**
 * Detects if the vadimcn.vscode-lldb extension is installed
 * @returns true if the LLDB extension is installed, false otherwise
 */
export function isLldbExtensionInstalled(): boolean {
    return commonIsLldbInstalled();
}

/**
 * Detects if the Microsoft C/C++ extension is installed
 * @returns true if the C/C++ extension is installed, false otherwise
 */
export function isCppToolsExtensionInstalled(): boolean {
    return commonIsCppToolsInstalled();
}

/**
 * Detects if the Microsoft Python extension is installed
 * @returns true if the Python extension is installed, false otherwise
 */
export function isPythonExtensionInstalled(): boolean {
    return commonIsPythonInstalled();
}

/**
 * Detects if running in Cursor editor
 * @returns true if running in Cursor, false otherwise
 */
export function isCursorEditor(): boolean {
    return commonIsCursorEditor();
}
