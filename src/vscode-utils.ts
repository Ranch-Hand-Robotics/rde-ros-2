// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as fs from "fs";
import * as child_process from "child_process";
import * as vscode from "vscode";

import * as pfs from "./promise-fs";
import * as ros_utils from "./ros/utils";
import * as extension from "./extension";

export interface IPackageInfo {
    name: string;
    version: string;
    aiKey: string;
}

export function getPackageInfo(extensionId: string): IPackageInfo {
    const extension = vscode.extensions.getExtension(extensionId);
    const metadata = extension.packageJSON;
    if (metadata && ("name" in metadata) && ("version" in metadata) && ("aiKey" in metadata)) {
        return {
            name: metadata.name,
            version: metadata.version,
            aiKey: metadata.aiKey,
        };
    }
    return undefined;
}

export function getExtensionConfiguration(): vscode.WorkspaceConfiguration {
    const rosConfigurationName: string = "ROS2";
    return vscode.workspace.getConfiguration(rosConfigurationName);
}

/**
 * Gets the ROS setup script path from user settings with Windows default for Pixi.
 * Returns the full path to the setup script that should be sourced.
 */
export function getRosSetupScript(): string {
    const config = getExtensionConfiguration();
    let rosSetupScript = config.get("rosSetupScript", "");
    
    if (!rosSetupScript) {
        // Read the flag that allows Pixi usage on non-Windows platforms (defaults to false)
        const usePixiOnAllPlatforms = config.get("usePixiOnAllPlatforms", false);

        if (process.platform === "win32") {
            // Default to Pixi installation on Windows
            const shellInfo = ros_utils.detectUserShell();
            const setupFileName = `local_setup${shellInfo.scriptExtension}`;
            rosSetupScript = path.join("c:", "pixi_ws", `ros2-windows`, setupFileName);
        } else {
            // On Unix-like systems we currently require the user to specify the setup script path.
            // The `usePixiOnAllPlatforms` flag is reserved for future behavior changes; for now
            // return an empty string to indicate no default was found.
            if (!usePixiOnAllPlatforms) {
                return "";
            }

            // If Pixi-on-all-platforms is enabled, we still don't auto-guess a Unix path at this time.
            return "";
        }
    }
    
    // Handle workspace folder variable substitution
    const regex = /\$\{workspaceFolder\}/g;
    if (rosSetupScript.includes("${workspaceFolder}")) {
        if (vscode.workspace.workspaceFolders && vscode.workspace.workspaceFolders.length === 1) {
            rosSetupScript = rosSetupScript.replace(regex, vscode.workspace.workspaceFolders[0].uri.fsPath);
        }
    }
    
    // Normalize path separators for current platform
    return path.normalize(rosSetupScript);
}

export function createOutputChannel(): vscode.OutputChannel {
    return vscode.window.createOutputChannel("ROS 2");
}

/**
 * Shows the output panel if autoShowOutputChannel is enabled (default: true).
 * @param outputChannel The output channel to show
 */
export function showOutputPanel(outputChannel: vscode.OutputChannel): void {
    if (getExtensionConfiguration().get<boolean>("autoShowOutputChannel", true)) {
        outputChannel.show();
    }
}

/**
 * Checks if the Python environment is externally managed (PEP 668).
 * This is common in Ubuntu 24.04+ and other modern Linux distributions.
 */
async function checkExternallyManagedEnvironment(env: any): Promise<boolean> {
    try {
        // Get the Python installation directory (where stdlib is located)
        const result = await new Promise<string>((resolve, reject) => {
            child_process.exec(`python3 -c "import sysconfig; print(sysconfig.get_path('stdlib'))"`, { env }, (err, stdout, stderr) => {
                if (err) {
                    reject(err);
                } else {
                    resolve(stdout.trim());
                }
            });
        });
        
        const stdlibPath = result.trim();
        if (stdlibPath) {
            // On Ubuntu 24.04, EXTERNALLY-MANAGED is in the Python installation directory
            // e.g., /usr/lib/python3.12/EXTERNALLY-MANAGED
            const externallyManagedPath = path.join(stdlibPath, 'EXTERNALLY-MANAGED');
            const exists = await pfs.exists(externallyManagedPath);
            
            // Also check the parent directory as a fallback
            // e.g., /usr/lib/python3.12/../EXTERNALLY-MANAGED
            if (!exists) {
                const altPath = path.join(path.dirname(stdlibPath), 'EXTERNALLY-MANAGED');
                return await pfs.exists(altPath);
            }
            
            return exists;
        }
        
        return false;
    } catch (err) {
        // If we can't check, assume it's not externally managed
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
    const isExternallyManaged = await checkExternallyManagedEnvironment(process.env);
    
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
                if (await pfs.exists(requirementsPath)) {
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
 * Detects if the extension is running in Cursor (Anysphere's editor)
 * @returns true if running in Cursor, false otherwise
 */
export function isRunningInCursor(): boolean {
    // Method 1: Check for Cursor-specific environment variables
    if (process.env.CURSOR_EXTENSION_HOST || process.env.ANYSPHERE_EXTENSION_HOST) {
        return true;
    }

    // Method 2: Check for Cursor-specific process name patterns
    if (process.env.VSCODE_PID) {
        try {
            // On Linux, we can check the process name
            if (process.platform === 'linux') {
                const fs = require('fs');
                const procPath = `/proc/${process.env.VSCODE_PID}/comm`;
                if (fs.existsSync(procPath)) {
                    const processName = fs.readFileSync(procPath, 'utf8').trim();
                    if (processName.includes('cursor') || processName.includes('Cursor')) {
                        return true;
                    }
                }
            }
        } catch (error) {
            // Ignore errors in process detection
        }
    }

    // Method 3: Check for Cursor-specific extension dependencies
    // This method was removed due to webpack bundling issues with require('../../package.json')
    // The other detection methods should be sufficient for most cases

    // Method 4: Check for Cursor-specific workspace settings or configurations
    const workspaceConfig = vscode.workspace.getConfiguration();
    const cursorSpecificSettings = [
        'cursor',
        'anysphere',
        'cursorExtensionHost'
    ];
    
    for (const setting of cursorSpecificSettings) {
        if (workspaceConfig.has(setting)) {
            return true;
        }
    }

    // Method 5: Check for Cursor-specific command line arguments
    if (process.argv.some(arg => arg.toLowerCase().includes('cursor'))) {
        return true;
    }

    return false;
}
