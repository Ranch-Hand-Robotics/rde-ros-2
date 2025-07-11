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

export function createOutputChannel(): vscode.OutputChannel {
    return vscode.window.createOutputChannel("ROS 2");
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
