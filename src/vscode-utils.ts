// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as fs from "fs";
import * as child_process from "child_process";
import * as vscode from "vscode";

import * as pfs from "./promise-fs";

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
 * Checks if the current system is Ubuntu 24.04 or later.
 */
async function isUbuntu2404OrLater(): Promise<boolean> {
    try {
        const result = await new Promise<string>((resolve, reject) => {
            child_process.exec('lsb_release -r -s', (err, stdout) => {
                if (err) {
                    // Fallback: try reading /etc/os-release
                    child_process.exec('grep VERSION_ID /etc/os-release', (err2, stdout2) => {
                        if (err2) {
                            reject(err2);
                        } else {
                            const match = stdout2.match(/VERSION_ID="?([^"]+)"?/);
                            resolve(match ? match[1] : '');
                        }
                    });
                } else {
                    resolve(stdout.trim());
                }
            });
        });
        
        const versionStr = result.trim();
        if (versionStr) {
            const version = parseFloat(versionStr);
            return version >= 24.04;
        }
        
        return false;
    } catch (err) {
        // If we can't determine the version, assume it's modern
        return false;
    }
}

/**
 * Checks if the Python environment is externally managed (PEP 668).
 * This is common in Ubuntu 24.04+ and other modern Linux distributions.
 */
async function checkExternallyManagedEnvironment(pythonCmd: string, env: any): Promise<boolean> {
    try {
        // Get the Python installation directory (where stdlib is located)
        const result = await new Promise<string>((resolve, reject) => {
            child_process.exec(`${pythonCmd} -c "import sysconfig; print(sysconfig.get_path('stdlib'))"`, { env }, (err, stdout, stderr) => {
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
 * Detects if we're currently in a virtual environment and provides information about it.
 * Enhanced for Ubuntu 24.04+ compatibility.
 */
export function detectVirtualEnvironment(): { type: string | null; path: string | null; isActive: boolean } {
    const env = process.env;
    
    // Check for conda environment
    if (env.CONDA_DEFAULT_ENV || env.CONDA_PREFIX) {
        return {
            type: 'conda',
            path: env.CONDA_PREFIX || null,
            isActive: true
        };
    }
    
    // Check for Python virtual environment
    if (env.VIRTUAL_ENV) {
        return {
            type: 'venv',
            path: env.VIRTUAL_ENV,
            isActive: true
        };
    }
    
    // Check for pipenv
    if (env.PIPENV_ACTIVE) {
        return {
            type: 'pipenv',
            path: env.VIRTUAL_ENV || null,
            isActive: true
        };
    }
    
    // Check for poetry
    if (env.POETRY_ACTIVE) {
        return {
            type: 'poetry',
            path: env.VIRTUAL_ENV || null,
            isActive: true
        };
    }
    
    // Additional check for common virtual environment indicators
    if (env.PYENV_VIRTUAL_ENV) {
        return {
            type: 'pyenv',
            path: env.PYENV_VIRTUAL_ENV,
            isActive: true
        };
    }
    
    // Check if we're in a workspace with a common virtual environment directory
    if (vscode.workspace.rootPath) {
        const commonVenvPaths = [
            path.join(vscode.workspace.rootPath, '.venv'),
            path.join(vscode.workspace.rootPath, 'venv'),
            path.join(vscode.workspace.rootPath, 'env')
        ];
        
        for (const venvPath of commonVenvPaths) {
            const activateScript = path.join(venvPath, 'bin', 'activate');
            const pythonExecutable = path.join(venvPath, 'bin', 'python');
            
            // Check if this looks like a virtual environment
            if (fs.existsSync(activateScript) && fs.existsSync(pythonExecutable)) {
                return {
                    type: 'venv-detected',
                    path: venvPath,
                    isActive: false // Not currently active, but available
                };
            }
        }
    }
    
    return {
        type: null,
        path: null,
        isActive: false
    };
}

/**
 * Detects the appropriate Python and pip commands for the current environment.
 * This function tries various methods to find Python and pip commands that work
 * with virtual environments, conda environments, and system Python installations.
 * Enhanced for Ubuntu 24.04+ which uses externally-managed-environment (PEP 668).
 */
/**
 * Detects and returns the extension's virtual environment specifically for MCP server use.
 * Returns null if not found or not applicable.
 */
export function detectMcpVirtualEnvironment(extensionPath: string): { python: string; pip: string } | null {
    const isWindows = process.platform === "win32";
    const extensionVenvPath = path.join(extensionPath, '.venv');
    
    const venvPython = isWindows 
        ? path.join(extensionVenvPath, 'Scripts', 'python.exe')
        : path.join(extensionVenvPath, 'bin', 'python');
    const venvPip = isWindows
        ? path.join(extensionVenvPath, 'Scripts', 'pip.exe')
        : path.join(extensionVenvPath, 'bin', 'pip');
        
    if (fs.existsSync(venvPython)) {
        return { python: venvPython, pip: venvPip };
    }
    
    return null;
}

/**
 * Creates and ensures the extension's virtual environment exists for MCP server use only.
 * This function is specifically for MCP server requirements and should not be used elsewhere.
 */
export async function ensureMcpVirtualEnvironment(outputChannel?: vscode.OutputChannel, extensionPath?: string): Promise<boolean> {
    if (!extensionPath) {
        return false;
    }
    
    // Check if MCP virtual environment already exists
    const mcpVenv = detectMcpVirtualEnvironment(extensionPath);
    if (mcpVenv) {
        if (outputChannel) {
            outputChannel.appendLine(`MCP virtual environment already exists: ${mcpVenv.python}`);
        }
        return true;
    }
    
    const config = getExtensionConfiguration();
    const isUbuntu = process.platform === "linux" && await isUbuntu2404OrLater();
    
    // Only create virtual environment on Ubuntu 24.04+ with externally managed Python
    if (!isUbuntu) {
        return true; // Not Ubuntu 24.04+, continue with system Python
    }
    
    // Detect system Python for creating the virtual environment
    const systemPythonCommands = await detectSystemPythonCommands(process.env, outputChannel);
    const isExternallyManaged = await checkExternallyManagedEnvironment(systemPythonCommands.python, process.env);
    
    if (isExternallyManaged) {
        if (outputChannel) {
            outputChannel.appendLine("Creating MCP-specific virtual environment for Ubuntu 24.04+ externally-managed Python.");
        }
        
        const venvPath = path.join(extensionPath, '.venv');
        
        try {
            if (outputChannel) {
                outputChannel.appendLine(`Creating MCP virtual environment at: ${venvPath}`);
            }
            
            // Create the virtual environment
            await new Promise<void>((resolve, reject) => {
                child_process.exec(`${systemPythonCommands.python} -m venv ${venvPath}`, (err, stdout, stderr) => {
                    if (err) {
                        // Check if this is the python3-venv missing error
                        if (stderr && stderr.includes('python3-venv')) {
                            const pythonVersion = systemPythonCommands.python.includes('python3.') 
                                ? systemPythonCommands.python.split('python')[1] // e.g., "3.12"
                                : '3-venv';
                            
                            const installCmd = `sudo apt install python${pythonVersion === '3' ? '3-venv' : pythonVersion + '-venv'}`;
                            
                            reject(new Error(
                                `MCP virtual environment creation requires python3-venv package.\n` +
                                `Please install it using: ${installCmd}\n` +
                                `Then try enabling the MCP server again.\n\n` +
                                `Original error: ${err.message}`
                            ));
                        } else {
                            reject(new Error(`Failed to create MCP virtual environment: ${err.message}\n${stderr}`));
                        }
                    } else {
                        if (outputChannel) {
                            outputChannel.appendLine(`MCP virtual environment created successfully`);
                            if (stdout) outputChannel.appendLine(stdout);
                        }
                        resolve();
                    }
                });
            });
            
            // Ensure pip is properly installed and updated in the virtual environment
            const venvPython = path.join(venvPath, 'bin', 'python');
            if (fs.existsSync(venvPython)) {
                if (outputChannel) {
                    outputChannel.appendLine(`Ensuring pip is available in MCP virtual environment`);
                }
                
                // Use ensurepip to make sure pip is available
                await new Promise<void>((resolve, reject) => {
                    child_process.exec(`${venvPython} -m ensurepip --upgrade`, (err, stdout, stderr) => {
                        if (err) {
                            // ensurepip might fail if pip is already installed, try to upgrade it instead
                            child_process.exec(`${venvPython} -m pip install --upgrade pip`, (err2, stdout2, stderr2) => {
                                if (err2) {
                                    if (outputChannel) {
                                        outputChannel.appendLine(`Warning: Could not ensure pip in virtual environment: ${err2.message}`);
                                        if (stderr2) outputChannel.appendLine(`stderr: ${stderr2}`);
                                    }
                                    // Don't fail the entire process, just warn
                                    resolve();
                                } else {
                                    if (outputChannel) {
                                        outputChannel.appendLine(`Pip updated successfully in MCP virtual environment`);
                                        if (stdout2) outputChannel.appendLine(stdout2);
                                    }
                                    resolve();
                                }
                            });
                        } else {
                            if (outputChannel) {
                                outputChannel.appendLine(`Pip ensured in MCP virtual environment`);
                                if (stdout) outputChannel.appendLine(stdout);
                            }
                            resolve();
                        }
                    });
                });
            }
            
            if (outputChannel) {
                outputChannel.appendLine(`MCP virtual environment ready at ${venvPath}`);
            }
            
            return true;
        } catch (err) {
            const errorMessage = `Failed to create MCP virtual environment: ${err.message}`;
            if (outputChannel) {
                outputChannel.appendLine(errorMessage);
            }
            
            // Check if this is a python3-venv missing error and offer helpful guidance
            if (err.message.includes('python3-venv')) {
                const response = await vscode.window.showErrorMessage(
                    "MCP server requires python3-venv package to create a virtual environment.",
                    "Show Install Command",
                    "Cancel"
                );
                
                if (response === "Show Install Command") {
                    // Extract the install command from the error message
                    const pythonVersion = err.message.match(/python(\d+\.\d+)-venv/)?.[1] || "3";
                    const installCmd = `sudo apt install python${pythonVersion}-venv`;
                    
                    const installResponse = await vscode.window.showInformationMessage(
                        `To install the required package, run this command in your terminal:\n\n${installCmd}\n\nAfter installation, try enabling the MCP server again.`,
                        "Copy Command",
                        "OK"
                    );
                    
                    if (installResponse === "Copy Command") {
                        await vscode.env.clipboard.writeText(installCmd);
                        vscode.window.showInformationMessage("Install command copied to clipboard!");
                    }
                }
            } else {
                vscode.window.showErrorMessage(errorMessage);
            }
            
            return false;
        }
    }
    
    return true;
}

/**
 * Detects system Python commands without virtual environment interference.
 * This is used for creating virtual environments and general system operations.
 */
export async function detectSystemPythonCommands(env: any, outputChannel?: vscode.OutputChannel): Promise<{ python: string; pip: string }> {
    const isWindows = process.platform === "win32";
    const isUbuntu = !isWindows && await isUbuntu2404OrLater();
    
    // List of Python command candidates to try, in order of preference
    const pythonCandidates = isWindows 
        ? ["python", "python3", "py"]
        : ["python3", "python"];
    
    // Try to find a working Python executable
    for (const pythonCmd of pythonCandidates) {
        try {
            // Test if the Python command exists and works
            const pythonVersion = await new Promise<string>((resolve, reject) => {
                child_process.exec(`${pythonCmd} --version`, { env }, (err, stdout, stderr) => {
                    if (err) {
                        reject(err);
                    } else {
                        resolve(stdout || stderr);
                    }
                });
            });
            
            if (outputChannel) {
                outputChannel.appendLine(`Found system Python: ${pythonCmd} (${pythonVersion.trim()})`);
            }
            
            // Try to find the corresponding pip command
            const pipCandidates = [
                `${pythonCmd} -m pip`,  // Most reliable method
                `pip3`,
                `pip`
            ];
            
            // For Ubuntu 24.04+, prefer using python -m pip with appropriate flags
            if (isUbuntu) {
                const config = getExtensionConfiguration();
                const preferSystemPackages = config.get<boolean>("preferSystemPackages", false);
                
                if (preferSystemPackages) {
                    pipCandidates.unshift(`${pythonCmd} -m pip --break-system-packages`);
                } else {
                    pipCandidates.push(`${pythonCmd} -m pip --break-system-packages`);
                }
            }
            
            for (const pipCmd of pipCandidates) {
                try {
                    // Test if pip command works
                    await new Promise<void>((resolve, reject) => {
                        child_process.exec(`${pipCmd} --version`, { env }, (err, stdout) => {
                            if (err) {
                                reject(err);
                            } else {
                                if (outputChannel) {
                                    outputChannel.appendLine(`Found system pip: ${pipCmd} (${stdout.trim()})`);
                                }
                                resolve();
                            }
                        });
                    });
                    
                    return { python: pythonCmd, pip: pipCmd };
                } catch (pipErr) {
                    continue;
                }
            }
            
            // If no pip found, use python -m pip as fallback
            const config = getExtensionConfiguration();
            const preferSystemPackages = config.get<boolean>("preferSystemPackages", false);
            
            const fallbackPip = isUbuntu && preferSystemPackages
                ? `${pythonCmd} -m pip --break-system-packages`
                : `${pythonCmd} -m pip`;
            return { python: pythonCmd, pip: fallbackPip };
            
        } catch (err) {
            continue;
        }
    }
    
    // Fallback to default commands if detection fails
    const fallbackPython = isWindows ? "python" : "python3";
    const fallbackPip = `${fallbackPython} -m pip`;
    
    if (outputChannel) {
        outputChannel.appendLine(`Warning: Could not detect system Python/pip commands, using fallback: ${fallbackPython}, ${fallbackPip}`);
    }
    return { python: fallbackPython, pip: fallbackPip };
}

/**
 * Detects Python commands specifically for MCP server use.
 * Uses extension virtual environment if available, otherwise falls back to system Python.
 */
export async function detectMcpPythonCommands(env: any, outputChannel?: vscode.OutputChannel, extensionPath?: string): Promise<{ python: string; pip: string }> {
    // First check for MCP virtual environment
    if (extensionPath) {
        const mcpVenv = detectMcpVirtualEnvironment(extensionPath);
        if (mcpVenv) {
            if (outputChannel) {
                outputChannel.appendLine(`Using MCP virtual environment: ${mcpVenv.python}`);
            }
            return mcpVenv;
        }
    }
    
    // Fall back to system Python for MCP server
    return await detectSystemPythonCommands(env, outputChannel);
}

/**
 * Installs Python packages from a requirements.txt file using MCP-specific environment.
 * This function is specifically for MCP server requirements and uses the extension's virtual environment.
 */
export async function installMcpPythonRequirements(
    requirementsPath: string, 
    env: any, 
    outputChannel?: vscode.OutputChannel,
    extensionPath?: string
): Promise<void> {
    if (outputChannel) {
        outputChannel.appendLine(`Installing MCP python packages from: ${requirementsPath}`);
    }
    
    // Check if requirements file exists
    if (!await pfs.exists(requirementsPath)) {
        throw new Error(`MCP requirements file not found: ${requirementsPath}`);
    }
    
    try {
        // Use MCP-specific Python command detection
        const pythonCommands = await detectMcpPythonCommands(env, outputChannel, extensionPath);
        if (outputChannel) {
            outputChannel.appendLine(`Using MCP Python: ${pythonCommands.python}, pip: ${pythonCommands.pip}`);
        }
        
        await new Promise<void>((resolve, reject) => {
            child_process.exec(
                `${pythonCommands.pip} install -r "${requirementsPath}"`, 
                { env: env }, 
                (err, stdout, stderr) => {
                    if (err) {
                        reject(new Error(`Failed to install MCP Python packages: ${err.message}\n${stderr}`));
                    } else {
                        if (outputChannel) {
                            outputChannel.appendLine("MCP Python packages installed successfully");
                            if (stdout) outputChannel.appendLine(stdout);
                        }
                        resolve();
                    }
                }
            );
        });
    } catch (err) {
        const errorMessage = `Failed to install MCP python packages from ${requirementsPath}: ${err.message}`;
        if (outputChannel) {
            outputChannel.appendLine(errorMessage);
        }
        throw new Error(errorMessage);
    }
}

/**
 * Installs Python packages from a requirements.txt file using system Python.
 * This function handles virtual environments, conda environments, and Ubuntu 24.04+ externally-managed environments
 * but does NOT use the extension's MCP virtual environment.
 */
export async function installPythonRequirements(
    requirementsPath: string, 
    env: any, 
    outputChannel?: vscode.OutputChannel,
    extensionPath?: string
): Promise<void> {
    if (outputChannel) {
        outputChannel.appendLine(`Installing python packages from: ${requirementsPath}`);
    }
    
    // Check if requirements file exists
    if (!await pfs.exists(requirementsPath)) {
        throw new Error(`Requirements file not found: ${requirementsPath}`);
    }
    
    try {
        // Try to detect the appropriate Python and pip commands (system Python, not MCP)
        const pythonCommands = await detectSystemPythonCommands(env, outputChannel);
        if (outputChannel) {
            outputChannel.appendLine(`Using Python: ${pythonCommands.python}, pip: ${pythonCommands.pip}`);
        }
        
        await new Promise<void>((resolve, reject) => {
            child_process.exec(
                `${pythonCommands.pip} install -r "${requirementsPath}"`, 
                { env: env }, 
                (err, stdout, stderr) => {
                    if (err) {
                        reject(new Error(`Failed to install Python packages: ${err.message}\n${stderr}`));
                    } else {
                        if (outputChannel) {
                            outputChannel.appendLine(`Installed python packages: ${stdout}`);
                            if (stderr) {
                                outputChannel.appendLine(`Installation warnings: ${stderr}`);
                            }
                        }
                        resolve();
                    }
                }
            );
        });
        
        if (outputChannel) {
            outputChannel.appendLine(`Python packages installed successfully from ${requirementsPath}`);
        }
    } catch (err) {
        const errorMessage = `Failed to install python packages from ${requirementsPath}: ${err.message}`;
        if (outputChannel) {
            outputChannel.appendLine(errorMessage);
        }
        throw new Error(errorMessage);
    }
}
