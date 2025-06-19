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
    const rosConfigurationName: string = "ros";
    return vscode.workspace.getConfiguration(rosConfigurationName);
}

export function createOutputChannel(): vscode.OutputChannel {
    return vscode.window.createOutputChannel("ROS");
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
        const result = await new Promise<string>((resolve, reject) => {
            child_process.exec(`${pythonCmd} -c "import sysconfig; print(sysconfig.get_path('stdlib'))"`, { env }, (err, stdout) => {
                if (err) {
                    reject(err);
                } else {
                    resolve(stdout.trim());
                }
            });
        });
        
        const stdlibPath = result.trim();
        if (stdlibPath) {
            // Check for EXTERNALLY-MANAGED file
            const externallyManagedPath = path.join(path.dirname(stdlibPath), 'EXTERNALLY-MANAGED');
            return await pfs.exists(externallyManagedPath);
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
export async function detectPythonCommands(env: any, outputChannel?: vscode.OutputChannel): Promise<{ python: string; pip: string }> {
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
                outputChannel.appendLine(`Found Python: ${pythonCmd} (${pythonVersion.trim()})`);
            }
            
            // Check if we're in a virtual environment or if system is externally managed
            const venvInfo = detectVirtualEnvironment();
            const isExternallyManaged = isUbuntu && !venvInfo.isActive && await checkExternallyManagedEnvironment(pythonCmd, env);
            
            if (isExternallyManaged && outputChannel) {
                outputChannel.appendLine(`Detected Ubuntu 24.04+ with externally-managed Python environment. Virtual environment recommended.`);
            }
            
            // Try to find the corresponding pip command
            const pipCandidates = [
                `${pythonCmd} -m pip`,  // Most reliable method, works in venv and system
                `pip3`,
                `pip`
            ];
            
            // For Ubuntu 24.04+, prefer using python -m pip with appropriate flags
            if (isUbuntu && !venvInfo.isActive) {
                const config = getExtensionConfiguration();
                const preferSystemPackages = config.get<boolean>("preferSystemPackages", false);
                
                if (preferSystemPackages) {
                    pipCandidates.unshift(`${pythonCmd} -m pip --break-system-packages`);
                } else {
                    // Still add it as an option, but lower priority
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
                                    outputChannel.appendLine(`Found pip: ${pipCmd} (${stdout.trim()})`);
                                }
                                resolve();
                            }
                        });
                    });
                    
                    return { python: pythonCmd, pip: pipCmd };
                } catch (pipErr) {
                    // Continue to next pip candidate
                    continue;
                }
            }
            
            // If no pip found, use python -m pip as fallback with appropriate flags
            const config = getExtensionConfiguration();
            const preferSystemPackages = config.get<boolean>("preferSystemPackages", false);
            
            const fallbackPip = isUbuntu && !venvInfo.isActive && preferSystemPackages
                ? `${pythonCmd} -m pip --break-system-packages`
                : `${pythonCmd} -m pip`;
            return { python: pythonCmd, pip: fallbackPip };
            
        } catch (err) {
            // Continue to next Python candidate
            continue;
        }
    }
    
    // Fallback to default commands if detection fails
    const fallbackPython = isWindows ? "python" : "python3";
    const fallbackPip = `${fallbackPython} -m pip`;
    
    if (outputChannel) {
        outputChannel.appendLine(`Warning: Could not detect Python/pip commands, using fallback: ${fallbackPython}, ${fallbackPip}`);
    }
    return { python: fallbackPython, pip: fallbackPip };
}

/**
 * Installs Python packages from a requirements.txt file using the appropriate Python environment.
 * This function handles virtual environments, conda environments, and Ubuntu 24.04+ externally-managed environments.
 * 
 * @param requirementsPath - Path to the requirements.txt file
 * @param env - Environment variables to use for the installation
 * @param outputChannel - Optional output channel for logging
 * @returns Promise that resolves when installation is complete
 */
export async function installPythonRequirements(
    requirementsPath: string, 
    env: any, 
    outputChannel?: vscode.OutputChannel
): Promise<void> {
    if (outputChannel) {
        outputChannel.appendLine(`Installing python packages from: ${requirementsPath}`);
    }
    
    // Check if requirements file exists
    if (!await pfs.exists(requirementsPath)) {
        throw new Error(`Requirements file not found: ${requirementsPath}`);
    }
    
    try {
        // Try to detect the appropriate Python and pip commands
        const pythonCommands = await detectPythonCommands(env, outputChannel);
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

/**
 * Suggests creating a virtual environment for Ubuntu 24.04+ systems.
 */
async function suggestVirtualEnvironment(pythonCmd: string, env: any): Promise<string | null> {
    try {
        // Check if python3-venv is available
        const hasVenv = await new Promise<boolean>((resolve) => {
            child_process.exec(`${pythonCmd} -m venv --help`, { env }, (err) => {
                resolve(!err);
            });
        });
        
        if (hasVenv) {
            const workspaceRoot = vscode.workspace.rootPath || process.cwd();
            const venvPath = path.join(workspaceRoot, '.venv');
            
            return `Create a virtual environment with: ${pythonCmd} -m venv ${venvPath} && source ${venvPath}/bin/activate`;
        }
        
        return `Install python3-venv with: sudo apt install python3-venv, then create a virtual environment`;
    } catch (err) {
        return null;
    }
}

/**
 * Creates a virtual environment automatically if running on Ubuntu 24.04+ 
 * and no virtual environment is detected, but only if user consents.
 */
export async function ensureVirtualEnvironmentForUbuntu2404(outputChannel?: vscode.OutputChannel): Promise<boolean> {
    const config = getExtensionConfiguration();
    const autoCreateVenv = config.get<boolean>("autoCreateVirtualEnv", true);
    const preferSystemPackages = config.get<boolean>("preferSystemPackages", false);
    
    const isUbuntu = process.platform === "linux" && await isUbuntu2404OrLater();
    if (!isUbuntu) {
        return true; // Not Ubuntu 24.04+, continue as normal
    }
    
    const venvInfo = detectVirtualEnvironment();
    if (venvInfo.isActive || venvInfo.type === 'venv-detected') {
        return true; // Already have a virtual environment
    }
    
    // Check if system Python is externally managed
    const pythonCommands = await detectPythonCommands(process.env, outputChannel);
    const isExternallyManaged = await checkExternallyManagedEnvironment(pythonCommands.python, process.env);
    
    if (isExternallyManaged) {
        if (outputChannel) {
            outputChannel.appendLine("Ubuntu 24.04+ detected with externally-managed Python environment.");
        }
        
        if (preferSystemPackages) {
            if (outputChannel) {
                outputChannel.appendLine("User preference set to use system packages with --break-system-packages");
            }
            return true;
        }
        
        if (!autoCreateVenv) {
            if (outputChannel) {
                outputChannel.appendLine("Auto-creation of virtual environments is disabled in settings");
            }
            return true;
        }
        
        // Ask user if they want to create a virtual environment
        const response = await vscode.window.showWarningMessage(
            "Ubuntu 24.04+ uses externally-managed Python environments. Would you like to create a virtual environment for this workspace?",
            "Create Virtual Environment",
            "Continue Anyway",
            "Cancel"
        );
        
        if (response === "Create Virtual Environment") {
            const workspaceRoot = vscode.workspace.rootPath;
            if (workspaceRoot) {
                const venvPath = path.join(workspaceRoot, '.venv');
                
                try {
                    if (outputChannel) {
                        outputChannel.appendLine(`Creating virtual environment at: ${venvPath}`);
                    }
                    
                    await new Promise<void>((resolve, reject) => {
                        child_process.exec(`${pythonCommands.python} -m venv ${venvPath}`, (err, stdout, stderr) => {
                            if (err) {
                                reject(new Error(`Failed to create virtual environment: ${err.message}\n${stderr}`));
                            } else {
                                if (outputChannel) {
                                    outputChannel.appendLine(`Virtual environment created successfully`);
                                    outputChannel.appendLine(stdout);
                                }
                                resolve();
                            }
                        });
                    });
                    
                    // Show instructions to activate
                    vscode.window.showInformationMessage(
                        `Virtual environment created at ${venvPath}. Please restart VS Code or run: source ${venvPath}/bin/activate`
                    );
                    
                    return true;
                } catch (err) {
                    const errorMessage = `Failed to create virtual environment: ${err.message}`;
                    if (outputChannel) {
                        outputChannel.appendLine(errorMessage);
                    }
                    vscode.window.showErrorMessage(errorMessage);
                    return false;
                }
            }
        } else if (response === "Cancel") {
            return false;
        }
        // If "Continue Anyway", we'll proceed with system Python and --break-system-packages
    }
    
    return true;
}

/**
 * Enhances the environment with virtual environment awareness.
 * This ensures that virtual environments are properly detected and their paths are included.
 * Enhanced for Ubuntu 24.04+ compatibility with externally-managed environments.
 */
export function enhanceEnvironmentForVirtualEnv(baseEnv: any, outputChannel?: vscode.OutputChannel): any {
    const venvInfo = detectVirtualEnvironment();
    
    if (venvInfo.isActive && venvInfo.path) {
        if (outputChannel) {
            outputChannel.appendLine(`Detected active ${venvInfo.type} environment at: ${venvInfo.path}`);
        }
        
        // Ensure the virtual environment's bin/Scripts directory is in PATH
        const envCopy = { ...baseEnv };
        const pathSeparator = process.platform === "win32" ? ";" : ":";
        const binDir = process.platform === "win32" 
            ? path.join(venvInfo.path, "Scripts")
            : path.join(venvInfo.path, "bin");
            
        if (envCopy.PATH) {
            if (!envCopy.PATH.split(pathSeparator).includes(binDir)) {
                envCopy.PATH = `${binDir}${pathSeparator}${envCopy.PATH}`;
            }
        } else {
            envCopy.PATH = binDir;
        }
        
        // Set virtual environment variables if not already set
        if (venvInfo.type === 'venv' && !envCopy.VIRTUAL_ENV) {
            envCopy.VIRTUAL_ENV = venvInfo.path;
        }
        
        return envCopy;
    } else if (venvInfo.type === 'venv-detected' && venvInfo.path) {
        // Found an inactive virtual environment - suggest activation
        if (outputChannel) {
            outputChannel.appendLine(`Found inactive virtual environment at: ${venvInfo.path}`);
            outputChannel.appendLine(`To activate it, run: source ${path.join(venvInfo.path, 'bin', 'activate')}`);
        }
        
        // Optionally, we could auto-activate it by modifying the environment
        const envCopy = { ...baseEnv };
        const pathSeparator = process.platform === "win32" ? ";" : ":";
        const binDir = path.join(venvInfo.path, "bin");
        
        if (envCopy.PATH) {
            if (!envCopy.PATH.split(pathSeparator).includes(binDir)) {
                envCopy.PATH = `${binDir}${pathSeparator}${envCopy.PATH}`;
            }
        } else {
            envCopy.PATH = binDir;
        }
        
        envCopy.VIRTUAL_ENV = venvInfo.path;
        if (outputChannel) {
            outputChannel.appendLine(`Auto-activating detected virtual environment for this session`);
        }
        
        return envCopy;
    }
    
    return baseEnv;
}
