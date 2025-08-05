// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as pfs from "../promise-fs";
import * as telemetry from "../telemetry-helper";

/**
 * Detected shell information
 */
export interface ShellInfo {
    name: string;
    executable: string;
    scriptExtension: string;
    sourceCommand: string;
}

/**
 * Detects the user's shell on Linux/macOS systems
 */
export function detectUserShell(): ShellInfo {
    if (process.platform === "win32") {
        return {
            name: "cmd",
            executable: "cmd",
            scriptExtension: ".bat",
            sourceCommand: "call"
        };
    }

    // Get shell from environment or fallback to bash
    let shellPath = process.env.SHELL || "/bin/bash";
    
    // Extract shell name from path
    const shellName = shellPath.split("/").pop() || "bash";
    
    // Map shell to appropriate configuration
    switch (shellName) {
        case "zsh":
            return {
                name: "zsh",
                executable: shellPath,
                scriptExtension: ".zsh",
                sourceCommand: "source"
            };
        case "fish":
            return {
                name: "fish",
                executable: shellPath,
                scriptExtension: ".fish",
                sourceCommand: "source"
            };
        case "dash":
        case "sh":
            return {
                name: "sh",
                executable: shellPath,
                scriptExtension: ".sh",
                sourceCommand: "."
            };
        case "tcsh":
        case "csh":
            return {
                name: "csh",
                executable: shellPath,
                scriptExtension: ".csh",
                sourceCommand: "source"
            };
        case "bash":
        default:
            // Default to bash for unknown shells or bash itself
            return {
                name: "bash",
                executable: shellPath,
                scriptExtension: ".bash",
                sourceCommand: "source"
            };
    }
}

/**
 * Finds Visual Studio installations by reading from the Windows registry
 */
function findVisualStudioInstallations(): string[] {
    if (process.platform !== "win32") {
        return [];
    }

    const installations: string[] = [];
    const child_process = require("child_process");

    try {
        const vswhereCmd = '"C:\\Program Files (x86)\\Microsoft Visual Studio\\Installer\\vswhere.exe" -all -property installationPath';
        const vswhereResult = child_process.execSync(vswhereCmd, { 
            encoding: 'utf8', 
            timeout: 5000,
            windowsHide: true 
        });
        
        const installPaths = vswhereResult.trim().split('\n').filter(path => path.trim());
        for (const installPath of installPaths) {
            const vcvarsPath = `${installPath.trim()}\\VC\\Auxiliary\\Build\\vcvarsall.bat`;
            const fs = require("fs");
            if (fs.existsSync(vcvarsPath)) {
                installations.push(vcvarsPath);
                extension.outputChannel.appendLine(`Found VS installation via vswhere: ${vcvarsPath}`);
            }
        }
    } catch (vswhereError) {
        extension.outputChannel.appendLine(`vswhere.exe not available: ${vswhereError.message}`);
    }

    // Remove duplicates and return
    return [...new Set(installations)];
}

/**
 * Gets the appropriate setup script extension for the detected shell
 */
export function getSetupScriptExtension(): string {
    return detectUserShell().scriptExtension;
}

/**
 * Executes a setup file and returns the resulting env.
 */
export function sourceSetupFile(filename: string, env?: any): Promise<any> {
    return new Promise((resolve, reject) => {
        let exportEnvCommand: string;
        
        if (process.platform === "win32") {
            // On Windows, create a composite environment by sourcing multiple setup scripts
            // Use a temporary batch file to avoid command line length limitations
            const path = require("path");
            const fs = require("fs");
            const os = require("os");
            
            const tempDir = os.tmpdir();
            const tempBatchFile = path.join(tempDir, `ros_env_setup_${Date.now()}.bat`);
            
            const setupCommands: string[] = [
                "@echo off",
                "REM Composite ROS 2 environment setup script"
            ];
            
            // 1. Visual Studio console environment (vcvarsall.bat)
            const vsInstallations = findVisualStudioInstallations();
            
            // Add VS environment setup if available (only call the first one found)
            setupCommands.push("REM Setup Visual Studio environment");
            for (const vsPath of vsInstallations) {
                setupCommands.push(`if exist "${vsPath}" (`);
                setupCommands.push(`    call "${vsPath}" x64`);
                setupCommands.push(`    goto :vs_done`);
                setupCommands.push(`)`);
            }
            setupCommands.push(":vs_done");
            
            // 2. Pixi shell from c:\pixi_ws
            setupCommands.push("REM Setup Pixi environment");
            setupCommands.push(`pixi shell-hook`);
            
            // 3. Local setup from c:\pixi_ws\ros2-windows
            setupCommands.push("REM Setup ROS2 Windows environment");
            setupCommands.push(`if exist "c:\\pixi_ws\\ros2-windows\\local_setup.bat" call "c:\\pixi_ws\\ros2-windows\\local_setup.bat"`);
            
            // 4. Finally, source the requested setup file
            setupCommands.push("REM Setup target file");
            setupCommands.push(`call "${filename}"`);
            
            // 5. Output environment variables
            setupCommands.push("REM Output environment");
            setupCommands.push("set");
            
            // Write the batch file
            const batchContent = setupCommands.join("\r\n");
            
            try {
                fs.writeFileSync(tempBatchFile, batchContent);
                exportEnvCommand = `cmd /c "${tempBatchFile}"`;
                
                extension.outputChannel.appendLine(`Created temporary batch file: ${tempBatchFile}`);
                extension.outputChannel.appendLine(`Batch content:\n${batchContent}`);
            } catch (writeError) {
                extension.outputChannel.appendLine(`Failed to create temporary batch file: ${writeError}`);
                // Fallback to simple approach
                exportEnvCommand = `cmd /c "call "${filename}" && set"`;
            }
        } else {
            const shellInfo = detectUserShell();
            
            // Special handling for different shells
            switch (shellInfo.name) {
                case "fish":
                    // Fish shell has different syntax
                    exportEnvCommand = `${shellInfo.executable} -c "${shellInfo.sourceCommand} '${filename}'; and env"`;
                    break;
                case "csh":
                case "tcsh":
                    // C shell family
                    exportEnvCommand = `${shellInfo.executable} -c "${shellInfo.sourceCommand} '${filename}' && env"`;
                    break;
                default:
                    // Bash, zsh, sh, and other POSIX-compatible shells
                    // Force login shell for ROS compatibility in containers
                    exportEnvCommand = `${shellInfo.executable} --login -c "${shellInfo.sourceCommand} '${filename}' && env"`;
                    break;
            }
            
            extension.outputChannel.appendLine(`Sourcing Environment using ${shellInfo.name}: ${exportEnvCommand}`);
        }

        const processOptions: child_process.ExecOptions = {
            cwd: vscode.workspace.rootPath,
            env: env,
            // Increase timeout for complex Windows setup chains
            timeout: 60000,
            // Set max buffer size to handle large environment outputs
            maxBuffer: 1024 * 1024, // 1MB
        };
        
        child_process.exec(exportEnvCommand, processOptions, (error, stdout, stderr) => {
            // Clean up temporary batch file on Windows
            if (process.platform === "win32" && exportEnvCommand.includes("ros_env_setup_")) {
                const fs = require("fs");
                try {
                    const tempFile = exportEnvCommand.match(/"([^"]*ros_env_setup_[^"]*\.bat)"/)?.[1];
                    if (tempFile) {
                        fs.unlinkSync(tempFile);
                        extension.outputChannel.appendLine(`Cleaned up temporary batch file: ${tempFile}`);
                    }
                } catch (cleanupError) {
                    extension.outputChannel.appendLine(`Failed to cleanup temporary file: ${cleanupError}`);
                }
            }
            
            if (error) {
                extension.outputChannel.appendLine(`Shell sourcing error: ${error.message}`);
                if (stderr) {
                    extension.outputChannel.appendLine(`Shell stderr: ${stderr}`);
                }
                reject(error);
                return;
            }

            try {
                // Parse environment variables with better error handling
                const parsedEnv = stdout
                    .split(os.EOL)
                    .filter(line => line.trim().length > 0) // Filter empty lines
                    .reduce((envObj: Record<string, string>, line: string) => {
                        const equalIndex = line.indexOf("=");
                        
                        // Skip lines that don't contain environment variables
                        if (equalIndex === -1 || equalIndex === 0) {
                            return envObj;
                        }
                        
                        const key = line.substring(0, equalIndex).trim();
                        const value = line.substring(equalIndex + 1);
                        
                        // Skip empty keys or keys with spaces (invalid env vars)
                        if (key && !key.includes(" ")) {
                            envObj[key] = value;
                        }
                        
                        return envObj;
                    }, {});
                
                extension.outputChannel.appendLine(`Successfully parsed ${Object.keys(parsedEnv).length} environment variables`);
                resolve(parsedEnv);
                
            } catch (parseError) {
                extension.outputChannel.appendLine(`Failed to parse environment variables: ${parseError}`);
                reject(parseError);
            }
        });
    });
}

export function xacro(filename: string): Promise<any> {
    return new Promise((resolve, reject) => {
        let processOptions = {
            cwd: vscode.workspace.rootPath,
            env: extension.env,
            windowsHide: false,
        };

        let xacroCommand: string;
        if (process.platform === "win32") {
            xacroCommand = `cmd /c "xacro "${filename}""`;
        } else {
            const shellInfo = detectUserShell();
            xacroCommand = `${shellInfo.executable} --login -c "xacro '${filename}'"`;
        }

        child_process.exec(xacroCommand, processOptions, (error, stdout, _stderr) => {
            if (!error) {
                resolve(stdout);
            } else {
                reject(error);
            }
        });
    });
}

/**
 * Gets the names of installed distros.
 */
export function getDistros(): Promise<string[]> {
    try {
        return pfs.readdir("/opt/ros");
    } catch (error) {
        return Promise.resolve([]);
    }
}

/**
 * Creates and shows a ROS-sourced terminal.
 */
export function createTerminal(context: vscode.ExtensionContext): vscode.Terminal {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.CreateTerminal);

    const terminal = vscode.window.createTerminal({ name: 'ros2', env: extension.env })
    terminal.show();

    return terminal;
}
