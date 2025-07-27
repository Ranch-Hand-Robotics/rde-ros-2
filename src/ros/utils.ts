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
            exportEnvCommand = `cmd /c "\"${filename}\" && set"`;
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

        let processOptions: child_process.ExecOptions = {
            cwd: vscode.workspace.rootPath,
            env: env,
        };
        
        child_process.exec(exportEnvCommand, processOptions, (error, stdout, stderr) => {
            if (!error) {
                resolve(stdout.split(os.EOL).reduce((env, line) => {
                    const index = line.indexOf("=");

                    if (index !== -1) {
                        env[line.substr(0, index)] = line.substr(index + 1);
                    }
                    
                    return env;
                }, {}));
            } else {
                extension.outputChannel.appendLine(`Shell sourcing error: ${error.message}`);
                if (stderr) {
                    extension.outputChannel.appendLine(`Shell stderr: ${stderr}`);
                }
                reject(error);
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
