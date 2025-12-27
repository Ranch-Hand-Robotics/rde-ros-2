// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as util from "util";
import * as fs from "fs";
import * as path from "path";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import * as vscode_utils from "../vscode-utils";

export async function oneTimePromiseFromEvent(eventCall, filter = undefined) : Promise<any> {
    return new Promise(resolve => {
        let disposable: vscode.Disposable;
        disposable = eventCall(event => {
            if (filter && !filter(event)) {
                return;
            }

            disposable.dispose();
            resolve(event);
        });
    });
}

/**
 * Gets stringified settings to pass to the debug server.
 */
export async function getDebugSettings(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.GetDebugSettings);

    return JSON.stringify({ env: extension.env });
}

export async function getPtvsdInjectCommand(host: string, port: number, pid: number): Promise<string> {
    // instead of requiring presence of correctly versioned ptvsd from pip (https://github.com/Microsoft/ptvsd)
    // use ptvsd shipped with vscode-python to avoid potential version mismatch

    const pyExtensionId: string = "ms-python.python";
    if (!vscode_utils.isPythonExtensionInstalled()) {
        const message = "Python debugging requires the Microsoft Python extension (ms-python.python).";
        vscode.window.showErrorMessage(message);
        throw new Error(message);
    }

    const pyExtension: vscode.Extension<IPythonExtensionApi> | undefined = vscode.extensions.getExtension(pyExtensionId);
    if (!pyExtension) {
        const message = "Python debugging requires the Microsoft Python extension (ms-python.python).";
        vscode.window.showErrorMessage(message);
        throw new Error(message);
    }

    if (!pyExtension.isActive) {
        await pyExtension.activate();
    }

    // tslint:disable-next-line:strict-boolean-expressions
    if (pyExtension.exports && pyExtension.exports.debug) {
        // hack python extension's api to get command for injecting ptvsd

        // pass false for waitForDebugger so the --wait flag won't be added
        const waitForDebugger: boolean = false;
        const ptvsdCommand = await pyExtension.exports.debug.getRemoteLauncherCommand(host, port, waitForDebugger);

        // prepend python interpreter
        ptvsdCommand.unshift("python");
        // append the --pid flag
        ptvsdCommand.push("--pid", pid.toString());
        return ptvsdCommand.join(" ");
    } else {
        throw new Error(`Update extension [${pyExtensionId}] to debug Python projects.`);
    }
}

/**
 * Parse environment variables from a .env file.
 * Supports basic KEY=VALUE format, ignoring comments and empty lines.
 * 
 * @param envFilePath Path to the .env file
 * @returns Object containing the parsed environment variables
 */
export function parseEnvFile(envFilePath: string): { [key: string]: string } {
    const envVars: { [key: string]: string } = {};
    
    try {
        const content = fs.readFileSync(envFilePath, 'utf8');
        const lines = content.split('\n');
        
        for (const line of lines) {
            // Trim whitespace
            const trimmedLine = line.trim();
            
            // Skip empty lines and comments
            if (!trimmedLine || trimmedLine.startsWith('#')) {
                continue;
            }
            
            // Find the first = character
            const separatorIndex = trimmedLine.indexOf('=');
            if (separatorIndex === -1) {
                continue; // Skip lines without =
            }
            
            const key = trimmedLine.substring(0, separatorIndex).trim();
            let value = trimmedLine.substring(separatorIndex + 1).trim();
            
            // Remove surrounding quotes if present
            if ((value.startsWith('"') && value.endsWith('"')) ||
                (value.startsWith("'") && value.endsWith("'"))) {
                value = value.substring(1, value.length - 1);
            }
            
            if (key) {
                envVars[key] = value;
            }
        }
    } catch (error) {
        const errorMessage = error instanceof Error ? error.message : String(error);
        // Only log if extension.outputChannel exists (may not exist in tests)
        if (extension.outputChannel) {
            extension.outputChannel.appendLine(`Warning: Failed to read env file ${envFilePath}: ${errorMessage}`);
        }
    }
    
    return envVars;
}

/**
 * Merge environment variables from an envFile with existing env object.
 * The env object takes precedence over envFile values.
 * 
 * @param env Existing environment variables object
 * @param envFile Path to the .env file (can be relative to workspace)
 * @param workspaceFolder Optional workspace folder for resolving relative paths
 * @returns Merged environment variables
 */
export function mergeEnvFile(
    env: { [key: string]: string } | undefined,
    envFile: string | undefined,
    workspaceFolder?: vscode.WorkspaceFolder
): { [key: string]: string } {
    const mergedEnv: { [key: string]: string } = {};
    
    // First, add variables from envFile if provided
    if (envFile) {
        let resolvedEnvFilePath = envFile;
        
        // If the path is not absolute, resolve it relative to workspace folder
        if (!path.isAbsolute(envFile)) {
            if (workspaceFolder) {
                resolvedEnvFilePath = path.join(workspaceFolder.uri.fsPath, envFile);
            } else {
                extension.outputChannel.appendLine(`Warning: envFile "${envFile}" is a relative path but no workspace folder is available. The path may not resolve correctly.`);
            }
        }
        
        const envFileVars = parseEnvFile(resolvedEnvFilePath);
        Object.assign(mergedEnv, envFileVars);
    }
    
    // Then, merge in the env object (which takes precedence)
    if (env) {
        Object.assign(mergedEnv, env);
    }
    
    return mergedEnv;
}
