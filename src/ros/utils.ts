// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as path from "path";
import { promises as fsPromises } from "fs";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import * as vscode_utils from "../vscode-utils";

// Re-export common shell utilities
export { 
    detectUserShell, 
    getSetupScriptExtension, 
    ShellInfo 
} from "@ranchhandrobotics/rde-common";

import {
    detectUserShell,
    sourceSetupFile as commonSourceSetupFile,
    SourceSetupOptions
} from "@ranchhandrobotics/rde-common";

/**
 * Executes a setup file and returns the resulting env.
 * This wraps the common sourceSetupFile with ROS-specific logging.
 */
export function sourceSetupFile(filename: string, env?: any): Promise<any> {
    const config = vscode_utils.getExtensionConfiguration();
    const pixiRoot = config.get("pixiRoot", "c:\\pixi_ws");
    
    const options: SourceSetupOptions = {
        cwd: vscode.workspace.rootPath,
        pixiRoot,
        onOutput: (message: string) => {
            extension.outputChannel.appendLine(message);
        }
    };
    
    return commonSourceSetupFile(filename, env, options);
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
        return fsPromises.readdir("/opt/ros");
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
