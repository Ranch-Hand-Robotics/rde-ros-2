// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import { windowsColconExecution } from "./windows-colcon-task";

export interface RosTaskDefinition extends vscode.TaskDefinition {
    name?: string;
    type: string;
    command: string;
    args?: string[];
    options?: { cwd?: string; env?: { [key: string]: string | null } };
    buildOptions?: { cwd?: string; env?: { [key: string]: string | null } };
}

export class RosShellTaskProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        return this.defaultRosTasks();
    }

    public defaultRosTasks(): vscode.Task[] {
        const rosCore = make('ros core', {type: 'ROS2', command: 'roscore'}, 'roscore');
        rosCore.isBackground = true;
        rosCore.problemMatchers = ['$roscore'];

        const rosLaunch = make('ros launch', {type: 'ROS2', command: 'roslaunch', args: ['package_name', 'launch_file.launch']}, 'roslaunch');
        rosLaunch.isBackground = true;
        rosLaunch.problemMatchers = ['$roslaunch'];

        return [rosCore, rosLaunch];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return resolve(task);
    }
}

export function registerRosShellTaskProvider(): vscode.Disposable[] {
    return [
        vscode.tasks.registerTaskProvider('ROS2', new RosShellTaskProvider()),
    ];
}

export function resolve(task: vscode.Task): vscode.Task {
    let definition = task.definition as RosTaskDefinition
    definition.command = definition.command || definition.type;
    // Ensure type is preserved when resolving
    definition.type = definition.type || 'ROS2';
    // VS Code requires the original definition when resolving tasks.json entries.
    // Preserve scope, presentation, group, and other user customizations as well.
    task.execution = make(definition.command, definition, undefined, task.scope).execution;
    return task;
}

export function make(name: string, definition: RosTaskDefinition, category?: string,
    scope: vscode.TaskScope | vscode.WorkspaceFolder = vscode.TaskScope.Workspace): vscode.Task {
    definition.command = definition.command || definition.type; // Command can be missing in build tasks that have type==command

    const args = definition.args || [];
    const windowsBuild = process.platform === "win32" && definition.type === "colcon" && args.includes("build");
    if (windowsBuild) {
        // 'options' is reserved by VS Code and removed from resolvedDefinition.
        // Carry our execution options in a contributed property that survives it.
        definition.buildOptions = { cwd: "${workspaceFolder}", ...definition.options, ...definition.buildOptions };
    }
    const task = new vscode.Task(definition, scope, name, definition.command);

    task.execution = windowsBuild ? windowsColconExecution(
        () => process.env,
        message => extension.outputChannel?.appendLine(message),
        typeof scope === "object" ? scope.uri.fsPath : vscode.workspace.rootPath,
        (activated, options) => extension.prepareRosBuildEnvironment(activated, options),
    ) : new vscode.ShellExecution(definition.command, args, {
        env: extension.env,
    });
    return task;
}
