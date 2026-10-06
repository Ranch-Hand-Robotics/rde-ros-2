// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import { windowsColconExecution } from "./windows-colcon-task";
import { deferredRosExecution, RosTaskOptions } from "./deferred-ros-task";

export interface RosTaskDefinition extends vscode.TaskDefinition {
    name?: string;
    type: string;
    command: string;
    args?: string[];
    options?: RosTaskOptions;
    taskOptions?: RosTaskOptions;
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

/** Recover reserved tasks.json options before carrying them through variable
 * substitution as taskOptions. Never borrow a same-named task from another scope.
 */
function configuredTaskOptions(task: vscode.Task): RosTaskOptions | undefined {
        if (!task.name) { return undefined; }
        const folder = typeof task.scope === "object" ? task.scope : undefined;
        const config = vscode.workspace.getConfiguration("tasks", folder?.uri);
        const inspected = config.inspect<RosTaskDefinition[]>("tasks");
    // A single-folder tasks.json appears at workspace level. In a saved/multi-root
    // workspace, folder tasks and workspace tasks are distinct scopes.
    const folderTasks = inspected?.workspaceFolderValue
        ?? (!vscode.workspace.workspaceFile ? inspected?.workspaceValue : undefined);
        const tasks = task.scope === vscode.TaskScope.Global ? inspected?.globalValue
        : folder ? folderTasks : inspected?.workspaceValue;
        const matches = tasks?.filter(candidate => candidate.label === task.name && candidate.type === task.definition.type);
        if (matches?.length !== 1) { return undefined; }
        const candidate = matches[0];
        const platform = process.platform === "darwin" ? "osx" : "linux";
        const globalOptions = config.get<RosTaskOptions>("options");
        const platformOptions = candidate[platform]?.options as RosTaskOptions | undefined;
        return {
                ...globalOptions, ...candidate.options, ...platformOptions,
                env: { ...globalOptions?.env, ...candidate.options?.env, ...platformOptions?.env },
        };
}

export function resolve(task: vscode.Task): vscode.Task {
    let definition = task.definition as RosTaskDefinition
    definition.command = definition.command || definition.type;
    // Ensure type is preserved when resolving
    definition.type = definition.type || 'ROS2';
    if ((process.platform === "linux" || process.platform === "darwin") && !definition.options) {
        const options = configuredTaskOptions(task);
        if (options) {
            definition.taskOptions = {
                ...options, ...definition.taskOptions,
                env: { ...options.env, ...definition.taskOptions?.env },
            };
        }
    }
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
    const deferred = process.platform === "linux" || process.platform === "darwin";
    if (deferred) {
        // taskOptions must be contributed for both ROS2 and colcon, so VS Code
        // preserves it and resolves variables before CustomExecution is called.
        definition.taskOptions = {
            ...(typeof scope === "object" || vscode.workspace.rootPath ? { cwd: "${workspaceFolder}" } : {}),
            ...definition.options, ...definition.taskOptions,
            ...(definition.options?.env || definition.taskOptions?.env ? {
                env: { ...definition.options?.env, ...definition.taskOptions?.env },
            } : {}),
        };
    }
    const task = new vscode.Task(definition, scope, name, definition.command);

    task.execution = windowsBuild ? windowsColconExecution(
        () => process.env,
        message => extension.outputChannel?.appendLine(message),
        typeof scope === "object" ? scope.uri.fsPath : vscode.workspace.rootPath,
        (activated, options) => extension.prepareRosBuildEnvironment(activated, options),
    ) : deferred ? deferredRosExecution(
        () => extension.resolvedEnv(),
        typeof scope === "object" ? scope.uri.fsPath : vscode.workspace.rootPath,
    ) : new vscode.ShellExecution(definition.command, args, {
        env: extension.env,
    });
    return task;
}
