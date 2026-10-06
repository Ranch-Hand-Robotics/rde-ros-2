// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as rosShell from "./ros-shell";
import * as colconUtils from "./colcon-utils";

export const COLCON_TASK_TYPE = "colcon";
const COLCON_COMMAND = "colcon";

function hasTaskType(task: vscode.Task, expectedType: string): boolean {
    const definition = task.definition as vscode.TaskDefinition | undefined;
    return definition?.type === expectedType;
}

async function makeColcon(name: string, command: string, verb: string, args: string[], category?: string): Promise<vscode.Task> {
    let installType = '--symlink-install';
    if (process.platform === "win32") {

        // Use Merge Install on Windows to support adminless builds and deployment.
        installType = '--merge-install';
    }

    const baseArgs = [verb, installType, '--event-handlers', 'console_cohesion+', '--base-paths', vscode.workspace.rootPath, `--cmake-args`, ...args];
    
    // Task discovery must work before ROS/colcon is installed or activated.
    const ignored = Object.entries(colconUtils.getColconIgnoreConfig())
        .filter(([, ignored]) => ignored).map(([name]) => name);
    if (ignored.length) {
        baseArgs.splice(baseArgs.indexOf('--cmake-args'), 0, '--packages-skip', ...ignored);
    }

    // The task type must match the provider id ('colcon') so VS Code can map tasks to this provider.
    const task = rosShell.make(name, {type: COLCON_TASK_TYPE, command: command, args: baseArgs}, category);
    task.problemMatchers = ["$colcon-gcc"];

    return task;
}

/**
 * Provides colcon build and test tasks.
 */
export class ColconProvider implements vscode.TaskProvider {
    public async provideTasks(token?: vscode.CancellationToken): Promise<vscode.Task[]> {
        if (!vscode.workspace.rootPath) {
            return [];
        }
        const make = await makeColcon('Colcon Build Release', 'colcon', 'build', [`-DCMAKE_BUILD_TYPE=RelWithDebInfo`], 'build');
        make.group = vscode.TaskGroup.Build;

        const makeDebug = await makeColcon('Colcon Build Debug', 'colcon', 'build', [`-DCMAKE_BUILD_TYPE=Debug`], 'build');
        makeDebug.group = vscode.TaskGroup.Build;
        
        const test = await makeColcon('Colcon Build Test Release', 'colcon', 'test', [`-DCMAKE_BUILD_TYPE=RelWithDebInfo`], 'test');
        test.group = vscode.TaskGroup.Test;

        const testDebug = await makeColcon('Colcon Build Test Debug', 'colcon', 'test', [`-DCMAKE_BUILD_TYPE=Debug`], 'test');
        testDebug.group = vscode.TaskGroup.Test;

        const tasks = [make, makeDebug, test, testDebug];
        return tasks.filter(task => hasTaskType(task, COLCON_TASK_TYPE));
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        if (!hasTaskType(task, COLCON_TASK_TYPE)) {
            return undefined;
        }

        const resolvedTask = rosShell.resolve(task);
        if (!hasTaskType(resolvedTask, COLCON_TASK_TYPE)) {
            return undefined;
        }

        return resolvedTask;
    }
}

export async function isApplicable(dir: string): Promise<boolean> {
    return (await colconUtils.getPackages(dir)).length > 0;
}

/**
 * Creates a colcon build task for a specific package
 */
export async function makeColconPackageTask(packageName: string, buildType: string = 'RelWithDebInfo'): Promise<vscode.Task> {
    let installType = '--symlink-install';
    if (process.platform === "win32") {
        installType = '--merge-install';
    }

    const args = [
        'build',
        installType,
        '--event-handlers',
        'console_cohesion+',
        '--base-paths',
        vscode.workspace.rootPath,
        process.platform === "win32" ? '--packages-up-to' : '--packages-select',
        packageName,
        '--cmake-args',
        `-DCMAKE_BUILD_TYPE=${buildType}`
    ];

    const task = rosShell.make(`Colcon Build ${packageName}`, {type: COLCON_TASK_TYPE, command: COLCON_COMMAND, args}, 'build');
    task.problemMatchers = ["$colcon-gcc"];
    task.group = vscode.TaskGroup.Build;

    return task;
}

