// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as path from "path";
import * as child_process from "child_process";
import * as extension from "../extension";
import * as common from "./common";
import * as rosShell from "./ros-shell";
import * as colconUtils from "./colcon-utils";
import { env } from "process";

async function makeColcon(name: string, command: string, verb: string, args: string[], category?: string): Promise<vscode.Task> {
    let installType = '--symlink-install';
    if (process.platform === "win32") {

        // Use Merge Install on Windows to support adminless builds and deployment.
        installType = '--merge-install';
    }

    const baseArgs = [verb, installType, '--event-handlers', 'console_cohesion+', '--base-paths', vscode.workspace.rootPath, `--cmake-args`, ...args];
    
    // Add --packages-select to filter out ignored packages
    const workspaceRoot = vscode.workspace.rootPath;
    if (workspaceRoot) {
        const nonIgnoredPackages = await colconUtils.getNonIgnoredPackages(workspaceRoot);
        if (nonIgnoredPackages.length > 0) {
            // Insert --packages-select before --cmake-args
            const cmakeArgsIndex = baseArgs.indexOf('--cmake-args');
            if (cmakeArgsIndex !== -1) {
                baseArgs.splice(cmakeArgsIndex, 0, '--packages-select', ...nonIgnoredPackages);
            }
        }
    }

    const task = rosShell.make(name, {type: command, command, args: baseArgs}, category);
    task.problemMatchers = ["$colcon-gcc"];

    return task;
}

/**
 * Provides colcon build and test tasks.
 */
export class ColconProvider implements vscode.TaskProvider {
    public async provideTasks(token?: vscode.CancellationToken): Promise<vscode.Task[]> {
        const make = await makeColcon('Colcon Build Release', 'colcon', 'build', [`-DCMAKE_BUILD_TYPE=RelWithDebInfo`], 'build');
        make.group = vscode.TaskGroup.Build;

        const makeDebug = await makeColcon('Colcon Build Debug', 'colcon', 'build', [`-DCMAKE_BUILD_TYPE=Debug`], 'build');
        makeDebug.group = vscode.TaskGroup.Build;
        
        const test = await makeColcon('Colcon Build Test Release', 'colcon', 'test', [`-DCMAKE_BUILD_TYPE=RelWithDebInfo`], 'test');
        test.group = vscode.TaskGroup.Test;

        const testDebug = await makeColcon('Colcon Build Test Debug', 'colcon', 'test', [`-DCMAKE_BUILD_TYPE=Debug`], 'test');
        testDebug.group = vscode.TaskGroup.Test;

        return [make, makeDebug, test, testDebug];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return rosShell.resolve(task);
    }
}

export async function isApplicable(dir: string): Promise<boolean> {
    let colconCommand: string;
    if (process.platform === "win32") {
        colconCommand = `colcon --log-base nul list --base-paths \"${dir}\"`;
    } else {
        colconCommand = `colcon --log-base /dev/null list --base-paths ${dir}`;
    }

    const { stdout, stderr } = await child_process.exec(colconCommand, { env: extension.env });

    // Does this workspace have packages?
    for await (const line of stdout) {
        // Yes.
        return true;
    }

    // no.
    return false;
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
        '--packages-select',
        packageName,
        '--cmake-args',
        `-DCMAKE_BUILD_TYPE=${buildType}`
    ];

    const task = rosShell.make(`Colcon Build ${packageName}`, {type: 'colcon', command: 'colcon', args}, 'build');
    task.problemMatchers = ["$colcon-gcc"];
    task.group = vscode.TaskGroup.Build;

    return task;
}

