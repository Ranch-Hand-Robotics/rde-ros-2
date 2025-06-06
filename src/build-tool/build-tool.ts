// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as pfs from "../promise-fs";
import * as telemetry from "../telemetry-helper";
import * as colcon from "./colcon";

export abstract class BuildTool {
    public static current: BuildTool;
    public static registerTaskProvider(): vscode.Disposable[] {
        return this.current._registerTaskProvider();
    }

    public static async createPackage(context: vscode.ExtensionContext) {
        const reporter = telemetry.getReporter();
        return this.current._createPackage();
    }

    protected abstract _registerTaskProvider(): vscode.Disposable[];
    protected abstract _createPackage(): Promise<void>;
}

// tslint:disable-next-line: max-classes-per-file
class NotImplementedBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable[] {
        return null;
    }

    protected async _createPackage(): Promise<void> {
        return;
    }
}
// tslint:disable-next-line: max-classes-per-file
class ColconBuildTool extends BuildTool {
    public static async isApplicable(dir: string): Promise<boolean> {
        return colcon.isApplicable(dir);
    }

    protected _registerTaskProvider(): vscode.Disposable[] {
        return [vscode.tasks.registerTaskProvider("colcon", new colcon.ColconProvider())];
    }

    protected async _createPackage(): Promise<void> {
        // Do nothing.
        return;
    }
}

BuildTool.current = new NotImplementedBuildTool();

/**
 * Determines build system and workspace path in use by checking for unique
 * auto-generated files.
 */
export async function determineBuildTool(dir: string): Promise<boolean> {
    while (dir && path.dirname(dir) !== dir) {
        if (await ColconBuildTool.isApplicable(dir)) {
            BuildTool.current = new ColconBuildTool();
            return true;
        }

        dir = path.dirname(dir);
    }
    return false;
}

/**
 * Check if a task belongs to our extension.
 * @param task Task to check
 */
export function isROSBuildTask(task: vscode.Task) {
    const types = new Set(["colcon"]);
    const isRosTask = types.has(task.definition.type);
    const isBuildTask = vscode.TaskGroup.Build === task.group;
    return isRosTask && isBuildTask;
}
