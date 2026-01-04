// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as child_process from "child_process";
import * as util from "util";
import * as extension from "../extension";
import * as vscode_utils from "../vscode-utils";

const promisifiedExec = util.promisify(child_process.exec);

/**
 * Represents a ROS 2 package
 */
export interface Package {
    name: string;
    path: string;
}

/**
 * Gets the list of packages in the workspace using colcon list
 */
export async function getPackages(workspaceRoot: string): Promise<Package[]> {
    try {
        let colconCommand: string;
        if (process.platform === "win32") {
            colconCommand = `colcon --log-base nul list --base-paths "${workspaceRoot}"`;
        } else {
            colconCommand = `colcon --log-base /dev/null list --base-paths "${workspaceRoot}"`;
        }

        const { stdout } = await promisifiedExec(colconCommand, { env: extension.env });
        
        const packages: Package[] = [];
        const lines = stdout.trim().split('\n');
        
        for (const line of lines) {
            if (line.trim()) {
                // colcon list output format: package_name    path
                const parts = line.split(/\s+/);
                if (parts.length >= 2) {
                    packages.push({
                        name: parts[0],
                        path: parts[1]
                    });
                }
            }
        }
        
        return packages;
    } catch (error) {
        extension.outputChannel.appendLine(`Error getting packages: ${error.message}`);
        return [];
    }
}

/**
 * Finds the package name for a given folder path
 */
export async function findPackageForPath(folderPath: string, workspaceRoot: string): Promise<string | undefined> {
    const packages = await getPackages(workspaceRoot);
    
    // Find package that matches or contains this path
    for (const pkg of packages) {
        const packagePath = path.isAbsolute(pkg.path) ? pkg.path : path.join(workspaceRoot, pkg.path);
        
        // Check if the folder is the package root or inside it
        if (folderPath === packagePath || folderPath.startsWith(packagePath + path.sep)) {
            return pkg.name;
        }
    }
    
    return undefined;
}

/**
 * Gets the colconIgnore configuration
 */
export function getColconIgnoreConfig(): { [key: string]: boolean } {
    const config = vscode_utils.getExtensionConfiguration();
    return config.get<{ [key: string]: boolean }>("colconIgnore", {});
}

/**
 * Updates the colconIgnore configuration
 */
export async function updateColconIgnoreConfig(packageName: string, ignore: boolean): Promise<void> {
    const config = vscode.workspace.getConfiguration("ROS2");
    const currentIgnore = config.get<{ [key: string]: boolean }>("colconIgnore", {});
    
    if (ignore) {
        currentIgnore[packageName] = true;
    } else {
        delete currentIgnore[packageName];
    }
    
    await config.update("colconIgnore", currentIgnore, vscode.ConfigurationTarget.Workspace);
}

/**
 * Gets the list of non-ignored packages
 */
export async function getNonIgnoredPackages(workspaceRoot: string): Promise<string[]> {
    const packages = await getPackages(workspaceRoot);
    const ignoreConfig = getColconIgnoreConfig();
    
    return packages
        .filter(pkg => !ignoreConfig[pkg.name])
        .map(pkg => pkg.name);
}
