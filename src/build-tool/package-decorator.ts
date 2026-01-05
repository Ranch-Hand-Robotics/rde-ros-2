// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as colconUtils from "./colcon-utils";

/**
 * Provides file decorations for ROS 2 packages in the Explorer
 * Shows whether a package is enabled or disabled in colcon builds
 */
export class PackageDecorationProvider implements vscode.FileDecorationProvider {
    private readonly onDidChangeFileDecorationEmitter = new vscode.EventEmitter<vscode.Uri | undefined>();
    public onDidChangeFileDecorations = this.onDidChangeFileDecorationEmitter.event;

    /**
     * Notifies the provider to refresh decorations for a specific URI
     */
    public async refresh(uri: vscode.Uri): Promise<void> {
        // Refresh the target folder and its immediate children so decorations re-evaluate
        this.onDidChangeFileDecorationEmitter.fire(uri);

        try {
            const entries = await vscode.workspace.fs.readDirectory(uri);
            for (const [name] of entries) {
                this.onDidChangeFileDecorationEmitter.fire(vscode.Uri.joinPath(uri, name));
            }
        } catch {
            // Ignore errors (e.g., folder no longer exists)
        }
    }

    /**
     * Clears the cache and refreshes all decorations
     */
    public refreshAll(): void {
        // Fire with undefined to force VS Code to refresh all decorations
        this.onDidChangeFileDecorationEmitter.fire(undefined);
    }

    /**
     * Provides a file decoration for a given URI
     */
    public async provideFileDecoration(uri: vscode.Uri): Promise<vscode.FileDecoration | undefined> {
        // Only decorate folders
        try {
            const stat = await vscode.workspace.fs.stat(uri);
            if (stat.type !== vscode.FileType.Directory) {
                return undefined;
            }
        } catch {
            return undefined;
        }

        const workspaceRoot = vscode.workspace.rootPath;
        if (!workspaceRoot) {
            return undefined;
        }

        try {
            // Check if this folder is a ROS 2 package
            const packageName = await colconUtils.findPackageForPath(uri.fsPath, workspaceRoot);
            if (!packageName) {
                return undefined;
            }

            // Get the ignore state
            const ignoreConfig = colconUtils.getColconIgnoreConfig();
            const isIgnored = ignoreConfig[packageName] === true;

            // Return decoration based on state
            if (isIgnored) {
                return new vscode.FileDecoration(
                    '✕',  // Badge text
                    'Package is disabled in colcon builds',  // Tooltip
                    new vscode.ThemeColor('debugIcon.breakpointDisabledForeground')  // Gray color for disabled
                );
            } else {
                // For enabled packages, don't set a color - use the default folder color
                return new vscode.FileDecoration(
                    '✓',  // Badge text
                    'Package is enabled in colcon builds'  // Tooltip
                );
            }
        } catch (error) {
            // Silently ignore errors - decoration is optional
            return undefined;
        }
    }
}

/**
 * Singleton instance of the decoration provider
 */
let decorationProvider: PackageDecorationProvider | undefined;

/**
 * Gets the singleton instance of the decoration provider
 */
function getDecorationProvider(): PackageDecorationProvider {
    if (!decorationProvider) {
        decorationProvider = new PackageDecorationProvider();
    }
    return decorationProvider;
}

/**
 * Registers the package decoration provider
 */
export function registerPackageDecorationProvider(): vscode.Disposable {
    const provider = getDecorationProvider();
    return vscode.window.registerFileDecorationProvider(provider);
}

/**
 * Exports the getter for use in commands
 */
export function refreshPackageDecoration(uri: vscode.Uri): Promise<void> {
    const provider = getDecorationProvider();
    return provider.refresh(uri);
}

export function refreshAllPackageDecorations(): void {
    const provider = getDecorationProvider();
    provider.refreshAll();
}
