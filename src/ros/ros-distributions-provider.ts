// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import { promises as fsPromises } from "fs";
import * as os from "os";

/**
 * Represents a single installed ROS distribution in the tree.
 */
export class RosDistributionItem extends vscode.TreeItem {
    constructor(
        public readonly distroName: string,
        public readonly setupScript: string,
        public readonly isActive: boolean
    ) {
        super(distroName, vscode.TreeItemCollapsibleState.None);

        this.description = isActive ? "active" : setupScript;
        this.tooltip = `ROS 2 distribution: ${distroName}\nSetup script: ${setupScript}`;
        this.iconPath = new vscode.ThemeIcon(isActive ? "check" : "package");
        this.contextValue = "rosDistribution";

        this.command = {
            command: "ROS2.setActiveDistro",
            title: "Set as Active Distribution",
            arguments: [setupScript],
        };
    }
}

/**
 * Detects installed ROS distributions by scanning standard install paths
 * on the current platform.
 */
async function detectInstalledDistros(): Promise<{ name: string; setupScript: string }[]> {
    const results: { name: string; setupScript: string }[] = [];
    const seenScripts = new Set<string>();

    const pushIfExists = async (name: string, setupScript: string): Promise<boolean> => {
        try {
            await fsPromises.access(setupScript);
            const normalized = path.normalize(setupScript);
            if (!seenScripts.has(normalized)) {
                seenScripts.add(normalized);
                results.push({ name, setupScript });
            }
            return true;
        } catch {
            return false;
        }
    };

    if (os.platform() === "win32") {
        // Windows: quickly probe Pixi roots first, then check C:\opt\ros.
        const config = vscode.workspace.getConfiguration("ROS2");
        const configuredPixiRoot: string = config.get("pixiRoot") ?? "";
        const defaultPixiRoot = "c:\\pixi_ws";
        const pixiRoots = Array.from(new Set([configuredPixiRoot, defaultPixiRoot].filter(Boolean)));

        // Check known Pixi layouts:
        // 1) legacy: <pixiRoot>/ros2-windows/local_setup.bat
        // 2) per-distro: <pixiRoot>/<distro>/install/setup.bat
        // 3) per-distro fallback: <pixiRoot>/<distro>/local_setup.bat
        // 4) pixi env: <pixiRoot>/<distro>/.pixi/envs/<distro>/Library/local_setup.bat
        // 5) pixi env fallback: .../Library/setup.bat
        for (const pixiRoot of pixiRoots) {
            await pushIfExists("ros2-windows (pixi)", path.join(pixiRoot, "ros2-windows", "local_setup.bat"));

            try {
                const entries = await fsPromises.readdir(pixiRoot, { withFileTypes: true });
                for (const entry of entries) {
                    if (!entry.isDirectory()) {
                        continue;
                    }

                    const distroName = `${entry.name} (pixi)`;
                    const installSetup = path.join(pixiRoot, entry.name, "install", "setup.bat");
                    const localSetup = path.join(pixiRoot, entry.name, "local_setup.bat");
                    const foundInstall = await pushIfExists(distroName, installSetup);
                    if (!foundInstall) {
                        const foundLocal = await pushIfExists(distroName, localSetup);
                        if (!foundLocal) {
                            const pixiEnvLocalSetup = path.join(
                                pixiRoot,
                                entry.name,
                                ".pixi",
                                "envs",
                                entry.name,
                                "Library",
                                "local_setup.bat"
                            );
                            const foundEnvLocal = await pushIfExists(distroName, pixiEnvLocalSetup);
                            if (!foundEnvLocal) {
                                const pixiEnvSetup = path.join(
                                    pixiRoot,
                                    entry.name,
                                    ".pixi",
                                    "envs",
                                    entry.name,
                                    "Library",
                                    "setup.bat"
                                );
                                await pushIfExists(distroName, pixiEnvSetup);
                            }
                        }
                    }
                }
            } catch {
                // Pixi root doesn't exist or cannot be read.
            }
        }

        // Standard Windows ROS install at C:\opt\ros\<distro>
        const winRosBase = "C:\\opt\\ros";
        try {
            const entries = await fsPromises.readdir(winRosBase, { withFileTypes: true });
            for (const entry of entries) {
                if (entry.isDirectory()) {
                    const script = path.join(winRosBase, entry.name, "x64", "local_setup.bat");
                    const foundX64 = await pushIfExists(entry.name, script);
                    if (!foundX64) {
                        // try setup.bat in root
                        const scriptRoot = path.join(winRosBase, entry.name, "local_setup.bat");
                        await pushIfExists(entry.name, scriptRoot);
                    }
                }
            }
        } catch {
            // C:\opt\ros doesn't exist
        }
    } else {
        // Linux/macOS: standard /opt/ros/<distro>
        const rosBase = "/opt/ros";
        try {
            const entries = await fsPromises.readdir(rosBase, { withFileTypes: true });
            for (const entry of entries) {
                if (entry.isDirectory()) {
                    const bashScript = path.join(rosBase, entry.name, "setup.bash");
                    const shScript = path.join(rosBase, entry.name, "setup.sh");
                    // Prefer setup.bash over setup.sh
                    let script: string | undefined;
                    try {
                        await fsPromises.access(bashScript);
                        script = bashScript;
                    } catch {
                        try {
                            await fsPromises.access(shScript);
                            script = shScript;
                        } catch {
                            // neither present
                        }
                    }
                    if (script) {
                        results.push({ name: entry.name, setupScript: script });
                    }
                }
            }
        } catch {
            // /opt/ros doesn't exist
        }
    }

    return results;
}

/**
 * TreeDataProvider for installed ROS distributions shown in the sidebar.
 * When no distributions are found, the view shows welcome content with
 * Install ROS and Find ROS buttons defined in package.json viewsWelcome.
 */
export class RosDistributionsProvider implements vscode.TreeDataProvider<RosDistributionItem>, vscode.Disposable {
    private _onDidChangeTreeData = new vscode.EventEmitter<RosDistributionItem | undefined | void>();
    readonly onDidChangeTreeData = this._onDidChangeTreeData.event;

    private cachedItems: RosDistributionItem[] | undefined;

    constructor() {}

    private async updateDistributionContext(hasDistributions: boolean, searchComplete: boolean): Promise<void> {
        await Promise.all([
            vscode.commands.executeCommand("setContext", "ros2.hasDistributions", hasDistributions),
            vscode.commands.executeCommand("setContext", "ros2.distributionSearchComplete", searchComplete),
        ]);
    }

    dispose(): void {
        this._onDidChangeTreeData.dispose();
    }

    refresh(): void {
        this.cachedItems = undefined;
        void this.updateDistributionContext(false, false);
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: RosDistributionItem): vscode.TreeItem {
        return element;
    }

    async getChildren(element?: RosDistributionItem): Promise<RosDistributionItem[]> {
        if (element) {
            return [];
        }

        if (this.cachedItems) {
            return this.cachedItems;
        }

        const config = vscode.workspace.getConfiguration("ROS2");
        const activeScript: string = config.get("rosSetupScript") ?? "";

        const distros = await detectInstalledDistros();
        this.cachedItems = distros.map(
            (d) => new RosDistributionItem(d.name, d.setupScript, d.setupScript === activeScript)
        );
        await this.updateDistributionContext(this.cachedItems.length > 0, true);

        return this.cachedItems;
    }
}
