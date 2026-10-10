// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import { promises as fsPromises } from "fs";
import * as os from "os";
import { getPixiInstallRoot } from "./installer/pixi-location";

export async function showRosInstallationOptions(): Promise<void> {
    const selected = await vscode.window.showQuickPick([
        { label: "$(cloud-download) Install ROS 2", command: "ROS2.installRos" },
        { label: "$(search) Find Existing ROS 2 Installation", command: "ROS2.findRos" },
        { label: "$(remote) Create ROS 2 Devcontainer for This Workspace", command: "ROS2.installRosContainer" },
    ], { title: "Add ROS 2 Installation", placeHolder: "Choose how to add ROS 2" });
    if (selected) {
        await vscode.commands.executeCommand(selected.command);
    }
}

export async function createRosDevContainer(): Promise<void> {
    if (!vscode.workspace.workspaceFolders?.length) {
        await vscode.window.showInformationMessage("Open a workspace folder before creating a ROS 2 devcontainer.");
        return;
    }
    if (!vscode.extensions.getExtension("ms-vscode-remote.remote-containers")) {
        const choice = await vscode.window.showInformationMessage(
            "The Dev Containers extension is required to create a ROS 2 devcontainer.", "Open Dev Containers"
        );
        if (choice === "Open Dev Containers") {
            await vscode.commands.executeCommand("workbench.extensions.action.showExtensionsWithIds", ["ms-vscode-remote.remote-containers"]);
        }
        return;
    }
    await vscode.commands.executeCommand("remote-containers.createDevContainerFile");
}

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

        this.description = isActive ? "active" : undefined;
        this.tooltip = `ROS 2 distribution: ${distroName}\nSetup script: ${setupScript}`;
        this.iconPath = new vscode.ThemeIcon(isActive ? "check" : "package");
        this.contextValue = "rosDistribution";

        this.command = {
            command: "ROS2.setActiveDistro",
            title: "Set as Active Distribution",
            arguments: [setupScript, distroName === "ros2-windows (pixi)" ? undefined : distroName.replace(/ \(pixi\)$/, "")],
        };
    }
}

/**
 * Detects installed ROS distributions by scanning standard install paths
 * on the current platform.
 */
export async function detectInstalledDistros(): Promise<{ name: string; setupScript: string }[]> {
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
        const configuredPixiRoot = getPixiInstallRoot();
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
                    const directory = path.join(winRosBase, entry.name);
                    for (const script of [
                        path.join(directory, "x64", "local_setup.bat"),
                        path.join(directory, "x64", "setup.bat"),
                        path.join(directory, "local_setup.bat"),
                        path.join(directory, "setup.bat"),
                    ]) {
                        if (await pushIfExists(entry.name, script)) {
                            break;
                        }
                    }
                }
            }
        } catch {
            // C:\opt\ros doesn't exist
        }
    } else {
        if (os.platform() === "darwin") {
            const configuredRoot = getPixiInstallRoot();
            const roots = new Set([configuredRoot, path.join(os.homedir(), "pixi_ws")].filter(Boolean));
            for (const root of roots) {
                try {
                    const entries = await fsPromises.readdir(root!, { withFileTypes: true });
                    for (const entry of entries) {
                        if (entry.isDirectory()) {
                            await pushIfExists(`${entry.name} (pixi)`, path.join(root!, entry.name, "setup.bash"));
                        }
                    }
                } catch {
                    continue;
                }
            }
        }
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

/** Select an actual setup script, preserving discovery's cached-Pixi-first order. */
export function selectInstalledDistro(
    distros: { name: string; setupScript: string }[],
    configuredDistro: string = "",
    environmentDistro: string = ""
): { name: string; setupScript: string } | undefined {
    const distroName = (name: string) => name.replace(/ \(pixi\)$/, "");
    const requestedDistro = configuredDistro || environmentDistro;
    if (requestedDistro) {
        return distros.find(distro => distroName(distro.name) === distroName(requestedDistro));
    }

    // Multiple installations of the same distro are not an ambiguous selection.
    const names = new Set(distros.map(distro => distroName(distro.name)));
    return names.size === 1 ? distros[0] : undefined;
}

/**
 * TreeDataProvider for installed ROS distributions shown in the sidebar.
 * When no distributions are found, the view shows welcome content with
 * Install ROS and Find ROS buttons defined in package.json viewsWelcome.
 */
export class RosDistributionsProvider implements vscode.TreeDataProvider<RosDistributionItem>, vscode.Disposable {
    private _onDidChangeTreeData = new vscode.EventEmitter<RosDistributionItem | undefined | void>();
    readonly onDidChangeTreeData = this._onDidChangeTreeData.event;
    private _onDidChangeLoading = new vscode.EventEmitter<boolean>();
    readonly onDidChangeLoading = this._onDidChangeLoading.event;
    private activeSearches = 0;

    private cachedItems: RosDistributionItem[] | undefined;
    private removing = false;

    constructor() {}

    private async removableDirectory(setupScript: string): Promise<string | undefined> {
        if (!path.isAbsolute(setupScript)) {
            return undefined;
        }
        const configuredRoot = getPixiInstallRoot();
        const defaultRoot = os.platform() === "win32" ? "c:\\pixi_ws" : path.join(os.homedir(), "pixi_ws");
        for (const root of new Set([configuredRoot, defaultRoot].filter(Boolean))) {
            if (!path.isAbsolute(root!)) {
                continue;
            }
            const relative = path.relative(root!, setupScript);
            const parts = relative.split(path.sep);
            if (path.isAbsolute(relative) || parts.length < 2 || parts[0] === ".." || !parts[0]) {
                continue;
            }
            const directory = path.join(root!, parts[0]);
            const allowedScripts = [
                "setup.bash", "local_setup.bat", path.join("install", "setup.bat"),
                path.join(".pixi", "envs", parts[0], "Library", "local_setup.bat"),
                path.join(".pixi", "envs", parts[0], "Library", "setup.bat"),
            ];
            if (!allowedScripts.includes(path.relative(directory, setupScript))) {
                continue;
            }
            try {
                const realRoot = await fsPromises.realpath(root!);
                const realDirectory = await fsPromises.realpath(directory);
                if (realDirectory !== path.join(realRoot, parts[0]) || !(await fsPromises.lstat(directory)).isDirectory()) {
                    continue;
                }
                const protectedPaths = [os.homedir(), ...(vscode.workspace.workspaceFolders ?? []).map(folder => folder.uri.fsPath)];
                let protectedDirectory = false;
                for (const protectedPath of protectedPaths) {
                    const realProtectedPath = await fsPromises.realpath(protectedPath);
                    const relativeProtectedPath = path.relative(realDirectory, realProtectedPath);
                    if (!relativeProtectedPath || (!path.isAbsolute(relativeProtectedPath) && relativeProtectedPath.split(path.sep)[0] !== "..")) {
                        protectedDirectory = true;
                        break;
                    }
                }
                if (protectedDirectory || !(await fsPromises.lstat(path.join(directory, "pixi.toml"))).isFile()
                    || !(await fsPromises.lstat(path.join(directory, ".pixi"))).isDirectory()) {
                    continue;
                }
                const realScript = await fsPromises.realpath(setupScript);
                const relativeScript = path.relative(realDirectory, realScript);
                if (path.isAbsolute(relativeScript) || relativeScript.split(path.sep)[0] === ".."
                    || !(await fsPromises.stat(realScript)).isFile()) {
                    continue;
                }
                return directory;
            } catch {
                continue;
            }
        }
        return undefined;
    }

    async remove(item?: RosDistributionItem): Promise<void> {
        if (!(item instanceof RosDistributionItem) || this.removing) {
            return;
        }
        this.removing = true;
        let offerReload = false;
        try {
            const directory = await this.removableDirectory(item.setupScript);
            if (!directory) {
                void vscode.window.showWarningMessage("This installation cannot be safely removed automatically. Use its package manager or original uninstall procedure, then refresh Distributions.");
                return;
            }
            const config = vscode.workspace.getConfiguration("ROS2");
            const wasActive = config.get<string>("rosSetupScript") === item.setupScript;
            const confirmed = await vscode.window.showWarningMessage(
                `Remove ROS 2 ${item.distroName}?`,
                { modal: true, detail: `Move this entire Pixi installation to the Trash:\n${directory}\n\nStop any running ROS nodes, debugging sessions, and terminals using it first. Other workspaces using it will need another distribution.${wasActive ? "\n\nThis is your active distribution. Reload the window after removal." : ""}` },
                "Move to Trash"
            );
            if (confirmed !== "Move to Trash") {
                return;
            }
            if (await this.removableDirectory(item.setupScript) !== directory) {
                throw new Error("The installation changed while confirming removal. Refresh Distributions and try again.");
            }
            await vscode.workspace.fs.delete(vscode.Uri.file(directory), { recursive: true, useTrash: true });
            try {
                const scopes: [vscode.WorkspaceConfiguration, vscode.ConfigurationTarget, "globalValue" | "workspaceValue" | "workspaceFolderValue"][] = [
                    [config, vscode.ConfigurationTarget.Global, "globalValue"],
                    [config, vscode.ConfigurationTarget.Workspace, "workspaceValue"],
                    ...(vscode.workspace.workspaceFolders ?? []).map(folder => [
                        vscode.workspace.getConfiguration("ROS2", folder.uri), vscode.ConfigurationTarget.WorkspaceFolder, "workspaceFolderValue",
                    ] as [vscode.WorkspaceConfiguration, vscode.ConfigurationTarget, "workspaceFolderValue"]),
                ];
                for (const [scope, target, value] of scopes) {
                    if (scope.inspect<string>("rosSetupScript")?.[value] === item.setupScript) {
                        await scope.update("distro", undefined, target);
                        await scope.update("rosSetupScript", undefined, target);
                    }
                }
            } finally {
                this.refresh();
            }
            offerReload = wasActive;
        } finally {
            this.removing = false;
        }
        if (offerReload) {
            const action = await vscode.window.showInformationMessage("Distribution removed. Reload the window to clear the old ROS environment.", "Reload Window");
            if (action === "Reload Window") {
                await vscode.commands.executeCommand("workbench.action.reloadWindow");
            }
        }
    }

    private async updateDistributionContext(hasDistributions: boolean, searchComplete: boolean): Promise<void> {
        await Promise.all([
            vscode.commands.executeCommand("setContext", "ros2.hasDistributions", hasDistributions),
            vscode.commands.executeCommand("setContext", "ros2.distributionSearchComplete", searchComplete),
        ]);
    }

    dispose(): void {
        this._onDidChangeTreeData.dispose();
        this._onDidChangeLoading.dispose();
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

        this.activeSearches++;
        this._onDidChangeLoading.fire(true);
        try {
            return await vscode.window.withProgress({ location: { viewId: "ros2Distributions" } }, async () => {
                const config = vscode.workspace.getConfiguration("ROS2");
                const activeScript: string = config.get("rosSetupScript") ?? "";
                const distros = await detectInstalledDistros();
                this.cachedItems = distros.map(
                    (d) => new RosDistributionItem(d.name, d.setupScript, d.setupScript === activeScript)
                );
                await this.updateDistributionContext(this.cachedItems.length > 0, true);
                return this.cachedItems;
            });
        } finally {
            this.activeSearches--;
            if (this.activeSearches === 0) {
                this._onDidChangeLoading.fire(false);
            }
        }
    }
}
