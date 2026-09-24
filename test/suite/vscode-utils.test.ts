import * as assert from "assert";
import * as vscode from "vscode";

import { compareVersions, setRosSetupScript } from "../../src/vscode-utils";
import { RosDistributionItem } from "../../src/ros/ros-distributions-provider";

describe("ROS setup selection settings scope", () => {
    const script = "/test/ros/setup.bash";
    let restore: (() => void)[];
    let updates: unknown[][];
    let commands: string[];
    let prompts: number;
    let choice: string | undefined;

    function replaceProperty(object: object, key: string, value: unknown): void {
        const descriptor = Object.getOwnPropertyDescriptor(object, key)!;
        Object.defineProperty(object, key, { configurable: true, value });
        restore.push(() => Object.defineProperty(object, key, descriptor));
    }

    beforeEach(() => {
        restore = [];
        updates = [];
        commands = [];
        prompts = 0;
        choice = undefined;
        replaceProperty(vscode.workspace, "workspaceFolders", undefined);
        replaceProperty(vscode.workspace, "workspaceFile", undefined);
        replaceProperty(vscode.workspace, "getConfiguration", () => ({
            update: async (...args: unknown[]) => { updates.push(args); },
        }));
        replaceProperty(vscode.window, "showInformationMessage", async () => {
            prompts++;
            return choice;
        });
        replaceProperty(vscode.commands, "executeCommand", async (command: string) => { commands.push(command); });
    });

    afterEach(() => {
        restore.reverse().forEach(restoreProperty => restoreProperty());
    });

    it("writes a global default only after explicit consent in an empty window", async () => {
        choice = "Set Global Default";
        assert.strictEqual(await setRosSetupScript(script), true);
        assert.deepStrictEqual(updates, [["rosSetupScript", script, vscode.ConfigurationTarget.Global]]);
        assert.strictEqual(prompts, 1);
    });

    it("does not write settings when the prompt is dismissed", async () => {
        assert.strictEqual(await setRosSetupScript(script), false);
        assert.deepStrictEqual(updates, []);
        assert.deepStrictEqual(commands, []);
    });

    for (const [label, command] of [
        ["Open Folder", "workbench.action.files.openFolder"],
        ["Open Workspace", "workbench.action.openWorkspace"],
    ]) {
        it(`offers ${label} without changing settings`, async () => {
            choice = label;
            assert.strictEqual(await setRosSetupScript(script), false);
            assert.deepStrictEqual(updates, []);
            assert.deepStrictEqual(commands, [command]);
        });
    }

    it("keeps settings workspace-scoped when a folder is open", async () => {
        replaceProperty(vscode.workspace, "workspaceFolders", [{ uri: vscode.Uri.file("/test/workspace") }]);
        assert.strictEqual(await setRosSetupScript(script), true);
        assert.deepStrictEqual(updates, [["rosSetupScript", script, vscode.ConfigurationTarget.Workspace]]);
        assert.strictEqual(prompts, 0);
    });

    it("supports an empty saved multi-root workspace without prompting", async () => {
        replaceProperty(vscode.workspace, "workspaceFolders", []);
        replaceProperty(vscode.workspace, "workspaceFile", vscode.Uri.file("/test/empty.code-workspace"));
        assert.strictEqual(await setRosSetupScript(script), true);
        assert.deepStrictEqual(updates, [["rosSetupScript", script, vscode.ConfigurationTarget.Workspace]]);
        assert.strictEqual(prompts, 0);
    });

    for (const [label, distro] of [["rolling (pixi)", "rolling"], ["jazzy", "jazzy"]]) {
        it(`saves the canonical ${distro} name with its setup script`, async () => {
            replaceProperty(vscode.workspace, "workspaceFolders", [{ uri: vscode.Uri.file("/test/workspace") }]);
            const item = new RosDistributionItem(label, script, false);
            assert.deepStrictEqual(item.command.arguments, [script, distro]);
            assert.strictEqual(await setRosSetupScript(item.command.arguments[0], item.command.arguments[1]), true);
            assert.deepStrictEqual(updates, [
                ["rosSetupScript", script, vscode.ConfigurationTarget.Workspace],
                ["distro", distro, vscode.ConfigurationTarget.Workspace],
            ]);
        });
    }

    it("updates both global defaults after consent without a workspace", async () => {
        choice = "Set Global Default";
        assert.strictEqual(await setRosSetupScript(script, "rolling"), true);
        assert.deepStrictEqual(updates, [
            ["rosSetupScript", script, vscode.ConfigurationTarget.Global],
            ["distro", "rolling", vscode.ConfigurationTarget.Global],
        ]);
    });

    for (const selection of [undefined, "Open Folder", "Open Workspace"]) {
        it(`leaves both settings unchanged for ${selection ?? "cancellation"}`, async () => {
            choice = selection;
            assert.strictEqual(await setRosSetupScript(script, "rolling"), false);
            assert.deepStrictEqual(updates, []);
        });
    }

    it("does not treat the legacy Windows layout name as a ROS distro", () => {
        const item = new RosDistributionItem("ros2-windows (pixi)", script, false);
        assert.deepStrictEqual(item.command.arguments, [script, undefined]);
    });
});

describe("VS Code Utils - Version Comparison", () => {
    describe("compareVersions", () => {
        it("should detect patch-level upgrades by default", () => {
            assert.strictEqual(compareVersions("1.2.3", "1.2.4"), -1);
            assert.strictEqual(compareVersions("1.2.4", "1.2.3"), 1);
        });

        it("should ignore patch differences when ignorePatch is true", () => {
            assert.strictEqual(compareVersions("1.2.3", "1.2.4", true), 0);
            assert.strictEqual(compareVersions("1.2.9", "1.2.0", true), 0);
        });

        it("should still detect major/minor upgrades when ignorePatch is true", () => {
            assert.strictEqual(compareVersions("1.2.3", "1.3.0", true), -1);
            assert.strictEqual(compareVersions("1.2.3", "2.0.0", true), -1);
            assert.strictEqual(compareVersions("2.1.0", "1.9.9", true), 1);
        });

        it("should treat missing components as zero", () => {
            assert.strictEqual(compareVersions("1", "1.0.0"), 0);
            assert.strictEqual(compareVersions("1.2", "1.2.0"), 0);
            assert.strictEqual(compareVersions("1.2", "1.3"), -1);
        });
    });
});
