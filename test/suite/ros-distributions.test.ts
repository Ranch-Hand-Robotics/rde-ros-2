import * as assert from "assert";
import * as vscode from "vscode";
import * as path from "path";
import * as os from "os";
import { promises as fs } from "fs";
import { RosDistributionItem, RosDistributionsProvider } from "../../src/ros/ros-distributions-provider";

describe("Distribution view actions", () => {
  let sandbox: string;
  let root: string;
  let directory: string;
  let script: string;
  let provider: RosDistributionsProvider;
  let restore: (() => void)[];
  let deletions: [string, { recursive: boolean; useTrash: boolean }][];
  let updates: unknown[][];
  let choice: string | undefined;
  let activeScript: string | undefined;
  let globalScript: string | undefined;
  let workspaceScript: string | undefined;
  let folderScript: string | undefined;
  let warnings: unknown[][];
  let refreshes: number;

  function replaceProperty(object: object, key: string, value: unknown): void {
    const descriptor = Object.getOwnPropertyDescriptor(object, key)!;
    Object.defineProperty(object, key, { configurable: true, value });
    restore.push(() => Object.defineProperty(object, key, descriptor));
  }

  beforeEach(async () => {
    restore = [];
    deletions = [];
    updates = [];
    warnings = [];
    refreshes = 0;
    choice = undefined;
    activeScript = undefined;
    globalScript = undefined;
    workspaceScript = undefined;
    folderScript = undefined;
    sandbox = await fs.mkdtemp(path.join(os.tmpdir(), "ros-distributions-"));
    root = path.join(sandbox, "pixi_ws");
    directory = path.join(root, "jazzy");
    script = path.join(directory, "setup.bash");
    await fs.mkdir(path.join(directory, ".pixi"), { recursive: true });
    await fs.writeFile(path.join(directory, "pixi.toml"), "[workspace]\nname = 'jazzy'\n");
    await fs.writeFile(script, "");
    replaceProperty(vscode.workspace, "workspaceFolders", undefined);
    replaceProperty(vscode.workspace, "getConfiguration", (_section: string, resource?: vscode.Uri) => ({
      get: (key: string) => key === "pixiRoot" ? root : activeScript,
      inspect: () => ({ globalValue: globalScript, workspaceValue: workspaceScript, workspaceFolderValue: resource ? folderScript : undefined }),
      update: async (...args: unknown[]) => { updates.push(args); },
    }));
    replaceProperty(vscode.workspace, "fs", {
      ...vscode.workspace.fs,
      delete: async (uri: vscode.Uri, options: { recursive: boolean; useTrash: boolean }) => {
        deletions.push([uri.fsPath, options]);
      },
    });
    replaceProperty(vscode.window, "showWarningMessage", async (...args: unknown[]) => {
      warnings.push(args);
      return choice;
    });
    replaceProperty(vscode.window, "showInformationMessage", async () => undefined);
    replaceProperty(vscode.commands, "executeCommand", async () => undefined);
    provider = new RosDistributionsProvider();
    provider.onDidChangeTreeData(() => { refreshes++; });
  });

  afterEach(async () => {
    provider?.dispose();
    restore.reverse().forEach(restoreProperty => restoreProperty());
    await fs.rm(sandbox, { recursive: true, force: true });
  });

  const item = (setup: string) => new RosDistributionItem("jazzy (pixi)", setup, false);

  it("contributes install beside refresh and an inline trash action", async () => {
    const manifest = JSON.parse(await fs.readFile(path.resolve(__dirname, "../../../package.json"), "utf8"));
    const { commands, menus } = manifest.contributes;
    assert.strictEqual(commands.find(command => command.command === "ROS2.installRos").icon, "$(add)");
    assert.ok(menus["view/title"].some(action => action.command === "ROS2.installRos" && action.when === "view == ros2Distributions" && action.group === "navigation@1"));
    assert.ok(menus["view/title"].some(action => action.command === "ROS2.distributions.refresh" && action.group === "navigation@2"));
    assert.strictEqual(commands.find(command => command.command === "ROS2.distributions.remove").icon, "$(trash)");
    assert.ok(menus["view/item/context"].some(action => action.command === "ROS2.distributions.remove" && action.when.includes(`viewItem == ${item(script).contextValue}`) && action.group === "inline"));
  });

  it("does nothing when confirmation is dismissed", async () => {
    await provider.remove(item(script));
    assert.strictEqual((warnings[0][1] as vscode.MessageOptions).modal, true);
    assert.ok((warnings[0][1] as vscode.MessageOptions).detail.includes(directory));
    assert.deepStrictEqual(deletions, []);
    assert.deepStrictEqual(updates, []);
    assert.strictEqual(refreshes, 0);
  });

  it("moves only the selected installation to Trash and refreshes", async () => {
    choice = "Move to Trash";
    await provider.remove(item(script));
    assert.deepStrictEqual(deletions, [[vscode.Uri.file(directory).fsPath, { recursive: true, useTrash: true }]]);
    assert.deepStrictEqual(updates, []);
    assert.strictEqual(refreshes, 1);
  });

  it("clears matching global and folder settings but preserves another workspace distribution", async () => {
    choice = "Move to Trash";
    globalScript = script;
    folderScript = script;
    workspaceScript = path.join(root, "kilted", "setup.bash");
    replaceProperty(vscode.workspace, "workspaceFolders", [{ uri: vscode.Uri.file(sandbox) }]);
    await provider.remove(item(script));
    assert.deepStrictEqual(updates, [
      ["distro", undefined, vscode.ConfigurationTarget.Global],
      ["rosSetupScript", undefined, vscode.ConfigurationTarget.Global],
      ["distro", undefined, vscode.ConfigurationTarget.WorkspaceFolder],
      ["rosSetupScript", undefined, vscode.ConfigurationTarget.WorkspaceFolder],
    ]);
  });

  it("clears an active workspace selection and offers a reload", async () => {
    choice = "Move to Trash";
    activeScript = script;
    workspaceScript = script;
    const commands: string[] = [];
    replaceProperty(vscode.window, "showInformationMessage", async () => "Reload Window");
    replaceProperty(vscode.commands, "executeCommand", async (command: string) => { commands.push(command); });
    await provider.remove(item(script));
    assert.deepStrictEqual(updates, [
      ["distro", undefined, vscode.ConfigurationTarget.Workspace],
      ["rosSetupScript", undefined, vscode.ConfigurationTarget.Workspace],
    ]);
    assert.ok(commands.includes("workbench.action.reloadWindow"));
  });

  it("never deletes an unmanaged installation or the Pixi root", async () => {
    choice = "Move to Trash";
    await provider.remove(item(path.join(sandbox, "setup.bash")));
    await provider.remove(item(path.join(root, "setup.bash")));
    await fs.unlink(path.join(directory, "pixi.toml"));
    await provider.remove(item(script));
    assert.deepStrictEqual(deletions, []);
    assert.deepStrictEqual(updates, []);
  });

  it("removes the last installation while the previous reload notification is open", async () => {
    choice = "Move to Trash";
    activeScript = script;
    workspaceScript = script;
    const secondDirectory = path.join(root, "rolling");
    const secondScript = path.join(secondDirectory, "setup.bash");
    await fs.mkdir(path.join(secondDirectory, ".pixi"), { recursive: true });
    await fs.writeFile(path.join(secondDirectory, "pixi.toml"), "[workspace]\nname = 'rolling'\n");
    await fs.writeFile(secondScript, "");
    replaceProperty(vscode.workspace, "fs", {
      ...vscode.workspace.fs,
      delete: async (uri: vscode.Uri, options: { recursive: boolean; useTrash: boolean }) => {
        deletions.push([uri.fsPath, options]);
        await fs.rename(uri.fsPath, path.join(sandbox, `${path.basename(uri.fsPath)}-trashed`));
      },
    });
    let dismiss: (choice: undefined) => void;
    let notify: () => void;
    const notificationShown = new Promise<void>(resolve => { notify = resolve; });
    replaceProperty(vscode.window, "showInformationMessage", () => {
      notify();
      return new Promise(resolve => { dismiss = resolve; });
    });
    const firstRemoval = provider.remove(item(script));
    try {
      await notificationShown;
      await provider.remove(new RosDistributionItem("rolling (pixi)", secondScript, false));
      assert.strictEqual(deletions.length, 2, "Reload notifications must not block subsequent removal");
      assert.deepStrictEqual(await fs.readdir(root), [], "The final installation must also be removed");
      assert.strictEqual(refreshes, 2);
      assert.ok(deletions.every(([, options]) => options.useTrash));
    } finally {
      dismiss(undefined);
      await firstRemoval;
    }
  });

  it("does not hold the removal lock while an unsupported-installation warning is open", async () => {
    let dismiss: (choice: undefined) => void;
    let notify: () => void;
    const warningShown = new Promise<void>(resolve => { notify = resolve; });
    replaceProperty(vscode.window, "showWarningMessage", async (...args: unknown[]) => {
      if (args.length === 1) {
        notify();
        return new Promise(resolve => { dismiss = resolve; });
      }
      return "Move to Trash";
    });
    const unsupportedRemoval = provider.remove(item(path.join(sandbox, "setup.bash")));
    try {
      await warningShown;
      await provider.remove(item(script));
      assert.strictEqual(deletions.length, 1);
    } finally {
      dismiss(undefined);
      await unsupportedRemoval;
    }
  });

  it("still blocks concurrent removals while confirmation is pending", async () => {
    let confirm: (choice: string) => void;
    let notify: () => void;
    const confirmationShown = new Promise<void>(resolve => { notify = resolve; });
    let confirmations = 0;
    replaceProperty(vscode.window, "showWarningMessage", () => {
      confirmations++;
      notify();
      return new Promise(resolve => { confirm = resolve; });
    });
    const firstRemoval = provider.remove(item(script));
    try {
      await confirmationShown;
      await provider.remove(item(script));
      assert.strictEqual(confirmations, 1);
      assert.deepStrictEqual(deletions, []);
    } finally {
      confirm("Move to Trash");
      await firstRemoval;
    }
    assert.strictEqual(deletions.length, 1);
  });

  it("refuses a directory containing an open workspace", async () => {
    choice = "Move to Trash";
    replaceProperty(vscode.workspace, "workspaceFolders", [{ uri: vscode.Uri.file(path.join(directory, ".pixi")) }]);
    await provider.remove(item(script));
    assert.deepStrictEqual(deletions, []);
  });

  it("rejects symlinked installation directories", async () => {
    choice = "Move to Trash";
    const linkedDirectory = path.join(root, "linked");
    await fs.symlink(directory, linkedDirectory, process.platform === "win32" ? "junction" : "dir");
    await provider.remove(item(path.join(linkedDirectory, "setup.bash")));
    assert.deepStrictEqual(deletions, []);
  });

  it("supports Windows Pixi environment setup layout", async () => {
    choice = "Move to Trash";
    const windowsScript = path.join(directory, ".pixi", "envs", "jazzy", "Library", "local_setup.bat");
    await fs.mkdir(path.dirname(windowsScript), { recursive: true });
    await fs.writeFile(windowsScript, "");
    await provider.remove(item(windowsScript));
    assert.strictEqual(deletions[0][0], vscode.Uri.file(directory).fsPath);
  });

  it("revalidates the installation after confirmation", async () => {
    replaceProperty(vscode.window, "showWarningMessage", async () => {
      await fs.unlink(path.join(directory, "pixi.toml"));
      return "Move to Trash";
    });
    await assert.rejects(provider.remove(item(script)), /installation changed/);
    assert.deepStrictEqual(deletions, []);
  });

  it("preserves settings when Trash fails and permits retry", async () => {
    choice = "Move to Trash";
    globalScript = script;
    replaceProperty(vscode.workspace, "fs", {
      ...vscode.workspace.fs,
      delete: async () => { throw new Error("Trash unavailable"); },
    });
    await assert.rejects(provider.remove(item(script)), /Trash unavailable/);
    await assert.rejects(provider.remove(item(script)), /Trash unavailable/);
    assert.deepStrictEqual(updates, []);
    assert.strictEqual(refreshes, 0);
  });
});