// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as path from "path";
import * as fs from "fs";
import * as os from "os";
import * as vscode from "vscode";
import { describe as suite, it as test, beforeEach as setup, afterEach as teardown } from "mocha";

import * as install_ros from "../../src/ros/installer/install-ros";
import { RosDistributionsProvider } from "../../src/ros/ros-distributions-provider";

suite("Installer terminal retention", () => {
  let restore: (() => void)[];
  let errors: string[];
  let directory: string;
  let script: string;
  let log: string;
  let installedSetup: string;
  const distro = install_ros.ROS2_DISTROS[0];

  function replaceProperty(object: object, key: string, value: unknown): void {
    const descriptor = Object.getOwnPropertyDescriptor(object, key)!;
    Object.defineProperty(object, key, { configurable: true, value });
    restore.push(() => Object.defineProperty(object, key, descriptor));
  }

  setup(async () => {
    restore = [];
    errors = [];
    directory = await fs.promises.mkdtemp(path.join(os.tmpdir(), "installer-cleanup-"));
    script = path.join(directory, "install.sh");
    log = path.join(directory, "install.log");
    installedSetup = path.join(directory, "setup.bash");
    await Promise.all([script, log, installedSetup].map(file => fs.promises.writeFile(file, "fixture")));
    replaceProperty(vscode.window, "showErrorMessage", async (message: string) => { errors.push(message); });
    replaceProperty(vscode.window, "showInformationMessage", async () => undefined);
  });

  teardown(async () => {
    restore.reverse().forEach(restoreProperty => restoreProperty());
    await fs.promises.rm(directory, { recursive: true, force: true });
  });

  for (const exitCode of [0, 1, undefined]) {
    test(`handles process exit ${exitCode} without waiting for terminal closure`, async () => {
      let listener: (event: vscode.TaskProcessEndEvent) => Promise<void>;
      let disposed = false;
      let task: vscode.Task;
      let complete = 0;
      let failed = 0;
      let configured = 0;
      replaceProperty(vscode.tasks, "onDidEndTaskProcess", callback => {
        listener = callback;
        return { dispose: () => { disposed = true; } };
      });
      replaceProperty(vscode.tasks, "executeTask", async (created: vscode.Task) => {
        assert.ok(listener, "Completion listener must be registered before starting the task");
        task = created;
        return { task, terminate: () => {} };
      });
      await install_ros.runInstallationTask("exit 1", { executable: "/bin/bash", shellArgs: ["-c"] }, distro, {
        markComplete: () => { complete++; },
        markFailed: () => { failed++; },
      }, log, async () => { configured++; }, script);
      assert.strictEqual(task.presentationOptions.close, false);
      assert.strictEqual(task.presentationOptions.clear, false);
      assert.strictEqual(task.presentationOptions.panel, vscode.TaskPanelKind.New);
      assert.strictEqual(task.presentationOptions.showReuseMessage, false);
      await listener({ execution: { task: {} as vscode.Task, terminate: () => {} }, exitCode });
      assert.strictEqual(disposed, false, "Unrelated tasks must not complete the install");
      assert.ok(fs.existsSync(script), "Running install scripts must be preserved");
      await listener({ execution: { task, terminate: () => {} }, exitCode });
      assert.ok(!fs.existsSync(script));
      assert.ok(fs.existsSync(log));
      assert.ok(fs.existsSync(installedSetup));
      assert.strictEqual(disposed, true);
      assert.strictEqual(complete, exitCode === 0 ? 1 : 0);
      assert.strictEqual(failed, exitCode === 0 ? 0 : 1);
      assert.strictEqual(configured, exitCode === 0 ? 1 : 0);
      if (exitCode === 1) { assert.ok(errors[0].includes("kept open")); }
    });
  }

  test("releases the installation when task startup fails", async () => {
    let failed = 0;
    let disposed = false;
    replaceProperty(vscode.tasks, "onDidEndTaskProcess", () => ({ dispose: () => { disposed = true; } }));
    replaceProperty(vscode.tasks, "executeTask", async () => { throw new Error("Task startup failed"); });
    await assert.rejects(install_ros.runInstallationTask("exit 1", {}, distro, {
      markComplete: () => assert.fail("Must not mark complete"),
      markFailed: () => { failed++; },
    }, log, undefined, script), /Task startup failed/);
    assert.ok(!fs.existsSync(script));
    assert.ok(fs.existsSync(log));
    assert.ok(fs.existsSync(installedSetup));
    assert.strictEqual(failed, 1);
    assert.strictEqual(disposed, true);
  });

  test("cleans cancelled tasks without a process event and handles duplicate events once", async () => {
    let listener: (event: vscode.TaskEndEvent) => Promise<void>;
    let processListener: (event: vscode.TaskProcessEndEvent) => Promise<void>;
    let taskExecution: vscode.TaskExecution;
    let failed = 0;
    replaceProperty(vscode.tasks, "onDidEndTask", callback => {
      listener = callback;
      return { dispose: () => {} };
    });
    replaceProperty(vscode.tasks, "onDidEndTaskProcess", callback => {
      processListener = callback;
      return { dispose: () => {} };
    });
    replaceProperty(vscode.tasks, "executeTask", async (task: vscode.Task) => {
      taskExecution = { task, terminate: () => {} };
      return taskExecution;
    });
    await install_ros.runInstallationTask("exit 1", {}, distro, {
      markComplete: () => assert.fail("Cancelled task must not complete"),
      markFailed: () => { failed++; },
    }, log, undefined, script);
    await listener({ execution: taskExecution });
    await processListener({ execution: taskExecution, exitCode: 1 });
    assert.strictEqual(failed, 1);
    assert.ok(!fs.existsSync(script));
    assert.ok(fs.existsSync(log));
    assert.ok(fs.existsSync(installedSetup));
  });

  test("cleanup errors do not hide the original task launch failure", async () => {
    replaceProperty(vscode.tasks, "executeTask", async () => { throw new Error("Task startup failed"); });
    for (const target of [path.join(directory, "already-removed.sh"), directory]) {
      let failed = 0;
      await assert.rejects(install_ros.runInstallationTask("exit 1", {}, distro, {
        markComplete: () => assert.fail("Must not mark complete"),
        markFailed: () => { failed++; },
      }, log, undefined, target), /Task startup failed/);
      assert.strictEqual(failed, 1);
    }
    assert.ok(fs.existsSync(log));
  });

  test("keeps a real failed installation task terminal available", async function () {
    if (process.platform === "win32") { this.skip(); }
    const displayName = `Retention Test ${Date.now()}`;
    const name = `ROS 2 ${displayName} Installation`;
    let failed = 0;
    let timer: NodeJS.Timeout;
    let listener: vscode.Disposable;
    let markFinished: () => void;
    const finished = new Promise<void>(resolve => { markFinished = resolve; });
    const ended = new Promise<void>((resolve, reject) => {
      timer = setTimeout(() => reject(new Error("Installation task did not finish")), 15000);
      listener = vscode.tasks.onDidEndTask(event => {
        if (event.execution.task.name === name) { resolve(); }
      });
    });
    try {
      await install_ros.runInstallationTask("printf 'Simulated ROS install error\\n'; exit 7", {
        executable: "/bin/bash", shellArgs: ["--noprofile", "--norc", "-c"],
      }, { ...distro, displayName }, {
        markComplete: () => assert.fail("Failed task must not mark complete"),
        markFailed: () => { failed++; markFinished(); },
      }, log, undefined, script);
      await ended;
      await finished;
      assert.strictEqual(failed, 1);
      assert.ok(!fs.existsSync(script));
      assert.ok(fs.existsSync(log));
      assert.ok(errors.some(message => message.includes("exit code: 7")));
      assert.ok(vscode.window.terminals.some(terminal => terminal.name.includes(displayName)), "Failed installer terminal must remain visible after the task ends");
    } finally {
      clearTimeout(timer);
      listener.dispose();
      vscode.window.terminals.filter(terminal => terminal.name.includes(displayName)).forEach(terminal => terminal.dispose());
    }
  });
});

describe("Install ROS Test Suite", () => {
  let testWorkspaceFolder: string;

  beforeEach(async () => {
    // Create a temporary test workspace in the system temp directory
    testWorkspaceFolder = path.join(os.tmpdir(), "test-workspace-install-" + Date.now());
    if (!fs.existsSync(testWorkspaceFolder)) {
      fs.mkdirSync(testWorkspaceFolder, { recursive: true });
    }
  });

  afterEach(async () => {
    // Clean up test workspace
    if (fs.existsSync(testWorkspaceFolder)) {
      fs.rmSync(testWorkspaceFolder, { recursive: true, force: true });
    }
  });

  it("Should detect ROS workspace with package.xml in root", async () => {
    // Create a package.xml file
    const packageXmlPath = path.join(testWorkspaceFolder, "package.xml");
    const packageXmlContent = `<?xml version="1.0"?>
<package format="3">
  <name>test_package</name>
  <version>0.0.1</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
</package>`;
    fs.writeFileSync(packageXmlPath, packageXmlContent);

    // Mock workspace folders
    const originalWorkspaceFolders = vscode.workspace.workspaceFolders;
    Object.defineProperty(vscode.workspace, "workspaceFolders", {
      configurable: true,
      get: () => [{ uri: { fsPath: testWorkspaceFolder } }],
    });

    try {
      const isRos = await install_ros.isRosWorkspace();
      assert.strictEqual(isRos, true, "Should detect ROS workspace with package.xml");
    } finally {
      // Restore original workspace folders
      Object.defineProperty(vscode.workspace, "workspaceFolders", {
        configurable: true,
        get: () => originalWorkspaceFolders,
      });
    }
  });

  it("Should detect ROS workspace with package.xml in src subdirectory", async () => {
    // Create a src directory and package.xml inside it
    const srcPath = path.join(testWorkspaceFolder, "src", "test_package");
    fs.mkdirSync(srcPath, { recursive: true });

    const packageXmlPath = path.join(srcPath, "package.xml");
    const packageXmlContent = `<?xml version="1.0"?>
<package format="3">
  <name>test_package</name>
  <version>0.0.1</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
</package>`;
    fs.writeFileSync(packageXmlPath, packageXmlContent);

    // Mock workspace folders
    const originalWorkspaceFolders = vscode.workspace.workspaceFolders;
    Object.defineProperty(vscode.workspace, "workspaceFolders", {
      configurable: true,
      get: () => [{ uri: { fsPath: testWorkspaceFolder } }],
    });

    try {
      const isRos = await install_ros.isRosWorkspace();
      assert.strictEqual(isRos, true, "Should detect ROS workspace with package.xml in src");
    } finally {
      // Restore original workspace folders
      Object.defineProperty(vscode.workspace, "workspaceFolders", {
        configurable: true,
        get: () => originalWorkspaceFolders,
      });
    }
  });

  it("Should not detect ROS workspace without package.xml", async () => {
    // Create an empty workspace
    // Mock workspace folders
    const originalWorkspaceFolders = vscode.workspace.workspaceFolders;
    Object.defineProperty(vscode.workspace, "workspaceFolders", {
      configurable: true,
      get: () => [{ uri: { fsPath: testWorkspaceFolder } }],
    });

    try {
      const isRos = await install_ros.isRosWorkspace();
      assert.strictEqual(isRos, false, "Should not detect ROS workspace without package.xml");
    } finally {
      // Restore original workspace folders
      Object.defineProperty(vscode.workspace, "workspaceFolders", {
        configurable: true,
        get: () => originalWorkspaceFolders,
      });
    }
  });

  it("Should not detect ROS workspace when no workspace is open", async () => {
    // Mock no workspace folders
    const originalWorkspaceFolders = vscode.workspace.workspaceFolders;
    Object.defineProperty(vscode.workspace, "workspaceFolders", {
      configurable: true,
      get: () => undefined,
    });

    try {
      const isRos = await install_ros.isRosWorkspace();
      assert.strictEqual(isRos, false, "Should not detect ROS workspace when no workspace is open");
    } finally {
      // Restore original workspace folders
      Object.defineProperty(vscode.workspace, "workspaceFolders", {
        configurable: true,
        get: () => originalWorkspaceFolders,
      });
    }
  });

  it("ROS2_DISTROS should contain LTS and non-LTS releases", () => {
    const ltsDistros = install_ros.ROS2_DISTROS.filter((d) => d.isLTS);
    const nonLtsDistros = install_ros.ROS2_DISTROS.filter((d) => !d.isLTS);

    assert.ok(ltsDistros.length > 0, "Should have at least one LTS distro");
    assert.ok(nonLtsDistros.length > 0, "Should have at least one non-LTS distro");
  });

  test("macOS discovers verified Pixi setup but not incomplete installations", async function () {
    if (process.platform !== "darwin") { this.skip(); }
    const config = vscode.workspace.getConfiguration("ROS2");
    const originalRoot = config.inspect<string>("pixiRoot")?.workspaceValue;
    const provider = new RosDistributionsProvider();
    try {
      const completed = path.join(testWorkspaceFolder, "jazzy");
      const pending = path.join(testWorkspaceFolder, "humble");
      await fs.promises.mkdir(completed, { recursive: true });
      await fs.promises.mkdir(pending, { recursive: true });
      await fs.promises.writeFile(path.join(completed, "setup.bash"), "export ROS_DISTRO=jazzy\n");
      await fs.promises.writeFile(path.join(pending, ".setup.bash"), "export ROS_DISTRO=humble\n");
      await config.update("pixiRoot", testWorkspaceFolder, vscode.ConfigurationTarget.Workspace);
      const distributions = await provider.getChildren();
      assert.ok(distributions.some(item => item.setupScript === path.join(completed, "setup.bash")));
      assert.ok(!distributions.some(item => item.setupScript.startsWith(pending)));
    } finally {
      provider.dispose();
      await config.update("pixiRoot", originalRoot, vscode.ConfigurationTarget.Workspace);
    }
  });

  it("ROS2_DISTROS should include Humble (LTS)", () => {
    const humble = install_ros.ROS2_DISTROS.find((d) => d.name === "humble");
    assert.ok(humble, "Should include Humble distro");
    assert.strictEqual(humble?.isLTS, true, "Humble should be marked as LTS");
  });

  it("ROS2_DISTROS should include Jazzy (LTS)", () => {
    const jazzy = install_ros.ROS2_DISTROS.find((d) => d.name === "jazzy");
    assert.ok(jazzy, "Should include Jazzy distro");
    assert.strictEqual(jazzy?.isLTS, true, "Jazzy should be marked as LTS");
  });

  it("ROS2_DISTROS should include non-LTS releases", () => {
    const iron = install_ros.ROS2_DISTROS.find((d) => d.name === "iron");
    const kilted = install_ros.ROS2_DISTROS.find((d) => d.name === "kilted");

    assert.ok(iron || kilted, "Should include at least one non-LTS distro (Iron or Kilted)");
  });
});
