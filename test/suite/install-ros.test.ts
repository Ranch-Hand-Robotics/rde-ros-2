// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as path from "path";
import * as fs from "fs";
import * as vscode from "vscode";

import * as install_ros from "../../src/ros/install-ros";

suite("Install ROS Test Suite", () => {
  let testWorkspaceFolder: string;

  setup(async () => {
    // Create a temporary test workspace
    testWorkspaceFolder = path.join(__dirname, "..", "..", "..", "test-workspace-install");
    if (!fs.existsSync(testWorkspaceFolder)) {
      fs.mkdirSync(testWorkspaceFolder, { recursive: true });
    }
  });

  teardown(async () => {
    // Clean up test workspace
    if (fs.existsSync(testWorkspaceFolder)) {
      fs.rmSync(testWorkspaceFolder, { recursive: true, force: true });
    }
  });

  test("Should detect ROS workspace with package.xml in root", async () => {
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

  test("Should detect ROS workspace with package.xml in src subdirectory", async () => {
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

  test("Should not detect ROS workspace without package.xml", async () => {
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

  test("Should not detect ROS workspace when no workspace is open", async () => {
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

  test("ROS2_DISTROS should contain LTS and non-LTS releases", () => {
    const ltsDistros = install_ros.ROS2_DISTROS.filter((d) => d.isLTS);
    const nonLtsDistros = install_ros.ROS2_DISTROS.filter((d) => !d.isLTS);

    assert.ok(ltsDistros.length > 0, "Should have at least one LTS distro");
    assert.ok(nonLtsDistros.length > 0, "Should have at least one non-LTS distro");
  });

  test("ROS2_DISTROS should include Humble (LTS)", () => {
    const humble = install_ros.ROS2_DISTROS.find((d) => d.name === "humble");
    assert.ok(humble, "Should include Humble distro");
    assert.strictEqual(humble?.isLTS, true, "Humble should be marked as LTS");
  });

  test("ROS2_DISTROS should include Jazzy (LTS)", () => {
    const jazzy = install_ros.ROS2_DISTROS.find((d) => d.name === "jazzy");
    assert.ok(jazzy, "Should include Jazzy distro");
    assert.strictEqual(jazzy?.isLTS, true, "Jazzy should be marked as LTS");
  });

  test("ROS2_DISTROS should include non-LTS releases", () => {
    const iron = install_ros.ROS2_DISTROS.find((d) => d.name === "iron");
    const kilted = install_ros.ROS2_DISTROS.find((d) => d.name === "kilted");

    assert.ok(iron || kilted, "Should include at least one non-LTS distro (Iron or Kilted)");
  });
});
