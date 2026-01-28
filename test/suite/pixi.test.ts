// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as path from "path";
import * as vscode from "vscode";

import * as vscode_utils from "../../src/vscode-utils";
import * as ros_utils from "../../src/ros/utils";

describe("ROS Setup Script Configuration Tests", () => {
    
    it.skip("getRosSetupScript returns platform-appropriate default path", () => {
        const config = vscode_utils.getExtensionConfiguration();
        
        // Clear any existing configuration to test defaults
        const originalRosSetupScript = config.get("rosSetupScript");
        
        try {
            // Test with empty configuration (should use defaults)
            config.update("rosSetupScript", "", vscode.ConfigurationTarget.Global);
            
            const setupScriptPath = vscode_utils.getRosSetupScript();
            
            if (process.platform === "win32") {
                // Should default to Pixi path on Windows
                const defaultPixiRoot = config.get("pixiRoot", "c:\\pixi_ws");
                assert.ok(setupScriptPath.includes(defaultPixiRoot), "Should use Windows Pixi default path");
                assert.ok(setupScriptPath.includes("ros2-windows"), "Should include platform in path");
                assert.ok(setupScriptPath.endsWith(".bat"), "Should end with .bat on Windows");
                assert.ok(setupScriptPath.includes("local_setup"), "Should include setup script name");
            } else {
                // Should return empty string on Unix-like systems (requiring user configuration)
                assert.strictEqual(setupScriptPath, "", "Should require user configuration on Unix-like systems");
            }
        } finally {
            // Restore original configuration
            config.update("rosSetupScript", originalRosSetupScript, vscode.ConfigurationTarget.Global);
        }
    });
    
    it.skip("getRosSetupScript respects custom configuration", () => {
        const config = vscode_utils.getExtensionConfiguration();
        const originalRosSetupScript = config.get("rosSetupScript");
        const customPath = process.platform === "win32" ? "D:\\custom\\ros\\setup.bat" : "/opt/ros/humble/setup.bash";
        
        try {
            config.update("rosSetupScript", customPath, vscode.ConfigurationTarget.Global);
            
            const setupScriptPath = vscode_utils.getRosSetupScript();
            const normalizedCustomPath = path.normalize(customPath);
            
            assert.strictEqual(setupScriptPath, normalizedCustomPath, "Should use custom path");
            
        } finally {
            // Restore original configuration
            config.update("rosSetupScript", originalRosSetupScript, vscode.ConfigurationTarget.Global);
        }
    });
    
    it("getRosSetupScript handles workspaceFolder variable substitution", async () => {
        const config = vscode_utils.getExtensionConfiguration();
        const originalRosSetupScript = config.get("rosSetupScript");
        const scriptWithVariable = "${workspaceFolder}/install/setup.bash";
        
        try {
            // Use Workspace target to avoid Global/user overrides; await update so value is applied
            await config.update("rosSetupScript", scriptWithVariable, vscode.ConfigurationTarget.Workspace);
            
            const setupScriptPath = vscode_utils.getRosSetupScript();
            
            if (vscode.workspace.workspaceFolders && vscode.workspace.workspaceFolders.length === 1) {
                const expectedPath = path.normalize(
                    scriptWithVariable.replace("${workspaceFolder}", vscode.workspace.workspaceFolders[0].uri.fsPath)
                );
                assert.strictEqual(setupScriptPath, expectedPath, "Should substitute workspaceFolder variable");
                assert.ok(!setupScriptPath.includes("${workspaceFolder}"), "Should not contain unsubstituted variable");
            }
            
        } finally {
            // Restore original configuration, awaiting persistence
            await config.update("rosSetupScript", originalRosSetupScript, vscode.ConfigurationTarget.Workspace);
        }
    });
    
    it("detectUserShell returns appropriate shell info", () => {
        const shellInfo = ros_utils.detectUserShell();
        
        assert.ok(shellInfo.name, "Should have shell name");
        assert.ok(shellInfo.executable, "Should have shell executable");
        assert.ok(shellInfo.scriptExtension, "Should have script extension");
        assert.ok(shellInfo.sourceCommand, "Should have source command");
        
        if (process.platform === "win32") {
            assert.strictEqual(shellInfo.name, "cmd", "Should be cmd on Windows");
            assert.strictEqual(shellInfo.scriptExtension, ".bat", "Should use .bat extension on Windows");
            assert.strictEqual(shellInfo.sourceCommand, "call", "Should use call command on Windows");
        } else {
            // Unix-like systems
            assert.ok(["bash", "zsh", "fish", "sh", "csh"].includes(shellInfo.name), "Should be a recognized Unix shell");
            assert.ok(shellInfo.scriptExtension.startsWith("."), "Script extension should start with dot");
            assert.ok(["source", "."].includes(shellInfo.sourceCommand), "Should use appropriate source command");
        }
    });
    
    it.skip("pixiRoot configuration exists and defaults to c:\\pixi_ws", () => {
        const config = vscode_utils.getExtensionConfiguration();
        
        // Should be able to get the configuration value (default c:\pixi_ws)
        const pixiRoot = config.get("pixiRoot", undefined);
        assert.notStrictEqual(pixiRoot, undefined, "pixiRoot setting should exist");
        assert.strictEqual(config.get("pixiRoot"), "c:\\pixi_ws", "pixiRoot should default to c:\\pixi_ws");
        
        // Test setting it to a custom path
        const originalValue = config.get("pixiRoot");
        try {
            config.update("pixiRoot", "D:\\custom\\pixi", vscode.ConfigurationTarget.Global);
            assert.strictEqual(config.get("pixiRoot"), "D:\\custom\\pixi", "Should be able to set custom pixiRoot");
            
            // Test that getRosSetupScript uses the custom path on Windows
            if (process.platform === "win32") {
                config.update("rosSetupScript", "", vscode.ConfigurationTarget.Global);
                const setupScript = vscode_utils.getRosSetupScript();
                assert.ok(setupScript.includes("D:\\custom\\pixi"), "Should use custom pixiRoot in setup script path");
            }
        } finally {
            // Restore original value
            config.update("pixiRoot", originalValue, vscode.ConfigurationTarget.Global);
            config.update("rosSetupScript", "", vscode.ConfigurationTarget.Global);
        }
    });

    it.skip("pixiRoot configuration exists and has correct default", () => {
        const config = vscode_utils.getExtensionConfiguration();
        
        // Should be able to get the configuration value
        const pixiRoot = config.get("pixiRoot", undefined);
        assert.notStrictEqual(pixiRoot, undefined, "pixiRoot setting should exist");
        assert.strictEqual(config.get("pixiRoot"), "c:\\pixi_ws", "pixiRoot should default to c:\\pixi_ws");
        
        // Test setting it to a custom value
        const originalValue = config.get("pixiRoot");
        try {
            config.update("pixiRoot", "/custom/pixi/path", vscode.ConfigurationTarget.Global);
            const newValue = config.get("pixiRoot");
            assert.strictEqual(newValue, "/custom/pixi/path", "Should be able to set pixiRoot to custom path");
        } finally {
            // Restore original value
            config.update("pixiRoot", originalValue, vscode.ConfigurationTarget.Global);
        }
    });
    
    it.skip("getRosSetupScript respects pixiRoot setting across all platforms", () => {
        const config = vscode_utils.getExtensionConfiguration();
        const originalRosSetupScript = config.get("rosSetupScript");
        const originalPixiRoot = config.get("pixiRoot");
        
        try {
            // Clear setup script to test defaults
            config.update("rosSetupScript", "", vscode.ConfigurationTarget.Global);
            
            // Test with pixiRoot set to custom value
            const customPixiRoot = "/custom/pixi/workspace";
            config.update("pixiRoot", customPixiRoot, vscode.ConfigurationTarget.Global);
            
            let setupScriptPath = vscode_utils.getRosSetupScript();
            
            // Should use pixiRoot on all platforms when set
            assert.ok(setupScriptPath.includes(customPixiRoot), "Should use custom pixiRoot on any platform");
            assert.ok(setupScriptPath.includes("ros2-windows"), "Should include ros2-windows subdirectory");
            
            // Test with pixiRoot cleared
            config.update("pixiRoot", "", vscode.ConfigurationTarget.Global);
            setupScriptPath = vscode_utils.getRosSetupScript();
            
            // Should return empty when pixiRoot is not set
            assert.strictEqual(setupScriptPath, "", "Should return empty when pixiRoot is not configured");
            
        } finally {
            // Restore original configuration
            config.update("rosSetupScript", originalRosSetupScript, vscode.ConfigurationTarget.Global);
            config.update("pixiRoot", originalPixiRoot, vscode.ConfigurationTarget.Global);
        }
    });
});