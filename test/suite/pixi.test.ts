// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as path from "path";
import * as vscode from "vscode";

import * as vscode_utils from "../../src/vscode-utils";
import * as ros_utils from "../../src/ros/utils";

suite("ROS Setup Script Configuration Tests", () => {
    
    test("getRosSetupScript returns platform-appropriate default path", () => {
        const config = vscode_utils.getExtensionConfiguration();
        
        // Clear any existing configuration to test defaults
        const originalRosSetupScript = config.get("rosSetupScript");
        
        try {
            // Test with empty configuration (should use defaults)
            config.update("rosSetupScript", "", vscode.ConfigurationTarget.Global);
            
            const setupScriptPath = vscode_utils.getRosSetupScript();
            
            if (process.platform === "win32") {
                // Should default to Pixi path on Windows
                assert.ok(setupScriptPath.includes("c:\\pixi_ws"), "Should use Windows Pixi default path");
                assert.ok(setupScriptPath.includes("ros2-win32"), "Should include platform in path");
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
    
    test("getRosSetupScript respects custom configuration", () => {
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
    
    test("getRosSetupScript handles workspaceFolder variable substitution", () => {
        const config = vscode_utils.getExtensionConfiguration();
        const originalRosSetupScript = config.get("rosSetupScript");
        const scriptWithVariable = "${workspaceFolder}/install/setup.bash";
        
        try {
            config.update("rosSetupScript", scriptWithVariable, vscode.ConfigurationTarget.Global);
            
            const setupScriptPath = vscode_utils.getRosSetupScript();
            
            if (vscode.workspace.workspaceFolders && vscode.workspace.workspaceFolders.length === 1) {
                const expectedPath = path.normalize(
                    scriptWithVariable.replace("${workspaceFolder}", vscode.workspace.workspaceFolders[0].uri.fsPath)
                );
                assert.strictEqual(setupScriptPath, expectedPath, "Should substitute workspaceFolder variable");
                assert.ok(!setupScriptPath.includes("${workspaceFolder}"), "Should not contain unsubstituted variable");
            }
            
        } finally {
            // Restore original configuration
            config.update("rosSetupScript", originalRosSetupScript, vscode.ConfigurationTarget.Global);
        }
    });
    
    test("detectUserShell returns appropriate shell info", () => {
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
    
    test("usePixiOnAllPlatforms configuration exists and defaults to false", () => {
        const config = vscode_utils.getExtensionConfiguration();
        
        // Should be able to get the configuration value (default false)
        const usePixiOnAllPlatforms = config.get("usePixiOnAllPlatforms", undefined);
        assert.notStrictEqual(usePixiOnAllPlatforms, undefined, "usePixiOnAllPlatforms setting should exist");
        assert.strictEqual(config.get("usePixiOnAllPlatforms"), false, "usePixiOnAllPlatforms should default to false");
        
        // Test setting it to true
        const originalValue = config.get("usePixiOnAllPlatforms");
        try {
            config.update("usePixiOnAllPlatforms", true, vscode.ConfigurationTarget.Global);
            const newValue = config.get("usePixiOnAllPlatforms");
            assert.strictEqual(newValue, true, "Should be able to set usePixiOnAllPlatforms to true");
        } finally {
            // Restore original value
            config.update("usePixiOnAllPlatforms", originalValue, vscode.ConfigurationTarget.Global);
        }
    });
    
    test("getRosSetupScript respects usePixiOnAllPlatforms setting", () => {
        const config = vscode_utils.getExtensionConfiguration();
        const originalRosSetupScript = config.get("rosSetupScript");
        const originalUsePixiOnAllPlatforms = config.get("usePixiOnAllPlatforms");
        
        try {
            // Clear setup script to test defaults
            config.update("rosSetupScript", "", vscode.ConfigurationTarget.Global);
            
            // Test with usePixiOnAllPlatforms disabled (default)
            config.update("usePixiOnAllPlatforms", false, vscode.ConfigurationTarget.Global);
            let setupScriptPath = vscode_utils.getRosSetupScript();
            
            if (process.platform === "win32") {
                // Should still get default on Windows even when usePixiOnAllPlatforms is false
                assert.ok(setupScriptPath.includes("c:\\pixi_ws"), "Should use Windows default when usePixiOnAllPlatforms is false");
            } else {
                // Should return empty on Unix when usePixiOnAllPlatforms is false
                assert.strictEqual(setupScriptPath, "", "Should return empty on Unix when usePixiOnAllPlatforms is false");
            }
            
            // Test with usePixiOnAllPlatforms enabled
            config.update("usePixiOnAllPlatforms", true, vscode.ConfigurationTarget.Global);
            setupScriptPath = vscode_utils.getRosSetupScript();
            
            if (process.platform === "win32") {
                // Should still get default on Windows
                assert.ok(setupScriptPath.includes("c:\\pixi_ws"), "Should still use Windows default when usePixiOnAllPlatforms is true");
            } else {
                // Should still return empty on Unix (user must configure)
                assert.strictEqual(setupScriptPath, "", "Should still require user configuration on Unix even when usePixiOnAllPlatforms is true");
            }
            
        } finally {
            // Restore original configuration
            config.update("rosSetupScript", originalRosSetupScript, vscode.ConfigurationTarget.Global);
            config.update("usePixiOnAllPlatforms", originalUsePixiOnAllPlatforms, vscode.ConfigurationTarget.Global);
        }
    });
});