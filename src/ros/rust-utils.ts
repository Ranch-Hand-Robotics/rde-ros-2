// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as path from "path";
import * as vscode from "vscode";
import * as util from "util";

import * as extension from "../extension";
import * as vscode_utils from "../vscode-utils";
import * as ros_utils from "./utils";

const promisifiedExec = util.promisify(child_process.exec);

/**
 * Checks if Rust is installed and returns version information
 * @returns Object with installed status, version, and minimum version check
 */
export async function checkRustInstallation(): Promise<{
    installed: boolean;
    version?: string;
    meetsMinimumVersion?: boolean;
    cargoInstalled?: boolean;
}> {
    try {
        const rustcResult = await promisifiedExec("rustc --version");
        const version = rustcResult.stdout.trim();
        
        // Check for cargo as well
        let cargoInstalled = false;
        try {
            await promisifiedExec("cargo --version");
            cargoInstalled = true;
        } catch {
            cargoInstalled = false;
        }
        
        // ROS 2 Rust typically requires Rust 1.70.0 or higher
        // Extract version number from "rustc 1.xx.x ..."
        const versionMatch = version.match(/rustc (\d+\.\d+\.\d+)/);
        let meetsMinimumVersion = false;
        
        if (versionMatch) {
            const versionParts = versionMatch[1].split('.').map(Number);
            const major = versionParts[0];
            const minor = versionParts[1];
            
            // Require Rust 1.70.0 or higher
            meetsMinimumVersion = major > 1 || (major === 1 && minor >= 70);
        }
        
        return {
            installed: true,
            version: version,
            meetsMinimumVersion: meetsMinimumVersion,
            cargoInstalled: cargoInstalled
        };
    } catch (error) {
        return {
            installed: false
        };
    }
}

/**
 * Checks if a ROS 2 package is a Rust package by looking for Cargo.toml
 * @param packagePath Path to the package directory (directory containing package.xml)
 * @returns true if the package contains a Cargo.toml file
 */
export function isRustPackage(packagePath: string): boolean {
    const cargoTomlPath = path.join(packagePath, "Cargo.toml");
    return fs.existsSync(cargoTomlPath);
}

/**
 * Initializes a ROS 2 Rust workspace by cloning required repositories
 * @param workspaceRoot The workspace root directory
 */
export async function initializeRustWorkspace(workspaceRoot: string): Promise<void> {
    const distro = extension.env.ROS_DISTRO || "rolling";
    
    extension.outputChannel.appendLine(`Initializing ROS 2 Rust workspace for distro: ${distro}`);
    
    // Create src directory if it doesn't exist
    const srcDir = path.join(workspaceRoot, "src");
    if (!fs.existsSync(srcDir)) {
        fs.mkdirSync(srcDir, { recursive: true });
    }
    
    // Determine which repos to clone based on ROS distro
    const repos: { [key: string]: string } = {};
    
    if (distro === "rolling" || distro === "kilted") {
        // For Rolling and Kilted, we need to clone rosidl_rust temporarily
        repos["rosidl_rust"] = "https://github.com/ros2-rust/rosidl_rust.git";
        
        // Also clone examples
        repos["examples"] = "https://github.com/ros2-rust/examples.git";
    } else if (distro === "jazzy") {
        // For Jazzy, clone required interface packages
        repos["common_interfaces"] = "https://github.com/ros2/common_interfaces.git#jazzy";
        repos["example_interfaces"] = "https://github.com/ros2/example_interfaces.git#jazzy";
        repos["rcl_interfaces"] = "https://github.com/ros2/rcl_interfaces.git#jazzy";
        repos["rosidl_core"] = "https://github.com/ros2/rosidl_core.git#jazzy";
        repos["rosidl_defaults"] = "https://github.com/ros2/rosidl_defaults.git#jazzy";
        repos["unique_identifier_msgs"] = "https://github.com/ros2/unique_identifier_msgs.git#jazzy";
        repos["rosidl_rust"] = "https://github.com/ros2-rust/rosidl_rust.git";
        repos["examples"] = "https://github.com/ros2-rust/examples.git";
    } else if (distro === "humble") {
        // For Humble, clone required interface packages
        repos["common_interfaces"] = "https://github.com/ros2/common_interfaces.git#humble";
        repos["example_interfaces"] = "https://github.com/ros2/example_interfaces.git#humble";
        repos["rcl_interfaces"] = "https://github.com/ros2/rcl_interfaces.git#humble";
        repos["rosidl_core"] = "https://github.com/ros2/rosidl_core.git#humble";
        repos["rosidl_defaults"] = "https://github.com/ros2/rosidl_defaults.git#humble";
        repos["unique_identifier_msgs"] = "https://github.com/ros2/unique_identifier_msgs.git#humble";
        repos["rosidl_rust"] = "https://github.com/ros2-rust/rosidl_rust.git";
        repos["examples"] = "https://github.com/ros2-rust/examples.git";
    } else {
        throw new Error(`Unsupported ROS distro for Rust: ${distro}`);
    }
    
    // Clone each repository
    for (const [repoName, repoUrl] of Object.entries(repos)) {
        const repoPath = path.join(srcDir, repoName);
        
        // Skip if already exists
        if (fs.existsSync(repoPath)) {
            extension.outputChannel.appendLine(`  Repository ${repoName} already exists, skipping...`);
            continue;
        }
        
        extension.outputChannel.appendLine(`  Cloning ${repoName}...`);
        
        try {
            // Parse branch from URL if present
            const [url, branch] = repoUrl.includes("#") ? repoUrl.split("#") : [repoUrl, null];
            
            const cloneCmd = branch 
                ? `git clone -b ${branch} ${url} ${repoPath}`
                : `git clone ${url} ${repoPath}`;
            
            await promisifiedExec(cloneCmd);
            extension.outputChannel.appendLine(`    ✓ Cloned ${repoName}`);
        } catch (error) {
            extension.outputChannel.appendLine(`    ✗ Failed to clone ${repoName}: ${error}`);
            throw error;
        }
    }
    
    extension.outputChannel.appendLine(`Rust ROS 2 workspace initialization complete!`);
}

/**
 * Installs Rust using rustup in a ROS terminal
 * @param context VS Code extension context
 */
export async function installRustInTerminal(context: vscode.ExtensionContext): Promise<void> {
    const terminal = vscode.window.createTerminal({
        name: 'ROS 2 - Rust Installation',
        env: extension.env
    });
    
    terminal.show();
    
    // Send installation commands to terminal
    terminal.sendText("# Installing Rust...");
    terminal.sendText("curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh");
    terminal.sendText("# After installation completes, install colcon plugins:");
    terminal.sendText("pip install colcon-cargo colcon-ros-cargo");
    
    vscode.window.showInformationMessage(
        "Rust installation started in terminal. Follow the prompts to complete installation."
    );
}

/**
 * Updates Rust to the latest version in a ROS terminal
 * @param context VS Code extension context
 */
export async function updateRustInTerminal(context: vscode.ExtensionContext): Promise<void> {
    const terminal = vscode.window.createTerminal({
        name: 'ROS 2 - Rust Update',
        env: extension.env
    });
    
    terminal.show();
    
    // Send update commands to terminal
    terminal.sendText("# Updating Rust...");
    terminal.sendText("rustup update");
    
    vscode.window.showInformationMessage(
        "Rust update started in terminal."
    );
}
