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
    // Check for user-configured Rust/Cargo paths
    const config = vscode_utils.getExtensionConfiguration();
    const rustcPath = config.get<string>("rustcPath", "rustc");
    const cargoPath = config.get<string>("cargoPath", "cargo");
    
    try {
        const rustcResult = await promisifiedExec(`${rustcPath} --version`);
        const version = rustcResult.stdout.trim();
        
        // Check for cargo as well
        let cargoInstalled = false;
        try {
            await promisifiedExec(`${cargoPath} --version`);
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
 * Finds all package.xml files in a workspace recursively.
 * 
 * @param workspaceRoot Root directory of the workspace
 * @returns Array of package.xml file paths
 */
function findPackageXmlFiles(workspaceRoot: string): string[] {
    const packageXmlFiles: string[] = [];
    
    function searchDirectory(dirPath: string) {
        try {
            const entries = fs.readdirSync(dirPath, { withFileTypes: true });
            
            for (const entry of entries) {
                const fullPath = path.join(dirPath, entry.name);
                
                if (entry.isDirectory()) {
                    // Skip common directories that won't contain packages
                    if (['build', 'install', 'log', '.git', 'node_modules', '__pycache__'].includes(entry.name)) {
                        continue;
                    }
                    searchDirectory(fullPath);
                } else if (entry.isFile() && entry.name === 'package.xml') {
                    packageXmlFiles.push(fullPath);
                }
            }
        } catch (error) {
            // Ignore directories we can't read
        }
    }
    
    searchDirectory(workspaceRoot);
    return packageXmlFiles;
}

/**
 * Parses a package.xml file to extract package information.
 * 
 * @param packageXmlPath Path to the package.xml file
 * @returns Package information or null if parsing fails
 */
function parsePackageXml(packageXmlPath: string): { name: string; packageRoot: string } | null {
    try {
        const xmlContent = fs.readFileSync(packageXmlPath, 'utf8');
        const packageRoot = path.dirname(packageXmlPath);
        
        // Simple XML parsing to extract package name
        const nameMatch = xmlContent.match(/<name>\s*([^<]+)\s*<\/name>/);
        if (nameMatch && nameMatch[1]) {
            return {
                name: nameMatch[1].trim(),
                packageRoot: packageRoot
            };
        }
    } catch (error) {
        extension.outputChannel.appendLine(`Error parsing package.xml at ${packageXmlPath}: ${String(error)}`);
    }
    
    return null;
}

/**
 * Checks if an executable belongs to a Rust ROS 2 package.
 * 
 * @param executablePath Path to the executable
 * @returns true if the executable belongs to a Rust package, false otherwise
 */
export function isRustExecutable(executablePath: string): boolean {
    try {
        // Parse the executable path
        // Example: /path/to/workspace/install/package_name/lib/package_name/node_name
        const pathParts = executablePath.split(path.sep);
        
        // Find the 'install' directory in the path
        const installIndex = pathParts.lastIndexOf('install');
        if (installIndex === -1) {
            return false;
        }
        
        // Get package name (should be directory after 'install')
        if (installIndex + 1 >= pathParts.length) {
            return false;
        }
        
        const packageName = pathParts[installIndex + 1];
        const workspaceRoot = pathParts.slice(0, installIndex).join(path.sep);
        
        // Find package.xml files in workspace
        const packageXmlFiles = findPackageXmlFiles(workspaceRoot);
        
        // Find the package.xml that matches our package name
        for (const packageXmlPath of packageXmlFiles) {
            const packageInfo = parsePackageXml(packageXmlPath);
            if (packageInfo && packageInfo.name === packageName) {
                // Check if Cargo.toml exists in the package directory
                return isRustPackage(packageInfo.packageRoot);
            }
        }
        
        return false;
    } catch (error) {
        extension.outputChannel.appendLine(`Error checking if executable is Rust: ${error}`);
        return false;
    }
}

/**
 * Initializes a ROS 2 Rust workspace by cloning required repositories
 * @param workspaceRoot The workspace root directory
 * @param context VS Code extension context
 */
export async function initializeRustWorkspace(workspaceRoot: string, context: vscode.ExtensionContext): Promise<void> {
    const distro = extension.env.ROS_DISTRO || "rolling";
    
    extension.outputChannel.appendLine(`Initializing ROS 2 Rust workspace for distro: ${distro}`);
    
    // Create src directory if it doesn't exist
    const srcDir = path.join(workspaceRoot, "src");
    if (!fs.existsSync(srcDir)) {
        fs.mkdirSync(srcDir, { recursive: true });
    }
    
    // Determine which repos to clone based on ROS distro
    const repos: { [key: string]: string } = {};
    
    if (distro === "rolling" || distro === "kilted" || distro === "lyrical") {
        // For Rolling, Kilted, and Lyrical, we need to clone rosidl_rust temporarily
        repos["rosidl_rust"] = "https://github.com/ros2-rust/rosidl_rust.git";
    } else if (distro === "jazzy") {
        // For Jazzy, clone required interface packages
        repos["common_interfaces"] = "https://github.com/ros2/common_interfaces.git#jazzy";
        repos["example_interfaces"] = "https://github.com/ros2/example_interfaces.git#jazzy";
        repos["rcl_interfaces"] = "https://github.com/ros2/rcl_interfaces.git#jazzy";
        repos["rosidl_core"] = "https://github.com/ros2/rosidl_core.git#jazzy";
        repos["rosidl_defaults"] = "https://github.com/ros2/rosidl_defaults.git#jazzy";
        repos["unique_identifier_msgs"] = "https://github.com/ros2/unique_identifier_msgs.git#jazzy";
        repos["rosidl_rust"] = "https://github.com/ros2-rust/rosidl_rust.git";
    } else if (distro === "humble") {
        // For Humble, clone required interface packages
        repos["common_interfaces"] = "https://github.com/ros2/common_interfaces.git#humble";
        repos["example_interfaces"] = "https://github.com/ros2/example_interfaces.git#humble";
        repos["rcl_interfaces"] = "https://github.com/ros2/rcl_interfaces.git#humble";
        repos["rosidl_core"] = "https://github.com/ros2/rosidl_core.git#humble";
        repos["rosidl_defaults"] = "https://github.com/ros2/rosidl_defaults.git#humble";
        repos["unique_identifier_msgs"] = "https://github.com/ros2/unique_identifier_msgs.git#humble";
        repos["rosidl_rust"] = "https://github.com/ros2-rust/rosidl_rust.git";
    } else {
        throw new Error(`Unsupported ROS distro for Rust: ${distro}`);
    }
    
    // Create a ROS 2 terminal to run git clone commands
    const terminal = ros_utils.createTerminal(context);
    terminal.sendText(`cd "${srcDir}"`);
    
    // Clone each repository through the terminal
    for (const [repoName, repoUrl] of Object.entries(repos)) {
        const repoPath = path.join(srcDir, repoName);
        
        // Skip if already exists
        if (fs.existsSync(repoPath)) {
            extension.outputChannel.appendLine(`  Repository ${repoName} already exists, skipping...`);
            continue;
        }
        
        extension.outputChannel.appendLine(`  Cloning ${repoName}...`);
        
        // Parse branch from URL if present
        const [url, branch] = repoUrl.includes("#") ? repoUrl.split("#") : [repoUrl, null];
        
        const cloneCmd = branch 
            ? `git clone -b ${branch} ${url} ${repoName}`
            : `git clone ${url} ${repoName}`;
        
        terminal.sendText(cloneCmd);
    }
    
    terminal.sendText("# Rust ROS 2 workspace initialization complete!");
    extension.outputChannel.appendLine(`Rust ROS 2 workspace initialization commands sent to terminal.`);
}

/**
 * Installs Rust using rustup in a ROS terminal
 * @param context VS Code extension context
 */
export async function installRustInTerminal(context: vscode.ExtensionContext): Promise<void> {
    const terminal = ros_utils.createTerminal(context);
    
    // Send installation commands to terminal
    terminal.sendText("# Installing Rust...");
    terminal.sendText("curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh");
    terminal.sendText("# After installation completes, install colcon plugins:");
    terminal.sendText("pip install colcon-cargo colcon-ros-cargo");
    
    vscode.window.setStatusBarMessage(
        "$(sync~spin) Rust installation started in terminal. Follow the prompts to complete installation.",
        5000
    );
}

/**
 * Updates Rust to the latest version in a ROS terminal
 * @param context VS Code extension context
 */
export async function updateRustInTerminal(context: vscode.ExtensionContext): Promise<void> {
    const terminal = ros_utils.createTerminal(context);
    
    // Send update commands to terminal
    terminal.sendText("# Updating Rust...");
    terminal.sendText("rustup update");
    
    vscode.window.setStatusBarMessage(
        "$(sync~spin) Rust update started in terminal.",
        5000
    );
}
