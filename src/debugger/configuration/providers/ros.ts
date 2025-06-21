// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import { rosApi } from "../../../ros/ros";

// interact with the user to create a roslaunch or rosrun configuration
export class RosDebugConfigurationProvider implements vscode.DebugConfigurationProvider {
    
    /**
     * Provides initial debug configurations for VS Code's debug dropdown
     * When called, triggers the interactive configuration flow
     */
    public async provideDebugConfigurations(
        folder: vscode.WorkspaceFolder | undefined,
        token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {
        
        if (token?.isCancellationRequested) {
            return [];
        }

        // When VS Code asks for initial configurations (user selected "ROS 2" from dropdown),
        // trigger the interactive flow to find actual packages and launch files
        return this.provideDebugConfigurationsInteractive(folder, token);
    }

    /**
     * Interactive configuration method for when user needs more detailed setup
     */
    public async provideDebugConfigurationsInteractive(
        folder: vscode.WorkspaceFolder | undefined,
        token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {
        const type = await vscode.window.showQuickPick(
            ["ROS2: ROS 2 Launch", "ROS2: Debug ROS 2 Launch File", "ROS2: ROS 2 Attach"], { placeHolder: "Choose a request type" });
        if (!type) {
            return [];
        }

        switch (type) {
            case "ROS2: Debug ROS 2 Launch File":
            case "ROS2: ROS 2 Launch": {
                const packageName = await vscode.window.showQuickPick(rosApi.getPackageNames(), {
                    placeHolder: "Choose a package",
                });
                if (!packageName) {
                    return [];
                }
                const launchFiles = (await rosApi.findPackageLaunchFiles(packageName)).concat(await rosApi.findPackageTestFiles(packageName));
                const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
                const target = await vscode.window.showQuickPick(
                    launchFileBasenames, { placeHolder: "Choose a launch file" });
                const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
                if (!launchFilePath) {
                    return [];
                }

                if (type === "ROS2: Debug ROS 2 Launch File") {
                    return [{
                        name: type,
                        request: "debug_launch",
                        target: `${launchFilePath}`,
                        type: "ros2",
                    }];
                } else {
                    return [{
                        name: type,
                        request: "launch",
                        target: `${launchFilePath}`,
                        launch: ["rviz", "gz"],
                        type: "ros2",
                    }];
                }
            }
            case "ROS2: ROS 2 Attach": {
                return [{
                    name: "ROS: Attach",
                    request: "attach",
                    type: "ros2",
                }];
            }
        }

        return [];
    }
}
