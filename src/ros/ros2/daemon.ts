// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../extension";
import * as ros2_monitor from "./ros2-monitor"

async function runDaemonCommand(action: "start" | "stop"): Promise<void> {
    const command = `ros2 daemon ${action}`;
    const exec = util.promisify(child_process.exec);
    extension.outputChannel.appendLine(`Running ${command} (ROS_DISTRO=${extension.env?.ROS_DISTRO || "unset"}, ROS_DOMAIN_ID=${extension.env?.ROS_DOMAIN_ID || "0"})`);
    try {
        const { stdout, stderr } = await exec(command, { env: extension.env, timeout: 30000 });
        if (stdout.trim()) { extension.outputChannel.appendLine(stdout.trim()); }
        if (stderr.trim()) { extension.outputChannel.appendLine(stderr.trim()); }
        extension.outputChannel.appendLine(`Daemon ${action} command completed`);
    } catch (error) {
        extension.outputChannel.appendLine(`Daemon ${action} failed: ${error.message}`);
        throw error;
    }
}

/**
 * start the ROS2 daemon.
 */
export async function startDaemon() {
    await runDaemonCommand("start");
}

/**
 * stop the ROS2 daemon.
 */
export async function stopDaemon() {
    await runDaemonCommand("stop");
}

/**
 * Shows the ROS core status in the status bar.
 */
export class StatusBarItem {
    private item: vscode.StatusBarItem;
    private timeout: NodeJS.Timeout;
    private ros2cli: ros2_monitor.XmlRpcApi;
    private disposed = false;

    public constructor() {
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);

        const waitIcon = "$(clock)";
        const ros = "ROS";
        this.item.text = `${waitIcon} ${ros}`;
        this.item.command = extension.Commands.ShowCoreStatus;
        this.ros2cli = new ros2_monitor.XmlRpcApi();
    }

    public activate() {
        if (this.disposed) { return; }
        this.item.show();
        this.timeout = setTimeout(() => this.update(), 200);
    }

    public dispose() {
        this.disposed = true;
        clearTimeout(this.timeout);
        this.item.dispose();
    }

    private async update() {
        if (this.disposed) { return; }
        let status: boolean = false;
        try {
            const result = await this.ros2cli.getNodeNamesAndNamespaces();
            status = true;
        } catch (error) {
            // Do nothing
        } finally {
            if (this.disposed) { return; }
            const statusIcon = status ? "$(check)" : "$(x)";
            let ros = "ROS";

            // these environment variables are set by the ros_environment package
            // https://github.com/ros/ros_environment
            const rosVersionChecker = "ROS_VERSION";
            const rosDistroChecker = "ROS_DISTRO";
            if (extension.env && rosVersionChecker in extension.env && rosDistroChecker in extension.env) {
                const rosVersion: string = extension.env[rosVersionChecker];
                const rosDistro: string = extension.env[rosDistroChecker];
                ros += `${rosVersion}.${rosDistro}`;
            }
            this.item.text = `${statusIcon} ${ros}`;
            this.timeout = setTimeout(() => this.update(), 200);
        }
    }
}
