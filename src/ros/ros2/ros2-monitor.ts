// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import * as xmlrpc from "xmlrpc";

import * as extension from "../../extension";
import * as telemetry from "../../telemetry-helper";
import * as rosapi from "../ros";
// Global variable to track the existing panel
let existingPanel: vscode.WebviewPanel | undefined;
let isDaemonRunning: boolean = false;

function getDaemonPort() {
    let basePort: number = 11511;
    basePort += Number(process.env.ROS_DOMAIN_ID) || 0
    return basePort;
}

function getDaemonUri() {
    return `http://localhost:${getDaemonPort()}/ros2cli/`;
}

async function startRos2Daemon(panel: vscode.WebviewPanel) {
    try {
        // Send starting message first
        panel.webview.postMessage({
            daemonAction: 'starting',
            message: 'Starting ROS2 daemon...',
            isRunning: false // Still not running while we're starting
        });

        // Actually start the core
        await rosapi.rosApi.startCore();
        
        // Don't update isDaemonRunning here - let the polling detect it
        
        // Send success message
        panel.webview.postMessage({
            daemonAction: 'started',
            message: 'ROS2 daemon start command completed',
            isRunning: isDaemonRunning // Use current polled state
        });
    } catch (error) {
        // If start failed, daemon is still not running
        panel.webview.postMessage({
            daemonAction: 'error',
            message: `Failed to start daemon: ${error}`,
            isRunning: isDaemonRunning // Use current polled state
        });
    }
}

async function stopRos2Daemon(panel: vscode.WebviewPanel) {
    try {
        // Send stopping message first
        panel.webview.postMessage({
            daemonAction: 'stopping',
            message: 'Stopping ROS2 daemon...',
            isRunning: true // Still running while we're stopping
        });

        // Actually stop the core
        await rosapi.rosApi.stopCore();
        
        // Don't update isDaemonRunning here - let the polling detect it
        
        // Send success message
        panel.webview.postMessage({
            daemonAction: 'stopped',
            message: 'ROS2 daemon stop command completed',
            isRunning: isDaemonRunning // Use current polled state
        });

    } catch (error) {
        // If stop failed, use current polled state
        panel.webview.postMessage({
            daemonAction: 'error',
            message: `Failed to stop daemon: ${error}`,
            isRunning: isDaemonRunning
        });
    }
}

export function launchMonitor(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.ShowCoreStatus);

    // Check if an existing panel exists and reveal it
    if (existingPanel) {
        existingPanel.reveal(vscode.ViewColumn.Two);
        return;
    }

    const panel = vscode.window.createWebviewPanel(
        "ros2Status",
        "ROS 2 Status",
        vscode.ViewColumn.Two,
        {
            enableScripts: true,
        }
    );

    // Store reference to the panel
    existingPanel = panel;

    const stylesheet = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "assets", "ros", "core-monitor", "style.css")));

    const script = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "dist", "ros2_webview_main.js")));

    panel.webview.html = getCoreStatusWebviewContent(stylesheet, script);

    // Handle messages from webview
    panel.webview.onDidReceiveMessage(
        message => {
            switch (message.command) {
                case 'startDaemon':
                    startRos2Daemon(panel);
                    break;
                case 'stopDaemon':
                    stopRos2Daemon(panel);
                    break;
            }
        }
    );

    const ros2cliApi = new XmlRpcApi();
    const pollingHandle = setInterval(async () => {
        try {
            const result: any[] = await Promise.all([
                ros2cliApi.getNodeNamesAndNamespaces(), 
                ros2cliApi.getTopicNamesAndTypes(), 
                ros2cliApi.getServiceNamesAndTypes()]);
            const nodesJSON = JSON.stringify(result[0]);
            const topicsJSON = JSON.stringify(result[1]);
            const servicesJSON = JSON.stringify(result[2]);
            
            // Update daemon state based on successful connection
            isDaemonRunning = true;
            
            panel.webview.postMessage({
                ready: true,
                nodes: nodesJSON,
                topics: topicsJSON,
                services: servicesJSON,
                isDaemonRunning: true
            });
        } catch (e) {
            // Update daemon state based on failed connection
            isDaemonRunning = false;
            
            panel.webview.postMessage({
                ready: false,
                isDaemonRunning: false
            });
        }
    }, 200);

    panel.onDidDispose(() => {
        clearInterval(pollingHandle);
        existingPanel = undefined; // Clear the reference when panel is disposed
    });
}

function getCoreStatusWebviewContent(stylesheet: vscode.Uri, script: vscode.Uri): string {
    return `
<!DOCTYPE html>
<html lang="en">

<head>
    <link rel="stylesheet" href="${stylesheet.toString()}" />
    <script src="${script.toString()}"></script>
    <style>
        .menu-bar {
            background-color: #2d2d30;
            padding: 10px;
            margin-bottom: 20px;
            border-radius: 4px;
            display: flex;
            gap: 10px;
            align-items: center;
        }
        .menu-button {
            background-color: #0e639c;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
        }
        .menu-button:hover {
            background-color: #1177bb;
        }
        .menu-button:disabled {
            background-color: #666;
            cursor: not-allowed;
        }
        .menu-button.stop {
            background-color: #d73a49;
        }
        .menu-button.stop:hover {
            background-color: #e85a67;
        }
        .status-message {
            margin-left: 15px;
            font-style: italic;
            color: #cccccc;
        }
    </style>
</head>

<body>
    <div class="menu-bar">
        <button id="daemon-toggle-btn" class="menu-button">Start Daemon</button>
        <span id="daemon-status-message" class="status-message"></span>
    </div>
    <div id="parameters"></div>
    <div id="topics"></div>
    <div id="services"></div>
</body>

</html>
`;
}

/**
 * ros2cli xmlrpc interfaces.
 */
export class XmlRpcApi {
    private client: xmlrpc.Client;

    public constructor() {
        this.client = xmlrpc.createClient(getDaemonUri());
    }

    public check() : Promise<boolean> {
        // the ROS2 CLI doesn't have an API which returns detailed status, 
        // so we're just using another endpoint to verify it is running
        return this.methodCall("get_node_names_and_namespaces").then(() => true, () => false);
    }

    public getNodeNamesAndNamespaces() : Promise<any> {
        return this.methodCall("get_node_names_and_namespaces");
    }

    public getServiceNamesAndTypes() : Promise<any> {
        return this.methodCall("get_service_names_and_types");
    }

    public getTopicNamesAndTypes() : Promise<any> {
        return this.methodCall("get_topic_names_and_types");
    }

    private methodCall(method: string, ...args: any[]): Promise<any> {
        return new Promise((resolve, reject) => {
            this.client.methodCall(method, [...args], (err, val) => {
                if (err) {
                    reject(err);
                } else {
                    resolve(val);
                }
            });
        });
    }
}
