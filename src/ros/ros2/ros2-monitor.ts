// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import * as xmlrpc from "xmlrpc";

import * as extension from "../../extension";
import * as telemetry from "../../telemetry-helper";
import * as rosapi from "../ros";
import * as lifecycle from "./lifecycle";
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
        async message => {
            switch (message.command) {
                case 'startDaemon':
                    startRos2Daemon(panel);
                    break;
                case 'stopDaemon':
                    stopRos2Daemon(panel);
                    break;
                case 'triggerLifecycleTransition':
                    try {
                        const { nodeName, transitionId } = message;
                        const success = await lifecycle.triggerTransition(nodeName, transitionId);
                        panel.webview.postMessage({
                            command: 'transitionResult',
                            nodeName: nodeName,
                            success: success
                        });
                        // Note: Lifecycle nodes will be refreshed automatically by the polling cycle
                    } catch (error) {
                        extension.outputChannel.appendLine(`Error triggering lifecycle transition: ${error.message}`);
                    }
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
            
            // Filter out ros2cli nodes from the regular nodes list
            const filteredNodes = result[0].filter((nodeData: any[]) => {
                const nodeName = nodeData[0]; // Node name is the first element
                return !nodeName.includes("ros2cli");
            });
            
            // Filter out topics and services associated with ros2cli nodes
            const filteredTopics = result[1].filter((topicData: any[]) => {
                const topicName = topicData[0]; // Topic name is the first element
                return !topicName.includes("ros2cli");
            });
            
            const filteredServices = result[2].filter((serviceData: any[]) => {
                const serviceName = serviceData[0]; // Service name is the first element
                return !serviceName.includes("ros2cli");
            });
            
            const nodesJSON = JSON.stringify(filteredNodes);
            const topicsJSON = JSON.stringify(filteredTopics);
            const servicesJSON = JSON.stringify(filteredServices);
            
            // Update daemon state based on successful connection
            isDaemonRunning = true;
            
            // Get lifecycle nodes information
            try {
                const lifecycleNodes = await lifecycle.getLifecycleNodes();
                const nodeInfos = await Promise.all(
                    lifecycleNodes.map(async (nodeName) => {
                        const info = await lifecycle.getNodeInfo(nodeName);
                        return info;
                    })
                );
                
                panel.webview.postMessage({
                    ready: true,
                    nodes: nodesJSON,
                    topics: topicsJSON,
                    services: servicesJSON,
                    isDaemonRunning: true,
                    lifecycleNodes: nodeInfos.filter(info => info !== null)
                });
            } catch (lifecycleError) {
                // If lifecycle nodes fail to load, still send other data
                extension.outputChannel.appendLine(`Warning: Could not get lifecycle nodes: ${lifecycleError.message}`);
                panel.webview.postMessage({
                    ready: true,
                    nodes: nodesJSON,
                    topics: topicsJSON,
                    services: servicesJSON,
                    isDaemonRunning: true,
                    lifecycleNodes: []
                });
            }
        } catch (e) {
            // Update daemon state based on failed connection
            isDaemonRunning = false;
            
            panel.webview.postMessage({
                ready: false,
                isDaemonRunning: false,
                lifecycleNodes: []
            });
        }
    }, 1000);

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
        .section {
            margin: 20px 0;
        }
        .section h3 {
            color: #cccccc;
            border-bottom: 1px solid #444;
            padding-bottom: 5px;
        }
        .lifecycle-node {
            background-color: #2d2d30;
            margin: 10px 0;
            padding: 15px;
            border-radius: 4px;
            border: 1px solid #444;
        }
        .lifecycle-node-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }
        .node-name {
            font-weight: bold;
            color: #ffffff;
        }
        .node-state {
            padding: 4px 8px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
        }
        .state-unconfigured { background-color: #6c757d; color: white; }
        .state-inactive { background-color: #ffc107; color: black; }
        .state-active { background-color: #28a745; color: white; }
        .state-finalized { background-color: #dc3545; color: white; }
        .transitions {
            display: flex;
            gap: 5px;
            flex-wrap: wrap;
        }
        .transition-btn {
            background-color: #0e639c;
            color: white;
            border: none;
            padding: 4px 8px;
            border-radius: 3px;
            cursor: pointer;
            font-size: 12px;
        }
        .transition-btn:hover {
            background-color: #1177bb;
        }
    </style>
</head>

<body>
    <div class="menu-bar">
        <button id="daemon-toggle-btn" class="menu-button">Start Daemon</button>
        <span id="daemon-status-message" class="status-message"></span>
    </div>
    
    <div class="section">
        <h3>Lifecycle Nodes</h3>
        <div id="lifecycle-nodes">
            <p>Loading lifecycle nodes...</p>
        </div>
    </div>
    
    <div class="section">
        <h3>System Information</h3>
        <div id="parameters"></div>
        <div id="topics"></div>
        <div id="services"></div>
    </div>
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
