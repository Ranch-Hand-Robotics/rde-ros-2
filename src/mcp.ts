// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as http from "http";
import * as vscode from "vscode";

import * as vscode_utils from "./vscode-utils";
import * as ros_utils from "./ros/utils";

// Import from extension to access shared globals
import { 
    outputChannel, 
    extPath, 
    env, 
    extensionContext, 
    subscriptions 
} from "./extension";

/**
 * The MCP server terminal
 */
export let mcpServerTerminal: vscode.Terminal | null = null;
let mcpServerDefinitionProvider: vscode.Disposable | null = null;
let activeMcpServerPort: number | null = null;

const kDefaultMcpStartupTimeoutMs = 15000;
const kDefaultMcpPollIntervalMs = 250;
const kDefaultMcpProbeTimeoutMs = 1000;

function delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function supportsMcpRegistration(): boolean {
    return ("lm" in vscode && vscode.lm && "registerMcpServerDefinitionProvider" in vscode.lm);
}

function disposeMcpServerDefinitionProvider(): void {
    if (mcpServerDefinitionProvider) {
        mcpServerDefinitionProvider.dispose();
        mcpServerDefinitionProvider = null;
    }
}

export async function isMcpServerReady(port: number, probeTimeoutMs: number = kDefaultMcpProbeTimeoutMs): Promise<boolean> {
    return new Promise<boolean>((resolve) => {
        let settled = false;
        const finish = (result: boolean): void => {
            if (!settled) {
                settled = true;
                resolve(result);
            }
        };

        const request = http.request({
            host: "127.0.0.1",
            port,
            path: "/sse",
            method: "GET",
            timeout: probeTimeoutMs,
        }, (response) => {
            const statusCode = response.statusCode ?? 0;
            response.resume();
            finish(statusCode >= 200 && statusCode < 300);
            request.destroy();
        });

        request.on("timeout", () => {
            request.destroy();
            finish(false);
        });

        request.on("error", () => {
            finish(false);
        });

        request.end();
    });
}

export async function waitForMcpServerReady(
    port: number,
    startupTimeoutMs: number = kDefaultMcpStartupTimeoutMs,
    pollIntervalMs: number = kDefaultMcpPollIntervalMs,
    probeTimeoutMs: number = kDefaultMcpProbeTimeoutMs,
): Promise<boolean> {
    const deadline = Date.now() + startupTimeoutMs;

    while (Date.now() < deadline) {
        if (await isMcpServerReady(port, probeTimeoutMs)) {
            return true;
        }

        if (mcpServerTerminal?.exitStatus) {
            return false;
        }

        await delay(pollIntervalMs);
    }

    return false;
}

function registerMcpServerDefinitionProvider(mcpServerPort: number): void {
    try {
        disposeMcpServerDefinitionProvider();

        // Use type assertion to handle the API that might not be available in all environments
        const lm = vscode.lm as any;
        mcpServerDefinitionProvider = lm.registerMcpServerDefinitionProvider("ROS 2", {
            provideMcpServerDefinitions: async () => {
                const output: any[] = [];

                // Use the discovered port for the MCP server
                // Note: McpHttpServerDefinition might not be available in all environments
                if ("McpHttpServerDefinition" in vscode) {
                    const McpHttpServerDefinition = (vscode as any).McpHttpServerDefinition;
                    output.push(
                        new McpHttpServerDefinition(
                            "ROS 2",
                            vscode.Uri.parse(`http://localhost:${mcpServerPort}/sse`),
                        ),
                    );
                }

                return output;
            }
        });

        outputChannel.appendLine(`Registered MCP server with VS Code on port ${mcpServerPort}`);
    } catch (error) {
        outputChannel.appendLine(`Failed to register MCP server definition provider: ${error.message}`);
    }
}

/**
 * Shuts down the MCP server if it's currently running.
 */
export function shutdownMcpServer(): void {
    if (mcpServerTerminal) {
        outputChannel.appendLine("Shutting down MCP server");
        mcpServerTerminal.dispose();
        mcpServerTerminal = null;
    }

    disposeMcpServerDefinitionProvider();
    activeMcpServerPort = null;
}

/**
 * Starts the MCP server with proper setup and dependency management.
 * @param context The VS Code extension context
 */
export async function startMcpServer(context: vscode.ExtensionContext): Promise<void> {
    // MCP server is already running
    if (mcpServerTerminal && !mcpServerTerminal.exitStatus) {
        outputChannel.appendLine("MCP server is already running");

        if (supportsMcpRegistration() && !mcpServerDefinitionProvider && activeMcpServerPort !== null) {
            outputChannel.appendLine("MCP server appears to be running, waiting for readiness before registering with VS Code.");
            const ready = await waitForMcpServerReady(activeMcpServerPort);
            if (ready) {
                registerMcpServerDefinitionProvider(activeMcpServerPort);
            }
        }

        return;
    }

    // Get or create the MCP terminal for setup operations
    const mcpTerminal = getMcpTerminal();
    outputChannel.appendLine("Using MCP server terminal for setup operations");

    // Ensure we have a proper MCP virtual environment
    const canProceed = await vscode_utils.ensureMcpVirtualEnvironment(context, outputChannel, extPath);
    if (!canProceed) {
        outputChannel.appendLine("Virtual environment for MCP server is not ready.");
        vscode.window.showInformationMessage("Virtual environment for MCP server is not ready. Please check the MCP Server terminal and output channel for details.");
        showMcpServerTerminal();
        return;
    }

    try {
        const mcpServerPort = await vscode_utils.findAvailablePort(3002);
        const serverPath = path.join(extPath, "assets", "scripts", "server.py");
        
        // Show MCP server information for users without MCP support
        let canRegisterMcp = supportsMcpRegistration();

        if (!canRegisterMcp) {
            const infoMessage = `ROS 2 MCP Server starting on port ${mcpServerPort}.\n\nTo use MCP features in Cursor, add this server to your .cursor/mcp.json:\n\nServer URL: http://localhost:${mcpServerPort}/sse`;

            vscode.window.showInformationMessage(infoMessage, "Copy URL", "Open MCP Config", "Dismiss").then(selection => {
                if (selection === "Copy URL") {
                    vscode.env.clipboard.writeText(`http://localhost:${mcpServerPort}/sse`);
                    vscode.window.showInformationMessage("MCP Server URL copied to clipboard!");
                } else if (selection === "Open MCP Config") {
                    // Open the .cursor/mcp.json file if it exists, otherwise create it
                    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
                    if (workspaceRoot) {
                        const mcpConfigPath = path.join(workspaceRoot, '.cursor', 'mcp.json');
                        vscode.workspace.openTextDocument(mcpConfigPath).then(doc => {
                            vscode.window.showTextDocument(doc);
                        }, () => {
                            // File doesn't exist, create it with basic structure
                            const basicConfig = {
                                "mcpServers": {
                                    "ros2": {
                                        "url": `http://localhost:${mcpServerPort}/sse`
                                    }
                                }
                            };
                            vscode.workspace.fs.writeFile(
                                vscode.Uri.file(mcpConfigPath),
                                Buffer.from(JSON.stringify(basicConfig, null, 2))
                            ).then(() => {
                                vscode.workspace.openTextDocument(mcpConfigPath).then(doc => {
                                    vscode.window.showTextDocument(doc);
                                }, () => {
                                    // Handle any errors silently
                                });
                            });
                        });
                    }
                }
            });
        }
        
        if (await exists(serverPath)) {
            outputChannel.appendLine(`Starting MCP server from ${serverPath} on port ${mcpServerPort}`);

            
            const venvPath = path.join(extPath, ".venv");
            const pythonExecutable = process.platform === "win32" 
                ? path.join(venvPath, "Scripts", "python3.exe")
                : path.join(venvPath, "bin", "python3");

            if (process.platform === "win32") {
                mcpTerminal.sendText(`${path.join(venvPath, 'Scripts', 'activate.bat')}`);
            } else {
                const shellInfo = ros_utils.detectUserShell();
                const activateScript = path.join(venvPath, 'bin', 'activate');
                mcpTerminal.sendText(`${shellInfo.sourceCommand} ${activateScript}`);
            }
            mcpTerminal.sendText(`${pythonExecutable} ${serverPath} --port ${mcpServerPort}`);
            activeMcpServerPort = mcpServerPort;

            // Add to subscriptions to ensure it's terminated on environment change
            subscriptions.push({
                dispose: () => {
                    shutdownMcpServer();
                }
            });
        } else {
            throw new Error(`MCP server script not found at ${serverPath}`);
        }


        // Register MCP server definition provider only after startup is confirmed.
        if (canRegisterMcp) {
            outputChannel.appendLine(`Waiting for MCP server startup on port ${mcpServerPort} before registering with VS Code...`);
            const ready = await waitForMcpServerReady(mcpServerPort);

            if (ready) {
                registerMcpServerDefinitionProvider(mcpServerPort);
            } else {
                outputChannel.appendLine(`MCP server did not become ready within ${kDefaultMcpStartupTimeoutMs}ms. Skipping VS Code MCP registration.`);
                vscode.window.showWarningMessage("ROS 2 MCP server did not become ready in time. It was not registered with VS Code.");
            }
        }

    } catch (err) {
        activeMcpServerPort = null;
        outputChannel.appendLine(`Failed to start MCP server: ${err.message}`);
        vscode.window.showErrorMessage(`Failed to start MCP server: ${err.message}`);

        return;
    }
}

/**
 * Shows the MCP server terminal.
 */
export function showMcpServerTerminal(): void {
    if (mcpServerTerminal && !mcpServerTerminal.exitStatus) {
        mcpServerTerminal.show();
    }
}

/**
 * Get or create the MCP terminal.
 */
export function getMcpTerminal(): vscode.Terminal {
    if (mcpServerTerminal) {
        return mcpServerTerminal;
    }

    // Check if there's already a terminal with the same name
    const existingTerminal = vscode.window.terminals.find(terminal => terminal.name === 'ROS 2 MCP Server');
    if (existingTerminal) {
        mcpServerTerminal = existingTerminal;
        return mcpServerTerminal;
    }

    mcpServerTerminal = vscode.window.createTerminal({
        name: 'ROS 2 MCP Server',
        env: env
    });
    
    // Clean up terminal reference when closed
    const disposable = vscode.window.onDidCloseTerminal((closedTerminal) => {
        if (closedTerminal === mcpServerTerminal) {
            mcpServerTerminal = null;
            activeMcpServerPort = null;
            disposeMcpServerDefinitionProvider();
            disposable.dispose();
        }
    });
    
    extensionContext.subscriptions.push(disposable);
    
    return mcpServerTerminal;
}

/**
 * Check if a file or directory exists.
 */
async function exists(filePath: string): Promise<boolean> {
    try {
        const { promises: fsPromises } = await import("fs");
        await fsPromises.access(filePath);
        return true;
    } catch {
        return false;
    }
}

/**
 * Commands enum for MCP server
 */
export enum McpCommands {
    StartMcpServer = "ROS2.startMcpServer",
    StopMcpServer = "ROS2.stopMcpServer",
    ShowMcpTerminal = "ROS2.showMcpTerminal",
}

/**
 * Helper function to wrap command execution with error handling
 */
function ensureErrorMessageOnException(callback: (...args: any[]) => any) {
    try {
        return callback();
    } catch (err) {
        vscode.window.showErrorMessage(err.message);
    }
}

/**
 * Register all MCP-related commands.
 * @param context The VS Code extension context
 */
export function registerMcpCommands(context: vscode.ExtensionContext): void {
    // Register MCP server commands
    context.subscriptions.push(
        vscode.commands.registerCommand(McpCommands.StartMcpServer, () => {
            ensureErrorMessageOnException(() => {
                return startMcpServer(context);
            });
        })
    );

    context.subscriptions.push(
        vscode.commands.registerCommand(McpCommands.StopMcpServer, () => {
            ensureErrorMessageOnException(() => {
                shutdownMcpServer();
                vscode.window.showInformationMessage("MCP server stopped");
            });
        })
    );

    context.subscriptions.push(
        vscode.commands.registerCommand(McpCommands.ShowMcpTerminal, () => {
            ensureErrorMessageOnException(() => {
                showMcpServerTerminal();
            });
        })
    );
}
