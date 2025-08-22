// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as fs from "fs";
import * as vscode from "vscode";
import * as child_process from "child_process";

import * as cpp_formatter from "./cpp-formatter";
import * as pfs from "./promise-fs";
import * as telemetry from "./telemetry-helper";
import * as vscode_utils from "./vscode-utils";

import * as buildtool from "./build-tool/build-tool";

import * as ros_build_utils from "./ros/build-env-utils";
import * as ros_cli from "./ros/cli";
import * as ros_utils from "./ros/utils";
import { rosApi, selectROSApi } from "./ros/ros";
import * as lifecycle from "./ros/ros2/lifecycle";

import * as debug_manager from "./debugger/manager";
import * as debug_utils from "./debugger/utils";
import { registerRosShellTaskProvider } from "./build-tool/ros-shell";

/**
 * The sourced ROS environment.
 */
export let env: any;
export let processingWorkspace = false;

export let extPath: string;
export let outputChannel: vscode.OutputChannel;
export let mcpServerTerminal: vscode.Terminal | null = null;
export let extensionContext: vscode.ExtensionContext | null = null;

let onEnvChanged = new vscode.EventEmitter<void>();

/**
 * Triggered when the env is soured.
 */
export let onDidChangeEnv = onEnvChanged.event;

export async function resolvedEnv() {
    if (env === undefined) { // Env reload in progress
        await debug_utils.oneTimePromiseFromEvent(onDidChangeEnv, () => env !== undefined);
    }
    return env
}

/**
 * Subscriptions to dispose when the environment is changed.
 */
let subscriptions = <vscode.Disposable[]>[];

export enum Commands {
    CreateTerminal = "ROS2.createTerminal",
    GetDebugSettings = "ROS2.getDebugSettings",
    Rosrun = "ROS2.rosrun",
    Roslaunch = "ROS2.roslaunch",
    Rostest = "ROS2.rostest",
    Rosdep = "ROS2.rosdep",
    ShowCoreStatus = "ROS2.showCoreStatus",
    StartRosCore = "ROS2.startCore",
    TerminateRosCore = "ROS2.stopCore",
    UpdateCppProperties = "ROS2.updateCppProperties",
    UpdatePythonPath = "ROS2.updatePythonPath",
    PreviewURDF = "ROS2.previewUrdf",
    Doctor = "ROS2.doctor",
    StartMcpServer = "ROS2.startMcpServer",
    StopMcpServer = "ROS2.stopMcpServer",
    ShowMcpTerminal = "ROS2.showMcpTerminal",
    LifecycleListNodes = "ROS2.lifecycle.listNodes",
    LifecycleGetState = "ROS2.lifecycle.getState",
    LifecycleSetState = "ROS2.lifecycle.setState",
    LifecycleTriggerTransition = "ROS2.lifecycle.triggerTransition"
}

/**
 * Shuts down the MCP server if it's currently running.
 */
function shutdownMcpServer(): void {
    if (mcpServerTerminal) {
        outputChannel.appendLine("Shutting down MCP server");
        mcpServerTerminal.dispose();
        mcpServerTerminal = null;
    }
}

/**
 * Starts the MCP server with proper setup and dependency management.
 * @param context The VS Code extension context
 */
async function startMcpServer(context: vscode.ExtensionContext): Promise<void> {
    // MCP server is already running
    if (mcpServerTerminal && !mcpServerTerminal.exitStatus) {
        outputChannel.appendLine("MCP server is already running");
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
        const mcpServerPort = vscode_utils.getExtensionConfiguration().get<number>("mcpServerPort", 3002);
        const serverPath = path.join(extPath, "assets", "scripts", "server.py");
        
        // Show MCP server information for users without MCP support
        let supportsMcpRegistration = ('lm' in vscode && vscode.lm && 'registerMcpServerDefinitionProvider' in vscode.lm);

        if (!supportsMcpRegistration) {
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
                                        "command": "npx",
                                        "args": ["-y", "@modelcontextprotocol/server-ros2", "stdio"],
                                        "env": {}
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
        
        if (await pfs.exists(serverPath)) {
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

            // Add to subscriptions to ensure it's terminated on environment change
            subscriptions.push({
                dispose: () => {
                    shutdownMcpServer();
                }
            });
        } else {
            throw new Error(`MCP server script not found at ${serverPath}`);
        }


        // Register MCP server definition provider (only when LLDB extension is not available)
        if (supportsMcpRegistration) {
            try {
                // Use type assertion to handle the API that might not be available in all environments
                const lm = vscode.lm as any;
                context.subscriptions.push(lm.registerMcpServerDefinitionProvider('ROS 2', {
                    provideMcpServerDefinitions: async () => {
                        let output: any[] = [];

                        // Get the port from configuration or use default
                        const mcpServerPort = vscode_utils.getExtensionConfiguration().get<number>("mcpServerPort", 3002);

                        // Use the configured port for the MCP server
                        // Note: McpHttpServerDefinition might not be available in all environments
                        if ('McpHttpServerDefinition' in vscode) {
                            const McpHttpServerDefinition = (vscode as any).McpHttpServerDefinition;
                            output.push( 
                                new McpHttpServerDefinition(
                                    "ROS 2",
                                    vscode.Uri.parse(`http://localhost:${mcpServerPort}/sse`)
                                )
                            );
                        }

                        return output;
                    }
                }));
            } catch (error) {
                outputChannel.appendLine(`Failed to register MCP server definition provider: ${error.message}`);
            }
        }

    } catch (err) {
        outputChannel.appendLine(`Failed to start MCP server: ${err.message}`);
        vscode.window.showErrorMessage(`Failed to start MCP server: ${err.message}`);

        return;
    }

    
}

export async function activate(context: vscode.ExtensionContext) {
    try {
        const reporter = telemetry.getReporter();
        extPath = context.extensionPath;
        outputChannel = vscode_utils.createOutputChannel();
        extensionContext = context; // Store the context for later use
        context.subscriptions.push(outputChannel);

        // Log extension activation
        outputChannel.appendLine("ROS 2 Extension activating...");
        
    } catch (error) {
        console.error("Error during extension activation:", error);
        throw error;
    }

    // Detect if LLDB extension is installed
    const isLldbInstalled = vscode_utils.isLldbExtensionInstalled();
    if (isLldbInstalled) {
        outputChannel.appendLine("LLDB extension is installed - C++ debugging with LLDB available");
    } else {
        outputChannel.appendLine("LLDB extension not installed - C++ debugging with LLDB not available");
    }

    // Activate components when the ROS env is changed.
    context.subscriptions.push(onDidChangeEnv(activateEnvironment.bind(null, context)));

    // Activate components which don't require the ROS env.
    context.subscriptions.push(vscode.languages.registerDocumentFormattingEditProvider(
        "cpp", new cpp_formatter.CppFormatter()
    ));

    // Source the environment, and re-source on config change.
    let config = vscode_utils.getExtensionConfiguration();

    context.subscriptions.push(vscode.workspace.onDidChangeConfiguration(() => {
        const updatedConfig = vscode_utils.getExtensionConfiguration();
        const fields = Object.keys(config).filter(k => !(config[k] instanceof Function));
        const changed = fields.some(key => updatedConfig[key] !== config[key]);

        if (changed) {
            sourceRosAndWorkspace();
        }

        config = updatedConfig;
    }));

    vscode.commands.registerCommand(Commands.CreateTerminal, () => {
        ensureErrorMessageOnException(() => {
            ros_utils.createTerminal(context);
        });
    });

    vscode.commands.registerCommand(Commands.GetDebugSettings, () => {
        ensureErrorMessageOnException(() => {
            return debug_utils.getDebugSettings(context);
        });
    });

    vscode.commands.registerCommand(Commands.ShowCoreStatus, () => {
        ensureErrorMessageOnException(() => {
            rosApi.showCoreMonitor();
        });
    });

    vscode.commands.registerCommand(Commands.StartRosCore, () => {
        ensureErrorMessageOnException(() => {
            rosApi.startCore();
        });
    });

    vscode.commands.registerCommand(Commands.TerminateRosCore, () => {
        ensureErrorMessageOnException(() => {
            rosApi.stopCore();
        });
    });

    vscode.commands.registerCommand(Commands.UpdateCppProperties, () => {
        ensureErrorMessageOnException(() => {
            return ros_build_utils.updateCppProperties(context);
        });
    });

    vscode.commands.registerCommand(Commands.UpdatePythonPath, () => {
        ensureErrorMessageOnException(() => {
            ros_build_utils.updatePythonPath(context);
        });
    });

    vscode.commands.registerCommand(Commands.Rosrun, () => {
        ensureErrorMessageOnException(() => {
            return ros_cli.rosrun(context);
        });
    });

    vscode.commands.registerCommand(Commands.Roslaunch, () => {
        ensureErrorMessageOnException(() => {
            return ros_cli.roslaunch(context);
        });
    });

    vscode.commands.registerCommand(Commands.Rostest, () => {
        ensureErrorMessageOnException(() => {
            return ros_cli.rostest(context);
        });
    });

    vscode.commands.registerCommand(Commands.Rosdep, () => {
        ensureErrorMessageOnException(() => {
            rosApi.rosdep();
        });
    });

    vscode.commands.registerCommand(Commands.Doctor, () => {
        ensureErrorMessageOnException(() => {
            rosApi.doctor();
        });
    });

    // Register MCP server commands
    vscode.commands.registerCommand(Commands.StartMcpServer, () => {
        ensureErrorMessageOnException(() => {
            return startMcpServer(context);
        });
    });

    vscode.commands.registerCommand(Commands.StopMcpServer, () => {
        ensureErrorMessageOnException(() => {
            shutdownMcpServer();
            vscode.window.showInformationMessage("MCP server stopped");
        });
    });

    // Register Lifecycle commands
    vscode.commands.registerCommand(Commands.LifecycleListNodes, async () => {
        ensureErrorMessageOnException(async () => {
            const nodes = await lifecycle.getLifecycleNodes();
            if (nodes.length === 0) {
                vscode.window.showInformationMessage("No lifecycle nodes found.");
                return;
            }
            
            const nodeInfos = await Promise.all(
                nodes.map(async (nodeName) => {
                    const info = await lifecycle.getNodeInfo(nodeName);
                    return info ? `${nodeName} (${info.currentState.label})` : `${nodeName} (unknown state)`;
                })
            );
            
            const selected = await vscode.window.showQuickPick(nodeInfos, {
                placeHolder: "Select a lifecycle node to view details"
            });
            
            if (selected) {
                const nodeName = selected.split(' ')[0];
                const info = await lifecycle.getNodeInfo(nodeName);
                if (info) {
                    const transitions = info.availableTransitions.map(t => t.label).join(', ');
                    vscode.window.showInformationMessage(
                        `Node: ${nodeName}\nState: ${info.currentState.label}\nAvailable transitions: ${transitions}`
                    );
                }
            }
        });
    });

    vscode.commands.registerCommand(Commands.LifecycleGetState, async () => {
        ensureErrorMessageOnException(async () => {
            const nodes = await lifecycle.getLifecycleNodes();
            if (nodes.length === 0) {
                vscode.window.showInformationMessage("No lifecycle nodes found.");
                return;
            }
            
            const selected = await vscode.window.showQuickPick(nodes, {
                placeHolder: "Select a lifecycle node to get its state"
            });
            
            if (selected) {
                const state = await lifecycle.getNodeState(selected);
                if (state) {
                    vscode.window.showInformationMessage(`Node ${selected} is in state: ${state.label}`);
                } else {
                    vscode.window.showErrorMessage(`Could not get state for node: ${selected}`);
                }
            }
        });
    });

    vscode.commands.registerCommand(Commands.LifecycleSetState, async () => {
        ensureErrorMessageOnException(async () => {
            const nodes = await lifecycle.getLifecycleNodes();
            if (nodes.length === 0) {
                vscode.window.showInformationMessage("No lifecycle nodes found.");
                return;
            }
            
            const selectedNode = await vscode.window.showQuickPick(nodes, {
                placeHolder: "Select a lifecycle node"
            });
            
            if (selectedNode) {
                const states = Object.values(lifecycle.LIFECYCLE_STATES).map(s => s.label);
                const selectedState = await vscode.window.showQuickPick(states, {
                    placeHolder: "Select target state"
                });
                
                if (selectedState) {
                    const success = await lifecycle.setNodeToState(selectedNode, selectedState);
                    if (success) {
                        vscode.window.showInformationMessage(`Successfully set ${selectedNode} to ${selectedState} state`);
                    }
                }
            }
        });
    });

    vscode.commands.registerCommand(Commands.LifecycleTriggerTransition, async () => {
        ensureErrorMessageOnException(async () => {
            const nodes = await lifecycle.getLifecycleNodes();
            if (nodes.length === 0) {
                vscode.window.showInformationMessage("No lifecycle nodes found.");
                return;
            }
            
            const selectedNode = await vscode.window.showQuickPick(nodes, {
                placeHolder: "Select a lifecycle node"
            });
            
            if (selectedNode) {
                const availableTransitions = await lifecycle.getAvailableTransitions(selectedNode);
                if (availableTransitions.length === 0) {
                    vscode.window.showInformationMessage(`No transitions available for node ${selectedNode}`);
                    return;
                }
                
                const transitionLabels = availableTransitions.map(t => t.label);
                const selectedTransition = await vscode.window.showQuickPick(transitionLabels, {
                    placeHolder: "Select a transition to trigger"
                });
                
                if (selectedTransition) {
                    const success = await lifecycle.triggerTransitionByLabel(selectedNode, selectedTransition);
                    if (success) {
                        vscode.window.showInformationMessage(`Successfully triggered ${selectedTransition} on ${selectedNode}`);
                    }
                }
            }
        });
    });


    const reporter = telemetry.getReporter();
    reporter.sendTelemetryActivate();

    // Activate the workspace environment if possible.
    await activateEnvironment(context);

    return {
        getEnv: () => env,
        onDidChangeEnv: (listener: () => any, thisArg: any) => onDidChangeEnv(listener, thisArg),
    };
}

export async function deactivate() {
    subscriptions.forEach(disposable => disposable.dispose());
    await telemetry.clearReporter();
    shutdownMcpServer();
    
    // Clean up MCP terminal
    if (mcpServerTerminal && !mcpServerTerminal.exitStatus) {
        mcpServerTerminal.dispose();
        mcpServerTerminal = null;
    }
}

async function ensureErrorMessageOnException(callback: (...args: any[]) => any) {
    try {
        await callback();
    } catch (err) {
        vscode.window.showErrorMessage(err.message);
    }
}

/**
 * Activates components which require a ROS env.
 */
async function activateEnvironment(context: vscode.ExtensionContext) {

    if (processingWorkspace) {
        return;
    }

    processingWorkspace = true;

    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop().dispose();
    }

    await sourceRosAndWorkspace();

    if (typeof env.ROS_DISTRO === "undefined") {
        processingWorkspace = false;
        return;
    }

    if (typeof env.ROS_VERSION === "undefined") {
        processingWorkspace = false;
        return;
    }

    outputChannel.appendLine(`Determining build tool for workspace: ${vscode.workspace.rootPath}`);

    // Determine if we're in a ROS workspace.
    let buildToolDetected = await buildtool.determineBuildTool(vscode.workspace.rootPath);

    // http://www.ros.org/reps/rep-0149.html#environment-variables
    // Learn more about ROS_VERSION definition.
    selectROSApi(env.ROS_VERSION);

    // Do this again, after the build tool has been determined.
    await sourceRosAndWorkspace();

    rosApi.setContext(context, env);

    subscriptions.push(rosApi.activateCoreMonitor());
    if (buildToolDetected) {
        subscriptions.push(...buildtool.BuildTool.registerTaskProvider());
    } else {
        outputChannel.appendLine(`Build tool NOT detected`);

    }
    subscriptions.push(...registerRosShellTaskProvider());

    debug_manager.registerRosDebugManager(context);

    // Register commands dependent on a workspace
    if (buildToolDetected) {
        subscriptions.push(
            vscode.tasks.onDidEndTask((event: vscode.TaskEndEvent) => {
                if (buildtool.isROSBuildTask(event.execution.task)) {
                    sourceRosAndWorkspace();
                }
            }),
        );
    }

    // Generate config files if they don't already exist, but only for workspaces
    if (buildToolDetected) {
        ros_build_utils.createConfigFiles();
    }

    processingWorkspace = false;
}

/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
async function sourceRosAndWorkspace(): Promise<void> {

    // Processing a new environment can take time which introduces a race condition. 
    // Wait to atomicly switch by composing a new environment block then switching at the end.
    let newEnv = undefined;

    outputChannel.appendLine("Sourcing ROS and Workspace");

    const kWorkspaceConfigTimeout = 30000; // ms

    const config = vscode_utils.getExtensionConfiguration();

    let rosSetupScript = config.get("rosSetupScript", "");

    // If the workspace setup script is not set, try to find the ROS setup script in the environment
    let attemptWorkspaceDiscovery = true;

    // On Windows, ROS now uses Pixi to install; with a specific location for the setup script.
    if (process.platform === "win32" && !rosSetupScript) {
        rosSetupScript = path.join("C:", "pixi_ws", "ros2-windows", "local_setup.bat");
    }

    if (rosSetupScript) {
        // Regular expression to match '${workspaceFolder}'
        const regex = "\$\{workspaceFolder\}";
        if (rosSetupScript.includes(regex)) {
            if (vscode.workspace.workspaceFolders.length === 1) {
                // Replace all occurrences of '${workspaceFolder}' with the workspace string
                rosSetupScript = rosSetupScript.replace(regex, vscode.workspace.workspaceFolders[0].uri.fsPath);
            } else {
                outputChannel.appendLine(`Multiple or no workspaces found, but the ROS setup script setting \"ROS2.rosSetupScript\" is configured with '${rosSetupScript}'`);
            }
        }

        // Try to support cases where the setup script doesn't make sense on different environments, such as host vs container.
        if (await pfs.exists(rosSetupScript)) {
            try {
                newEnv = await ros_utils.sourceSetupFile(rosSetupScript, newEnv);

                outputChannel.appendLine(`Sourced ${rosSetupScript}`);

                attemptWorkspaceDiscovery = false;
            } catch (err) {
                await vscode.window.setStatusBarMessage(`A ROS setup script was provided, but could not source "${rosSetupScript}". Attempting standard discovery.`);
            }
        }
    }

    if (attemptWorkspaceDiscovery) {
        let distro = config.get("distro", "");

        // Is there a distro defined either by setting or environment?
        outputChannel.appendLine(`Current ROS_DISTRO environment variable: ${process.env.ROS_DISTRO}`);
        if (!distro) {
            // No? Try to find one.
            const installedDistros = await ros_utils.getDistros();
            if (!installedDistros.length) {
                outputChannel.appendLine(`No distros found.`);

                throw new Error("ROS has not been found on this system.");
            } else if (installedDistros.length === 1) {
                outputChannel.appendLine(`Only one distro, selecting ${installedDistros[0]}`);

                // if there is only one distro installed, directly choose it
                config.update("distro", installedDistros[0]);
                distro = installedDistros[0];
            } else {
                outputChannel.appendLine(`Multiple distros found, prompting user to select one.`);
                // dump installedDistros to outputChannel
                outputChannel.appendLine(`Installed distros: ${installedDistros}`);

                const message = "Unable to determine ROS distribution, please configure this workspace by adding \"ROS2.distro\": \"<ROS Distro>\" in settings.json";
                await vscode.window.setStatusBarMessage(message, kWorkspaceConfigTimeout);
            }
        }

        if (process.env.ROS_DISTRO && process.env.ROS_DISTRO !== distro) {
            outputChannel.appendLine(`ROS_DISTRO environment variable (${process.env.ROS_DISTRO}) does not match configured distro (${distro}).`);

            outputChannel.appendLine(`Overriding the configured distro with the environment variable.`);

            distro = process.env.ROS_DISTRO;
        }

        if (distro) {
            let setupScript: string;
            try {
                let globalInstallPath: string;
                if (process.platform === "win32") {
                    globalInstallPath = path.join("C:", "opt", "ros", `${distro}`, "x64");
                } else {
                    globalInstallPath = path.join("/", "opt", "ros", `${distro}`);
                }
                setupScript = path.format({
                    dir: globalInstallPath,
                    name: "setup",
                    ext: ros_utils.getSetupScriptExtension(),
                });

                outputChannel.appendLine(`Sourcing ROS Distro: ${setupScript}`);
                newEnv = await ros_utils.sourceSetupFile(setupScript, newEnv);
            } catch (err) {
                await vscode.window.setStatusBarMessage(`Could not source ROS setup script at "${setupScript}".`);
            }
        } else if (process.env.ROS_DISTRO) {
            newEnv = process.env;
        }
    }

    let workspaceOverlayPath: string = "";
    // Source the workspace setup over the top.

    if (newEnv.ROS_VERSION === "1") {
        outputChannel.appendLine(`this extension does not support ROS 1`);
    } else {    // FUTURE: Revisit if ROS_VERSION changes - not clear it will be called 3
        if (!await pfs.exists(workspaceOverlayPath)) {
            workspaceOverlayPath = path.join(`${vscode.workspace.rootPath}`, "install");
        }
    }

    let wsSetupScript: string = path.format({
        dir: workspaceOverlayPath,
        name: "setup",
        ext: ros_utils.getSetupScriptExtension(),
    });

    if (await pfs.exists(wsSetupScript)) {
        outputChannel.appendLine(`Workspace overlay path: ${wsSetupScript}`);

        try {
            newEnv = await ros_utils.sourceSetupFile(wsSetupScript, newEnv);
        } catch (_err) {
            vscode.window.showErrorMessage("Failed to source the workspace setup file.");
        }
    } else {
        outputChannel.appendLine(`Not sourcing workspace does not exist yet: ${wsSetupScript}. Need to build workspace.`);
    }

    env = newEnv;

    // Notify listeners the environment has changed.
    onEnvChanged.fire();
}

/**
 * Shows the MCP server terminal.
 */
function showMcpServerTerminal(): void {
    if (mcpServerTerminal && !mcpServerTerminal.exitStatus) {
        mcpServerTerminal.show();
    }
}

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
            disposable.dispose();
        }
    });
    
    extensionContext.subscriptions.push(disposable);
    
    return mcpServerTerminal;
}

