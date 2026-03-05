// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import { promises as fsPromises } from "fs";
import * as vscode from "vscode";
import * as child_process from "child_process";

import * as cpp_formatter from "./cpp-formatter";
import * as telemetry from "./telemetry-helper";
import * as vscode_utils from "./vscode-utils";

import * as buildtool from "./build-tool/build-tool";

import * as ros_build_utils from "./ros/build-env-utils";
import * as ros_cli from "./ros/cli";
import * as ros_utils from "./ros/utils";
import { rosApi, selectROSApi } from "./ros/ros";
import * as lifecycle from "./ros/ros2/lifecycle";
import { registerRosMessageProviders } from "./ros/ros-msg-providers";
import { registerLaunchLinkProvider } from "./ros/launch-link-provider";

import * as debug_manager from "./debugger/manager";
import * as debug_utils from "./debugger/utils";
import { registerRosShellTaskProvider } from "./build-tool/ros-shell";
import { RosTestProvider } from "./test-provider/ros-test-provider";
import { LaunchTreeDataProvider } from "./ros/launch-tree/launch-tree-provider";
import { registerPackageDecorationProvider, refreshPackageDecoration } from "./build-tool/package-decorator";

import * as mcp from "./mcp";

/**
 * Check if a file or directory exists.
 */
async function exists(filePath: string): Promise<boolean> {
    try {
        await fsPromises.access(filePath);
        return true;
    } catch {
        return false;
    }
}

/**
 * The sourced ROS environment.
 */
export let env: any;
export let processingWorkspace = false;

export let extPath: string;
export let outputChannel: vscode.OutputChannel;
export let extensionContext: vscode.ExtensionContext | null = null;
export let rosTestProvider: RosTestProvider | null = null;
export let launchTreeProvider: LaunchTreeDataProvider | null = null;

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
export let subscriptions = <vscode.Disposable[]>[];

export enum Commands {
    CreateTerminal = "ROS2.createTerminal",
    GetDebugSettings = "ROS2.getDebugSettings",
    Rosrun = "ROS2.rosrun",
    Roslaunch = "ROS2.roslaunch",
    Rostest = "ROS2.rostest",
    Rosdep = "ROS2.rosdep",
    ShowCoreStatus = "ROS2.showCoreStatus",
    TestsRefresh = "ROS2.tests.refresh",
    TestsRunAll = "ROS2.tests.runAll",
    TestsDebugAll = "ROS2.tests.debugAll",
    StartRosCore = "ROS2.startCore",
    TerminateRosCore = "ROS2.stopCore",
    UpdateCppProperties = "ROS2.updateCppProperties",
    UpdatePythonPath = "ROS2.updatePythonPath",
    PreviewURDF = "ROS2.previewUrdf",
    Doctor = "ROS2.doctor",
    LifecycleListNodes = "ROS2.lifecycle.listNodes",
    LifecycleGetState = "ROS2.lifecycle.getState",
    LifecycleSetState = "ROS2.lifecycle.setState",
    LifecycleTriggerTransition = "ROS2.lifecycle.triggerTransition",
    ShowWelcome = "ROS2.showWelcome",
    LaunchTreeRefresh = "ROS2.launchTree.refresh",
    LaunchTreeReveal = "ROS2.launchTree.reveal",
    LaunchTreeFindUsages = "ROS2.launchTree.findUsages",
    LaunchTreeRun = "ROS2.launchTree.run",
    LaunchTreeDebug = "ROS2.launchTree.debug",
    ColconToggleIgnore = "ROS2.colcon.toggleIgnore",
    ColconBuildPackageRelease = "ROS2.colcon.buildPackageRelease",
    ColconBuildPackageDebug = "ROS2.colcon.buildPackageDebug"
}

/**
 * The walkthrough ID for the getting started guide
 */
/**
 * The walkthrough ID for the getting started guide
 */
const WALKTHROUGH_ID = "Ranch-Hand-Robotics.rde-ros-2#ros2.gettingStarted";
const UPDATED_WELCOME_PROMPT = "The Robot Developer Extension for ROS 2 has been updated, would you like to see the welcome screen";
const UPDATED_WELCOME_PROMPT_YES = "Yes";
const UPDATED_WELCOME_PROMPT_NO = "No";
const UPDATED_WELCOME_PROMPT_NEVER = "Never show again";

/**
 * Updates workspace-scoped context keys used by UI visibility conditions.
 */
async function updateWorkspaceContextKeys(): Promise<void> {
    const hasPackageXml = await vscode_utils.workspaceContainsPackageXml(5);
    await vscode.commands.executeCommand("setContext", "ros2.hasPackageXml", hasPackageXml);
}

export async function activate(context: vscode.ExtensionContext) {
    try {
        const reporter = telemetry.getReporter();
        extPath = context.extensionPath;
        outputChannel = vscode_utils.createOutputChannel();
        extensionContext = context; // Store the context for later use
        context.subscriptions.push(outputChannel);

        // Set workspace context keys used by view visibility.
        await updateWorkspaceContextKeys();

        // Set explicit platform context keys for walkthrough visibility.
        const isLinuxHost = process.platform === "linux";
        const isWindowsHost = process.platform === "win32";
        const isMacHost = process.platform === "darwin";
        await Promise.all([
            vscode.commands.executeCommand("setContext", "ros2.isLinuxHost", isLinuxHost),
            vscode.commands.executeCommand("setContext", "ros2.isWindowsHost", isWindowsHost),
            vscode.commands.executeCommand("setContext", "ros2.isMacHost", isMacHost),
        ]);

        // Log extension activation
        outputChannel.appendLine("ROS 2 Extension activating...");
        outputChannel.appendLine(`Platform context: linux=${isLinuxHost}, windows=${isWindowsHost}, mac=${isMacHost}`);
        
    } catch (error) {
        console.error("Error during extension activation:", error);
        throw error;
    }

    // Detect C++ debugging capabilities
    const isLldbInstalled = vscode_utils.isLldbExtensionInstalled();
    const isCppToolsInstalled = vscode_utils.isCppToolsExtensionInstalled();
    const isCursor = vscode_utils.isCursorEditor();
    
    if (isCppToolsInstalled) {
        outputChannel.appendLine("Microsoft C/C++ extension is installed - C++ debugging via cpptools available");
    } else if (isLldbInstalled) {
        outputChannel.appendLine("LLDB extension is installed - C++ debugging with LLDB available");
    } else if (isCursor) {
        outputChannel.appendLine("No C++ debugger detected - install LLDB extension for C++ debugging");
    } else {
        outputChannel.appendLine("No C++ debugger detected - install Microsoft C/C++ extension (ms-vscode.cpptools) for C++ debugging");
    }

    // Activate components when the ROS env is changed.
    context.subscriptions.push(onDidChangeEnv(activateEnvironment.bind(null, context)));

    // Keep workspace visibility context updated.
    context.subscriptions.push(vscode.workspace.onDidChangeWorkspaceFolders(() => {
        void updateWorkspaceContextKeys();
    }));
    context.subscriptions.push(vscode.workspace.onDidCreateFiles((event) => {
        if (event.files.some(file => path.basename(file.fsPath) === "package.xml")) {
            void updateWorkspaceContextKeys();
        }
    }));
    context.subscriptions.push(vscode.workspace.onDidDeleteFiles((event) => {
        if (event.files.some(file => path.basename(file.fsPath) === "package.xml")) {
            void updateWorkspaceContextKeys();
        }
    }));

    // Activate components which don't require the ROS env.
    context.subscriptions.push(vscode.languages.registerDocumentFormattingEditProvider(
        "cpp", new cpp_formatter.CppFormatter()
    ));

    // Register ROS message language providers (Definition and Hover)
    context.subscriptions.push(...registerRosMessageProviders(context));

    // Register launch file link provider
    context.subscriptions.push(registerLaunchLinkProvider());

    // Initialize ROS 2 test provider (once during extension activation, not on environment changes)
    rosTestProvider = new RosTestProvider(context);
    context.subscriptions.push(rosTestProvider);

    // Initialize Launch Tree Provider
    launchTreeProvider = new LaunchTreeDataProvider(context, outputChannel, extPath);
    const launchTreeView = vscode.window.createTreeView('ros2LaunchTree', {
        treeDataProvider: launchTreeProvider,
        showCollapseAll: true
    });
    context.subscriptions.push(launchTreeView);
    context.subscriptions.push(launchTreeProvider);

    // Source the environment, and re-source on config change.
    let config = vscode_utils.getExtensionConfiguration();

    // Conditionally register package decoration provider based on setting
    let decorationProviderDisposable: vscode.Disposable | undefined;
    const updateDecorationRegistration = (enabled: boolean): void => {
        if (enabled && !decorationProviderDisposable) {
            decorationProviderDisposable = registerPackageDecorationProvider();
            context.subscriptions.push(decorationProviderDisposable);
        } else if (!enabled && decorationProviderDisposable) {
            decorationProviderDisposable.dispose();
            decorationProviderDisposable = undefined;
        }
    };
    updateDecorationRegistration(config.enableFileDecorations === true);

    context.subscriptions.push(vscode.workspace.onDidChangeConfiguration(() => {
        const updatedConfig = vscode_utils.getExtensionConfiguration();
        const fields = Object.keys(config).filter(k => !(config[k] instanceof Function));
        const changed = fields.some(key => updatedConfig[key] !== config[key]);

        if (changed) {
            sourceRosAndWorkspace();
        }

        updateDecorationRegistration(updatedConfig.enableFileDecorations === true);

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

    // Register Test commands
    vscode.commands.registerCommand(Commands.TestsRefresh, () => {
        ensureErrorMessageOnException(() => {
            if (rosTestProvider) {
                rosTestProvider.refresh();
                vscode.window.showInformationMessage("ROS 2 test discovery refreshed");
            } else {
                vscode.window.showWarningMessage("ROS 2 test provider not initialized");
            }
        });
    });

    vscode.commands.registerCommand(Commands.TestsRunAll, () => {
        ensureErrorMessageOnException(async () => {
            if (rosTestProvider) {
                await vscode.commands.executeCommand('test-explorer.run-all');
            } else {
                vscode.window.showWarningMessage("ROS 2 test provider not initialized");
            }
        });
    });

    vscode.commands.registerCommand(Commands.TestsDebugAll, () => {
        ensureErrorMessageOnException(async () => {
            if (rosTestProvider) {
                await vscode.commands.executeCommand('test-explorer.debug-all');
            } else {
                vscode.window.showWarningMessage("ROS 2 test provider not initialized");
            }
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

    // Register Welcome/Walkthrough command
    vscode.commands.registerCommand(Commands.ShowWelcome, () => {
        ensureErrorMessageOnException(() => {
            vscode.commands.executeCommand('workbench.action.openWalkthrough', WALKTHROUGH_ID);
        });
    });

    // Register Launch Tree commands
    vscode.commands.registerCommand(Commands.LaunchTreeRefresh, () => {
        ensureErrorMessageOnException(() => {
            if (launchTreeProvider) {
                launchTreeProvider.refresh();
                vscode.window.showInformationMessage("Launch tree refreshed");
            }
        });
    });

    vscode.commands.registerCommand(Commands.LaunchTreeReveal, async (uri: vscode.Uri) => {
        ensureErrorMessageOnException(async () => {
            // TODO: Implement reveal logic
            vscode.window.showInformationMessage(`Reveal ${uri.fsPath} in tree`);
        });
    });

    vscode.commands.registerCommand(Commands.LaunchTreeFindUsages, async (item: any) => {
        ensureErrorMessageOnException(async () => {
            if (!item || !item.launchFilePath) {
                return;
            }
            const fileName = path.basename(item.launchFilePath);
            vscode.window.showInformationMessage(`Finding usages of ${fileName}...`);
            // TODO: Implement find usages
        });
    });

    vscode.commands.registerCommand(Commands.LaunchTreeRun, async (item: any) => {
        ensureErrorMessageOnException(async () => {
            if (!item || !item.launchFilePath) {
                return;
            }
            // Delegate to existing roslaunch command
            await vscode.commands.executeCommand(Commands.Roslaunch);
        });
    });

    vscode.commands.registerCommand(Commands.LaunchTreeDebug, async (item: any) => {
        ensureErrorMessageOnException(async () => {
            if (!item || !item.launchFilePath) {
                return;
            }
            // Create debug configuration
            const config: vscode.DebugConfiguration = {
                type: 'ros2',
                name: `Debug ${path.basename(item.launchFilePath)}`,
                request: 'launch',
                target: item.launchFilePath
            };
            // Start debugging
            await vscode.debug.startDebugging(undefined, config);
        });
    });


    // Register Colcon commands
    vscode.commands.registerCommand(Commands.ColconToggleIgnore, async (uri: vscode.Uri) => {
        ensureErrorMessageOnException(async () => {
            const colconUtils = await import("./build-tool/colcon-utils");

            if (!uri || !uri.fsPath) {
                vscode.window.showErrorMessage("Please right-click on a folder to toggle colcon ignore");
                return;
            }

            const workspaceRoot = vscode.workspace.rootPath;
            if (!workspaceRoot) {
                vscode.window.showErrorMessage("No workspace folder found");
                return;
            }

            // Find package for this path
            const packageName = await colconUtils.findPackageForPath(uri.fsPath, workspaceRoot);
            if (!packageName) {
                vscode.window.showWarningMessage("No ROS 2 package found at this location");
                return;
            }

            const ignoreConfig = colconUtils.getColconIgnoreConfig();
            const isIgnored = ignoreConfig[packageName] === true;

            // Toggle the ignore state
            await colconUtils.updateColconIgnoreConfig(packageName, !isIgnored);

            // Update context variable for menu visibility
            await vscode.commands.executeCommand('setContext', 'ros2.packageIgnored', !isIgnored);

            // Give VS Code a moment to persist the config, then refresh the decoration
            setTimeout(() => {
                refreshPackageDecoration(uri);
            }, 100);

            if (isIgnored) {
                vscode.window.showInformationMessage(`Package '${packageName}' will now be included in colcon builds`);
            } else {
                vscode.window.showInformationMessage(`Package '${packageName}' will now be ignored in colcon builds`);
            }
        });
    });

    // Register a command to update the context when a folder is right-clicked
    vscode.commands.registerCommand('ROS2.colcon.updateIgnoredContext', async (uri: vscode.Uri) => {
        if (!uri || !uri.fsPath) {
            return;
        }

        const workspaceRoot = vscode.workspace.rootPath;
        if (!workspaceRoot) {
            return;
        }

        try {
            const colconUtils = await import("./build-tool/colcon-utils");
            const packageName = await colconUtils.findPackageForPath(uri.fsPath, workspaceRoot);
            
            if (packageName) {
                const ignoreConfig = colconUtils.getColconIgnoreConfig();
                const isIgnored = ignoreConfig[packageName] === true;
                await vscode.commands.executeCommand('setContext', 'ros2.packageIgnored', isIgnored);
            }
        } catch (error) {
            // Silently fail - this is just for context update
        }
    });

    vscode.commands.registerCommand(Commands.ColconBuildPackageRelease, async (uri: vscode.Uri) => {
        ensureErrorMessageOnException(async () => {
            const colconUtils = await import("./build-tool/colcon-utils");
            const colcon = await import("./build-tool/colcon");
            
            if (!uri || !uri.fsPath) {
                vscode.window.showErrorMessage("Please right-click on a folder to build a package");
                return;
            }

            const workspaceRoot = vscode.workspace.rootPath;
            if (!workspaceRoot) {
                vscode.window.showErrorMessage("No workspace folder found");
                return;
            }

            // Find package for this path
            const packageName = await colconUtils.findPackageForPath(uri.fsPath, workspaceRoot);
            if (!packageName) {
                vscode.window.showWarningMessage("No ROS 2 package found at this location");
                return;
            }

            // Create and execute the build task (RelWithDebInfo)
            const task = await colcon.makeColconPackageTask(packageName, 'RelWithDebInfo');
            await vscode.tasks.executeTask(task);
        });
    });

    vscode.commands.registerCommand(Commands.ColconBuildPackageDebug, async (uri: vscode.Uri) => {
        ensureErrorMessageOnException(async () => {
            const colconUtils = await import("./build-tool/colcon-utils");
            const colcon = await import("./build-tool/colcon");
            
            if (!uri || !uri.fsPath) {
                vscode.window.showErrorMessage("Please right-click on a folder to build a package");
                return;
            }

            const workspaceRoot = vscode.workspace.rootPath;
            if (!workspaceRoot) {
                vscode.window.showErrorMessage("No workspace folder found");
                return;
            }

            // Find package for this path
            const packageName = await colconUtils.findPackageForPath(uri.fsPath, workspaceRoot);
            if (!packageName) {
                vscode.window.showWarningMessage("No ROS 2 package found at this location");
                return;
            }

            // Create and execute the build task (Debug)
            const task = await colcon.makeColconPackageTask(packageName, 'Debug');
            await vscode.tasks.executeTask(task);
        });
    });

    // Register MCP commands
    mcp.registerMcpCommands(context);

    const reporter = telemetry.getReporter();
    reporter.sendTelemetryActivate();

    // Activate the workspace environment if possible.
    await activateEnvironment(context);

    // Show welcome walkthrough on first install or if ROS is not detected
    await showWelcomeIfNeeded(context);

    return {
        getEnv: () => env,
        onDidChangeEnv: (listener: () => any, thisArg: any) => onDidChangeEnv(listener, thisArg),
    };
}

/**
 * Shows the welcome walkthrough if needed based on first install, version upgrade, or ROS detection
 */
async function showWelcomeIfNeeded(context: vscode.ExtensionContext): Promise<void> {
    const config = vscode_utils.getExtensionConfiguration();
    const showWelcomeOnStartup = config.get("showROS2WelcomeOnStartup", true);
    
    // Check if user has disabled the welcome screen
    if (!showWelcomeOnStartup) {
        return;
    }

    // Double-check this is a ROS workspace before showing walkthrough.
    const hasPackageXml = await vscode_utils.workspaceContainsPackageXml(5);
    if (!hasPackageXml) {
        outputChannel.appendLine("Skipping welcome walkthrough: workspace does not contain package.xml.");
        return;
    }

    // Get current extension version and compare with last shown version
    const currentVersion = context.extension.packageJSON.version as string;
    const lastShownVersion = config.get("lastShownWelcomeVersion", "");
    // Show the walkthrough with a slight delay to ensure VS Code is ready
    setTimeout(async () => {
        if (shouldShowWelcome(lastShownVersion, currentVersion)) {
            const selection = await vscode.window.showInformationMessage(
                    UPDATED_WELCOME_PROMPT,
                    UPDATED_WELCOME_PROMPT_YES,
                    UPDATED_WELCOME_PROMPT_NO,
                    UPDATED_WELCOME_PROMPT_NEVER
                );

            await config.update("lastShownWelcomeVersion", currentVersion);
            if (selection === UPDATED_WELCOME_PROMPT_NEVER) {
                await config.update("showROS2WelcomeOnStartup",
                    false, vscode.ConfigurationTarget.Global);
                return;
            } else if (selection === UPDATED_WELCOME_PROMPT_NO || selection === undefined) {
                return;
            } else {
                vscode.commands.executeCommand('workbench.action.openWalkthrough', WALKTHROUGH_ID);
            }
        }
    }, 5000);
}

/**
 * Compare two semantic versions and determine if welcome should be shown.
 * Welcome is shown on first install and on major/minor upgrades only.
 * Patch-only updates are intentionally ignored.
 * @param lastVersion The last version the welcome was shown (empty string if never)
 * @param currentVersion The current extension version
 * @returns true if welcome should be shown (first install or major/minor upgrade)
 */
export function shouldShowWelcome(lastVersion: string, currentVersion: string): boolean {
    // Show on first install (lastVersion is empty)
    if (!lastVersion) {
        return true;
    }
    
    // Product decision: compare major/minor only, ignore patch updates for welcome prompts.
    return vscode_utils.compareVersions(lastVersion, currentVersion, true) < 0;
}

/**
 * Resolves the user's welcome prompt selection, defaulting to undefined when the timeout expires.
 */
export async function resolveWelcomePromptSelectionWithTimeout(
    selectionPromise: Thenable<string | undefined>,
    timeoutMs: number,
): Promise<string | undefined> {
    if (timeoutMs <= 0) {
        return undefined;
    }

    let timeoutHandle: NodeJS.Timeout | undefined;
    const timeoutPromise = new Promise<undefined>((resolve) => {
        timeoutHandle = setTimeout(() => resolve(undefined), timeoutMs);
    });

    try {
        return await Promise.race([
            Promise.resolve(selectionPromise),
            timeoutPromise,
        ]);
    } finally {
        if (timeoutHandle) {
            clearTimeout(timeoutHandle);
        }
    }
}

export async function deactivate() {
    subscriptions.forEach(disposable => disposable.dispose());
    await telemetry.clearReporter();
    mcp.shutdownMcpServer();
    
    // Clean up test provider
    if (rosTestProvider) {
        rosTestProvider.dispose();
        rosTestProvider = null;
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
export async function activateEnvironment(context: vscode.ExtensionContext) {

    if (processingWorkspace) {
        return;
    }

    processingWorkspace = true;

    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop().dispose();
    }

    await sourceRosAndWorkspace();

    if (!env || typeof env.ROS_DISTRO === "undefined") {
        outputChannel.appendLine("ROS environment not detected. ROS 2 features will be limited. Please install ROS 2 or configure a ROS setup script.");
        processingWorkspace = false;
        return;
    }

    if (typeof env.ROS_VERSION === "undefined") {
        outputChannel.appendLine("ROS_VERSION not set in environment. Please verify your ROS 2 installation.");
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

    // If no setup script is configured, try to get one from the workspace (e.g., via pixi if configured)
    if (!rosSetupScript) {
        rosSetupScript = vscode_utils.getRosSetupScript();
    }

    // If the workspace setup script is not set, try to find the ROS setup script in the environment
    let attemptWorkspaceDiscovery = true;

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
        if (await exists(rosSetupScript)) {
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
        outputChannel.appendLine(`No ROS 2 distro configured, attempting ROS 2 distro auto-discovery`);
        if (!distro) {
            // No? Try to find one.
            const installedDistros = await ros_utils.getDistros();
            if (!installedDistros.length) {
                outputChannel.appendLine(`No ROS 2 distros found.`);

                const message = "No ROS 2 distros found. Please install a ROS 2 distribution.";
                await vscode.window.setStatusBarMessage(message, kWorkspaceConfigTimeout);
            } else if (installedDistros.length === 1) {
                outputChannel.appendLine(`Only one ROS 2 distro found, selecting ${installedDistros[0]}`);

                // if there is only one ROS 2 distro installed, directly choose it
                config.update("distro", installedDistros[0]);
                distro = installedDistros[0];
            } else {
                outputChannel.appendLine(`Multiple ROS 2 distros found, prompting user to select one.`);
                // dump installedDistros to outputChannel
                outputChannel.appendLine(`Installed ROS 2 distros: ${installedDistros}`);

                const message = "Unable to determine ROS 2 distribution, please configure this workspace by adding \"ROS2.distro\": \"<ROS 2 Distro>\" in settings.json";
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

    if (newEnv && newEnv.ROS_VERSION === "1") {
        outputChannel.appendLine(`RDE ROS 2 does not support ROS 1`);
    } else if (newEnv) {    // FUTURE: Revisit if ROS_VERSION changes - not clear it will be called 3
        if (!await exists(workspaceOverlayPath)) {
            workspaceOverlayPath = path.join(`${vscode.workspace.rootPath}`, "install");
        }
    }

    let wsSetupScript: string = path.format({
        dir: workspaceOverlayPath,
        name: "setup",
        ext: ros_utils.getSetupScriptExtension(),
    });

    if (await exists(wsSetupScript)) {
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

