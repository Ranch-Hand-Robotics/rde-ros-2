// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import { promises as fsPromises } from "fs";
import * as os from "os";
import * as vscode from "vscode";
import * as child_process from "child_process";
import { promisify } from "util";

import * as cpp_formatter from "./cpp-formatter";
import * as telemetry from "./telemetry-helper";
import * as vscode_utils from "./vscode-utils";

import * as buildtool from "./build-tool/build-tool";
import { COLCON_TASK_TYPE, ColconProvider } from "./build-tool/colcon";

import * as ros_build_utils from "./ros/build-env-utils";
import * as ros_cli from "./ros/cli";
import * as ros_utils from "./ros/utils";
import { rosApi, selectROSApi } from "./ros/ros";
import * as lifecycle from "./ros/ros2/lifecycle";
import { registerRosMessageProviders } from "./ros/ros-msg-providers";
import { registerLaunchLinkProvider } from "./ros/launch-link-provider";
import * as install_ros from "./ros/installer/install-ros";

import * as debug_manager from "./debugger/manager";
import * as debug_utils from "./debugger/utils";
import { registerRosShellTaskProvider } from "./build-tool/ros-shell";
import { RosTestProvider } from "./test-provider/ros-test-provider";
import { LaunchTreeDataProvider } from "./ros/launch-tree/launch-tree-provider";
import { detectInstalledDistros, RosDistributionsProvider, selectInstalledDistro } from "./ros/ros-distributions-provider";
import { registerPackageDecorationProvider, refreshPackageDecoration } from "./build-tool/package-decorator";
import { TopicTreeDataProvider } from "./ros/topic-tree/topic-tree-provider";
import { TopicTreeItem } from "./ros/topic-tree/topic-tree-item";
import { TopicWebviewManager } from "./ros/ros2/topic-webview";
import { getPixiInstallRoot } from "./ros/installer/pixi-location";
import { buildInstallPrefixes, buildParentScripts, cleanBuildEnvironment, isWorkspaceInstall, RosBuildOptions, sameBuildScript } from "./ros/build-environment";

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
let environmentActivation: Promise<void> | undefined;
let colconTaskProvider: vscode.Disposable | undefined;

export let extPath: string;
export let outputChannel: vscode.OutputChannel;
export let extensionContext: vscode.ExtensionContext | null = null;
export let rosTestProvider: RosTestProvider | null = null;
export let launchTreeProvider: LaunchTreeDataProvider | null = null;
export let topicTreeProvider: TopicTreeDataProvider | null = null;
export let topicWebviewManager: TopicWebviewManager | null = null;
let topicTreeView: vscode.TreeView<TopicTreeItem> | null = null;
export let rosDistributionsProvider: RosDistributionsProvider | null = null;

let onEnvChanged = new vscode.EventEmitter<void>();
// Readiness also changes on topic-only sourcing, without reactivating providers.
const onEnvResolved = new vscode.EventEmitter<void>();

/**
 * Triggered when the env is soured.
 */
export let onDidChangeEnv = onEnvChanged.event;

export async function resolvedEnv() {
    if (env === undefined) { // Env reload in progress
        await debug_utils.oneTimePromiseFromEvent(onEnvResolved.event);
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
    ColconBuildPackageDebug = "ROS2.colcon.buildPackageDebug",
    TopicTreeRefresh = "ROS2.topicTree.refresh",
    TopicTreeStartWatcher = "ROS2.topicTree.startWatcher",
    TopicTreePauseWatcher = "ROS2.topicTree.pauseWatcher",
    TopicTreePauseAll = "ROS2.topicTree.pauseAll",
    InstallRos = "ROS2.installRos",
    CheckRosInstallation = "ROS2.checkInstallation",
    ShowInstallationReport = "ROS2.showInstallationReport",
    FindRos = "ROS2.findRos",
    SetActiveDistro = "ROS2.setActiveDistro",
    RefreshDistributions = "ROS2.distributions.refresh",
    RemoveDistribution = "ROS2.distributions.remove"
}

function syncTopicMonitoringState(): void {
    const monitoringEnabled = topicTreeView?.visible === true
        && topicTreeProvider?.isWatcherEnabled() === true;
    topicWebviewManager?.setMonitoringEnabled(monitoringEnabled);
}

async function refreshVisibleTopicTree(): Promise<void> {
    if (topicTreeView?.visible !== true) {
        return;
    }

    await sourceRosAndWorkspace(false);
    await topicTreeProvider?.refreshTopics();
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
    ensureColconTaskProvider(context);
    try {
        const reporter = telemetry.getReporter();
        extPath = context.extensionPath;
        outputChannel = vscode_utils.createOutputChannel();
        extensionContext = context; // Store the context for later use
        context.subscriptions.push(outputChannel);

        // Set workspace context keys used by view visibility.
        void ensureErrorMessageOnException(updateWorkspaceContextKeys);

        // Set explicit platform context keys for walkthrough visibility.
        const isLinuxHost = process.platform === "linux";
        const isWindowsHost = process.platform === "win32";
        const isMacHost = process.platform === "darwin";
        await Promise.all([
            vscode.commands.executeCommand("setContext", "ros2.isLinuxHost", isLinuxHost),
            vscode.commands.executeCommand("setContext", "ros2.isWindowsHost", isWindowsHost),
            vscode.commands.executeCommand("setContext", "ros2.isMacHost", isMacHost),
            vscode.commands.executeCommand("setContext", "ros2.hasDistributions", false),
            vscode.commands.executeCommand("setContext", "ros2.distributionSearchComplete", false),
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

    // Initialize ROS Distributions Provider
    rosDistributionsProvider = new RosDistributionsProvider();
    const distributionsView = vscode.window.createTreeView('ros2Distributions', {
        treeDataProvider: rosDistributionsProvider,
        showCollapseAll: false
    });
    context.subscriptions.push(distributionsView);
    context.subscriptions.push(rosDistributionsProvider);

    // Initialize Launch Tree Provider
    launchTreeProvider = new LaunchTreeDataProvider(context, outputChannel, extPath);
    const launchTreeView = vscode.window.createTreeView('ranchhandrobotics.rde-ros-2.launchTree', {
        treeDataProvider: launchTreeProvider,
        showCollapseAll: true
    });
    context.subscriptions.push(launchTreeView);
    context.subscriptions.push(launchTreeProvider);

    // Initialize Topic Webview Manager
    topicWebviewManager = new TopicWebviewManager(context, (topicName) => {
        topicTreeProvider?.markTopicMonitorClosed(topicName);
    });
    context.subscriptions.push(topicWebviewManager);

    // Initialize Topic Tree Provider
    topicTreeProvider = new TopicTreeDataProvider(
        context,
        outputChannel,
        (topic, subscribe: boolean) => {
            if (subscribe && topicWebviewManager) {
                topicWebviewManager.openTopicMonitor(topic.name, topic.type);
            } else if (!subscribe && topicWebviewManager) {
                topicWebviewManager.closeTopicMonitor(topic.name);
            }
        }
    );
    topicTreeView = vscode.window.createTreeView('ranchhandrobotics.rde-ros-2.topicTree', {
        treeDataProvider: topicTreeProvider,
        showCollapseAll: true
    });
    topicTreeProvider.setViewVisible(topicTreeView.visible);
    await vscode.commands.executeCommand("setContext", "ros2.topicWatcherEnabled", false);

    // Query topics only after sourceRosAndWorkspace has atomically replaced env.
    // This avoids the initial placeholder render consuming the refresh before ROS is ready.
    context.subscriptions.push(onDidChangeEnv(() => {
        if (topicTreeView?.visible === true) {
            void topicTreeProvider?.refreshTopics();
        }
    }));
    
    // Handle checkbox changes
    context.subscriptions.push(topicTreeView.onDidChangeCheckboxState((event) => {
        void topicTreeProvider?.handleCheckboxChange([event]);
    }));
    context.subscriptions.push(topicTreeView.onDidChangeVisibility((event) => {
        topicTreeProvider?.setViewVisible(event.visible);
        syncTopicMonitoringState();
        if (event.visible) {
            void refreshVisibleTopicTree();
        }
    }));
    
    context.subscriptions.push(topicTreeView);
    context.subscriptions.push(topicTreeProvider);

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

    context.subscriptions.push(vscode.workspace.onDidChangeConfiguration((event) => {
        const updatedConfig = vscode_utils.getExtensionConfiguration();
        if (event.affectsConfiguration("ROS2.rosSetupScript") || event.affectsConfiguration("ROS2.distro") ||
            event.affectsConfiguration("ROS2.pixiRoot")) {
            void ensureErrorMessageOnException(() => activateEnvironment(context));
        }

        updateDecorationRegistration(updatedConfig.enableFileDecorations === true);

        config = updatedConfig;
    }));

    vscode.commands.registerCommand(Commands.CreateTerminal, () => {
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => ros_utils.createTerminal(context)));
    });

    vscode.commands.registerCommand(Commands.GetDebugSettings, () => {
        ensureErrorMessageOnException(() => {
            return debug_utils.getDebugSettings(context);
        });
    });

    vscode.commands.registerCommand(Commands.ShowCoreStatus, () => {
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => rosApi.showCoreMonitor()));
    });

    vscode.commands.registerCommand(Commands.StartRosCore, () => {
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => rosApi.startCore()));
    });

    vscode.commands.registerCommand(Commands.TerminateRosCore, () => {
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => rosApi.stopCore()));
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
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => ros_cli.rosrun(context)));
    });

    vscode.commands.registerCommand(Commands.Roslaunch, () => {
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => ros_cli.roslaunch(context)));
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
        return ensureErrorMessageOnException(() => withRosEnvironment(context, () => rosApi.doctor()));
    });

    // Register Install ROS command
    context.subscriptions.push(vscode.commands.registerCommand(Commands.InstallRos, () =>
        ensureErrorMessageOnException(() => install_ros.installRos())
    ));
    context.subscriptions.push(vscode.commands.registerCommand(Commands.CheckRosInstallation, async (target) => {
        try {
            return await install_ros.checkRosInstallation(target);
        } catch (error) {
            vscode.window.showErrorMessage(error instanceof Error ? error.message : String(error));
            throw error;
        }
    }));
    context.subscriptions.push(vscode.commands.registerCommand(Commands.ShowInstallationReport, () =>
        ensureErrorMessageOnException(() => install_ros.showInstallationReport())
    ));

    // Register Find ROS command
    vscode.commands.registerCommand(Commands.FindRos, async () => {
        const pickAndSetRosSetupScript = async (scriptPath: string): Promise<void> => {
            if (!await vscode_utils.setRosSetupScript(scriptPath)) {
                return;
            }
            vscode.window.showInformationMessage(`ROS setup script set to: ${scriptPath}`);
            if (rosDistributionsProvider) {
                rosDistributionsProvider.refresh();
            }
        };

        // Fast-path: look in configured/default Pixi roots for known setup scripts.
        const config = vscode.workspace.getConfiguration("ROS2");
        const configuredPixiRoot = getPixiInstallRoot();
        const defaultPixiRoot = process.platform === "win32"
            ? "c:\\pixi_ws"
            : path.join(os.homedir(), "pixi_ws");
        const pixiRoots = Array.from(new Set([configuredPixiRoot, defaultPixiRoot].filter(Boolean)));

        const candidates: string[] = [];
        for (const root of pixiRoots) {
            if (process.platform === "win32") {
                candidates.push(path.join(root, "ros2-windows", "local_setup.bat"));
                try {
                    const entries = await fsPromises.readdir(root, { withFileTypes: true });
                    for (const entry of entries) {
                        if (!entry.isDirectory()) {
                            continue;
                        }
                        candidates.push(path.join(root, entry.name, "install", "setup.bat"));
                        candidates.push(path.join(root, entry.name, "local_setup.bat"));
                        candidates.push(path.join(root, entry.name, ".pixi", "envs", entry.name, "Library", "local_setup.bat"));
                        candidates.push(path.join(root, entry.name, ".pixi", "envs", entry.name, "Library", "setup.bat"));
                    }
                } catch {
                    // ignore missing roots
                }
            } else {
                candidates.push(path.join(root, "install", "setup.bash"));
                candidates.push(path.join(root, "local_setup.bash"));
                candidates.push(path.join(root, "local_setup.sh"));
                try {
                    const entries = await fsPromises.readdir(root, { withFileTypes: true });
                    for (const entry of entries) {
                        if (!entry.isDirectory()) {
                            continue;
                        }
                        candidates.push(path.join(root, entry.name, "install", "setup.bash"));
                        candidates.push(path.join(root, entry.name, "setup.bash"));
                        candidates.push(path.join(root, entry.name, "local_setup.bash"));
                        candidates.push(path.join(root, entry.name, "local_setup.sh"));
                    }
                } catch {
                    // ignore missing roots
                }
            }
        }

        const existingCandidates: string[] = [];
        const seen = new Set<string>();
        for (const candidate of candidates) {
            const normalized = path.normalize(candidate);
            if (seen.has(normalized)) {
                continue;
            }
            seen.add(normalized);
            if (await exists(normalized)) {
                existingCandidates.push(normalized);
            }
        }

        if (existingCandidates.length === 1) {
            await pickAndSetRosSetupScript(existingCandidates[0]);
            return;
        }

        if (existingCandidates.length > 1) {
            const selected = await vscode.window.showQuickPick(
                existingCandidates.map((p) => ({ label: path.basename(p), description: p, path: p })),
                {
                    placeHolder: "Select a discovered ROS setup script from Pixi roots",
                    ignoreFocusOut: true,
                }
            );
            if (selected?.path) {
                await pickAndSetRosSetupScript(selected.path);
                return;
            }
        }

        // Fall back to manual browse if quick discovery didn't find anything.
        const isWindows = process.platform === "win32";
        const filters: Record<string, string[]> = isWindows
            ? { "ROS Setup Script": ["bat"] }
            : { "ROS Setup Script": ["bash", "sh"] };

        const uris = await vscode.window.showOpenDialog({
            canSelectFiles: true,
            canSelectFolders: false,
            canSelectMany: false,
            openLabel: "Select ROS Setup Script",
            filters,
        });

        if (uris && uris.length > 0) {
            await pickAndSetRosSetupScript(uris[0].fsPath);
        }
    });

    // Register Set Active Distro command
    vscode.commands.registerCommand(Commands.SetActiveDistro, async (setupScript: string, distro?: string) => {
        if (!await vscode_utils.setRosSetupScript(setupScript, distro)) {
            return;
        }
        vscode.window.showInformationMessage(`Active ROS distribution set.`);
        if (rosDistributionsProvider) {
            rosDistributionsProvider.refresh();
        }
    });

    context.subscriptions.push(vscode.commands.registerCommand(Commands.RemoveDistribution, (item) => {
        return ensureErrorMessageOnException(() => rosDistributionsProvider?.remove(item));
    }));

    // Register Refresh Distributions command
    vscode.commands.registerCommand(Commands.RefreshDistributions, () => {
        if (rosDistributionsProvider) {
            rosDistributionsProvider.refresh();
        }
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

    // Register Topic Tree commands
    let topicWatcherRequest = 0;
    vscode.commands.registerCommand(Commands.TopicTreeRefresh, () => {
        return ensureErrorMessageOnException(async () => {
            await sourceRosAndWorkspace(false);
            await topicTreeProvider?.refreshTopics();
        });
    });

    vscode.commands.registerCommand(Commands.TopicTreeStartWatcher, () => {
        const request = ++topicWatcherRequest;
        return ensureErrorMessageOnException(async () => {
            await sourceRosAndWorkspace(false);
            // A Pause (or newer Play) during sourcing supersedes this request.
            if (request !== topicWatcherRequest) {
                return;
            }
            if (env?.ROS_VERSION !== "2") {
                throw new Error("No ROS 2 environment is configured. Use ROS2: Find ROS or select an installed distribution.");
            }
            topicTreeProvider?.setWatcherEnabled(true);
            syncTopicMonitoringState();
            // Subscriptions and the Pause button must not wait for the ROS graph.
            // Join the provider's in-flight query before yielding; do not change
            // watcher state after it completes, since Pause may have intervened.
            await Promise.all([
                vscode.commands.executeCommand("setContext", "ros2.topicWatcherEnabled", true),
                topicTreeProvider?.refreshTopics(),
            ]);
        });
    });

    vscode.commands.registerCommand(Commands.TopicTreePauseWatcher, () => {
        ++topicWatcherRequest;
        return ensureErrorMessageOnException(async () => {
            topicTreeProvider?.setWatcherEnabled(false);
            syncTopicMonitoringState();
            await vscode.commands.executeCommand("setContext", "ros2.topicWatcherEnabled", false);
        });
    });

    vscode.commands.registerCommand(Commands.TopicTreePauseAll, () => {
        return ensureErrorMessageOnException(async () => {
            await topicTreeProvider?.unsubscribeAll();
            vscode.window.showInformationMessage("All topic monitors stopped");
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

    // Task discovery must not await ROS sourcing or an installation prompt.
    // Runtime commands still wait for environmentActivation when they need ROS.
    void ensureErrorMessageOnException(async () => {
        await activateEnvironment(context);
        await refreshVisibleTopicTree();
        await showWelcomeIfNeeded(context);
    });

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
    colconTaskProvider?.dispose();
    colconTaskProvider = undefined;
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
        vscode.window.showErrorMessage(err instanceof Error ? err.message : String(err));
    }
}

/**
 * Activates components which require a ROS env.
 */
async function withRosEnvironment(context: vscode.ExtensionContext, callback: () => any): Promise<any> {
    if (environmentActivation) {
        await environmentActivation;
    } else if (env?.ROS_VERSION !== "2") {
        await activateEnvironment(context);
    }
    if (env?.ROS_VERSION !== "2") {
        throw new Error("No ROS 2 environment is configured. Use ROS2: Find ROS or select an installed distribution. No workspace is required; choose Set Global Default in an empty window.");
    }
    return callback();
}

/** Colcon discovery belongs to the extension lifetime, not the ROS environment. */
function ensureColconTaskProvider(context: vscode.ExtensionContext): void {
    if (!colconTaskProvider) {
        colconTaskProvider = vscode.tasks.registerTaskProvider(COLCON_TASK_TYPE, new ColconProvider());
        context.subscriptions.push(colconTaskProvider);
    }
}

export function activateEnvironment(context: vscode.ExtensionContext): Promise<void> {
    ensureColconTaskProvider(context);
    if (!environmentActivation) {
        environmentActivation = activateEnvironmentImpl(context).finally(() => {
            processingWorkspace = false;
            environmentActivation = undefined;
        });
    }
    return environmentActivation;
}

async function activateEnvironmentImpl(context: vscode.ExtensionContext) {

    if (processingWorkspace) {
        return;
    }

    processingWorkspace = true;

    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop()?.dispose();
    }

    await sourceRosAndWorkspace();

    if (typeof env?.ROS_DISTRO === "undefined") {
        // ROS is not detected, check if we should prompt for installation
        await install_ros.promptInstallRosIfNeeded();
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
    let buildToolDetected = await buildtool.determineBuildTool(vscode.workspace.rootPath ?? "");

    // http://www.ros.org/reps/rep-0149.html#environment-variables
    // Learn more about ROS_VERSION definition.
    selectROSApi(env.ROS_VERSION);

    // Do this again, after the build tool has been determined.
    await sourceRosAndWorkspace();

    rosApi.setContext(context, env);

    subscriptions.push(rosApi.activateCoreMonitor());
    if (!buildToolDetected) {
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
async function sourceRosAndWorkspace(
    notifyEnvironmentChange: boolean = true, baseEnv?: NodeJS.ProcessEnv, forBuild: boolean = false,
    buildPrefixes: string[] = [], onUnderlay?: (script: string) => void,
): Promise<NodeJS.ProcessEnv | undefined> {

    // Processing a new environment can take time which introduces a race condition. 
    // Wait to atomicly switch by composing a new environment block then switching at the end.
    let newEnv: Record<string, string | undefined> | undefined = undefined;

    outputChannel.appendLine("Sourcing ROS and Workspace");

    const kWorkspaceConfigTimeout = 30000; // ms

    const config = vscode_utils.getExtensionConfiguration();
    const sourceUnderlay = async (script: string): Promise<NodeJS.ProcessEnv> => {
        if (forBuild && isWorkspaceInstall(script, buildPrefixes)) {
            throw new Error(`Selected ROS underlay is inside the current workspace install: ${script}. Select the external ROS/Pixi setup in ROS2.rosSetupScript before rebuilding.`);
        }
        const sourced = await ros_utils.sourceSetupFile(script, baseEnv, forBuild);
        onUnderlay?.(script);
        return sourced;
    };
    const reportFailure = (script: string, error: unknown): void => {
        const failure = error as { message?: string; code?: string | number; signal?: string; stderr?: string };
        const reason = failure?.message ?? String(error);
        outputChannel.appendLine(`[ROS setup failed] ${script}\n${reason}`);
        if (failure?.code !== undefined) { outputChannel.appendLine(`Exit/error code: ${failure.code}`); }
        if (failure?.signal) { outputChannel.appendLine(`Signal: ${failure.signal}`); }
        if (failure?.stderr?.trim() && !reason.includes(failure.stderr.trim())) {
            outputChannel.appendLine(failure.stderr.trim());
        }
        vscode_utils.showOutputPanel(outputChannel);
    };

    // Only an explicit setup script may bypass distro selection. The utility's
    // implicit legacy Pixi default may belong to a different distro.
    let rosSetupScript = config.get<string>("rosSetupScript") ? vscode_utils.getRosSetupScript() : "";

    // If the workspace setup script is not set, try to find the ROS setup script in the environment
    let attemptWorkspaceDiscovery = true;

    if (rosSetupScript) {
        // Regular expression to match '${workspaceFolder}'
        const regex = "\$\{workspaceFolder\}";
        if (rosSetupScript.includes(regex)) {
            if ((vscode.workspace.workspaceFolders?.length ?? 0) === 1) {
                // Replace all occurrences of '${workspaceFolder}' with the workspace string
                rosSetupScript = rosSetupScript.replace(regex, vscode.workspace.workspaceFolders![0].uri.fsPath);
            } else {
                outputChannel.appendLine(`Multiple or no workspaces found, but the ROS setup script setting \"ROS2.rosSetupScript\" is configured with '${rosSetupScript}'`);
            }
        }

        // Try to support cases where the setup script doesn't make sense on different environments, such as host vs container.
        if (await exists(rosSetupScript)) {
            try {
                newEnv = await sourceUnderlay(rosSetupScript);

                outputChannel.appendLine(`Sourced ${rosSetupScript}`);

                attemptWorkspaceDiscovery = false;
            } catch (err) {
                reportFailure(rosSetupScript, err);
                if (forBuild) { throw err; }
                vscode.window.setStatusBarMessage(`Could not source "${rosSetupScript}". See Output > ROS 2. Attempting discovery.`, kWorkspaceConfigTimeout);
            }
        } else {
            if (forBuild) { throw new Error(`Configured ROS underlay is missing or inaccessible: ${rosSetupScript}. Select a working ROS 2 installation.`); }
            outputChannel.appendLine(`Configured ROS setup script is missing or inaccessible: ${rosSetupScript}. Attempting discovery.`);
        }
    }

    if (attemptWorkspaceDiscovery) {
        const configuredDistro = config.get("distro", "");
        outputChannel.appendLine("Discovering installed ROS 2 setup scripts (cached Pixi installations first).");
        const installedDistros = await detectInstalledDistros();
        const distro = selectInstalledDistro(installedDistros, configuredDistro, process.env.ROS_DISTRO);

        if (configuredDistro && process.env.ROS_DISTRO && process.env.ROS_DISTRO !== configuredDistro) {
            outputChannel.appendLine(`Ignoring ROS_DISTRO (${process.env.ROS_DISTRO}); using configured distro (${configuredDistro}).`);
        }

        if (distro) {
            const setupScript = distro.setupScript;
            try {
                outputChannel.appendLine(`Sourcing ROS Distro: ${setupScript}`);
                newEnv = await sourceUnderlay(setupScript);
            } catch (err) {
                reportFailure(setupScript, err);
                if (forBuild) { throw err; }
                vscode.window.setStatusBarMessage(`Could not source "${setupScript}". See Output > ROS 2 for the cause.`, kWorkspaceConfigTimeout);
            }
        } else {
            const requestedDistro = configuredDistro || process.env.ROS_DISTRO;
            const message = requestedDistro
                ? `No ROS 2 setup script found for "${requestedDistro}". Use ROS2: Find ROS or select an installed distribution.`
                : installedDistros.length
                    ? "Multiple ROS 2 distros found. Select an installed distribution or configure ROS2.distro."
                    : "No ROS 2 setup scripts found. Use ROS2: Find ROS or install a ROS 2 distribution.";
            outputChannel.appendLine(message);
            if (forBuild) { throw new Error(message); }
            await vscode.window.setStatusBarMessage(message, kWorkspaceConfigTimeout);
        }
    }

    // Build preparation deliberately never executes the current install overlay.
    // Keep the runtime/debug sourcing and its error reporting below unchanged.
    if (forBuild) { return newEnv; }

    let workspaceOverlayPath: string = "";
    // Source the workspace setup over the top.

    if (newEnv && (newEnv as Record<string, string>).ROS_VERSION === "1") {
        outputChannel.appendLine(`RDE ROS 2 does not support ROS 1`);
    } else if (newEnv && vscode.workspace.rootPath) {    // FUTURE: Revisit if ROS_VERSION changes - not clear it will be called 3
        if (!await exists(workspaceOverlayPath)) {
            workspaceOverlayPath = path.join(`${vscode.workspace.rootPath}`, "install");
        }
    }

    let wsSetupScript: string = path.format({
        dir: workspaceOverlayPath,
        name: "setup",
        ext: ros_utils.getSetupScriptExtension(),
    });

    if (workspaceOverlayPath && await exists(wsSetupScript)) {
        outputChannel.appendLine(`Workspace overlay path: ${wsSetupScript}`);

        try {
            newEnv = await ros_utils.sourceSetupFile(wsSetupScript, newEnv);
        } catch (err) {
            reportFailure(wsSetupScript, err);
            vscode.window.showErrorMessage("Failed to source the workspace setup file. See Output > ROS 2 for the cause.");
        }
    } else if (workspaceOverlayPath) {
        outputChannel.appendLine(`Not sourcing workspace does not exist yet: ${wsSetupScript}. Need to build workspace.`);
    }

    env = newEnv;
    onEnvResolved.fire();

    if (notifyEnvironmentChange) {
        // Notify listeners only when a full environment-dependent extension refresh is required.
        onEnvChanged.fire();
    }
    return newEnv;
}

/** Fresh sourcing and read-only CLI checks, never reuse a stale successful activation. */
export async function prepareRosBuildEnvironment(baseEnv: NodeJS.ProcessEnv, options: RosBuildOptions = {}): Promise<NodeJS.ProcessEnv> {
    const prefixes = buildInstallPrefixes(vscode.workspace.rootPath, options);
    const log = options.onOutput ?? (message => outputChannel.appendLine(message));
    const clean = cleanBuildEnvironment(baseEnv, prefixes);
    // ROS identity must be supplied by the selected underlay, not inherited flags.
    for (const key of Object.keys(clean)) {
        if (/^ROS_(VERSION|DISTRO)$/i.test(key)) { delete clean[key]; }
    }
    let selectedScript = "";
    let sourced = await sourceRosAndWorkspace(false, clean, true, prefixes, script => { selectedScript = script; });
    if (sourced?.ROS_VERSION !== "2" || !sourced.ROS_DISTRO) {
        throw new Error("ROS setup did not provide ROS_VERSION=2 and ROS_DISTRO. Select a working ROS 2 installation.");
    }
    const rejectSelfOverlay = (current: NodeJS.ProcessEnv, script: string): void => {
        if (Object.values(current).some(value => value?.split(";").some(entry => isWorkspaceInstall(entry, prefixes)))) {
            throw new Error(`ROS underlay ${script} reintroduced the current workspace install. Select an external underlay that does not source this workspace; no partially sourced environment will be used.`);
        }
    };
    rejectSelfOverlay(sourced, selectedScript);
    const selectedDistro = sourced.ROS_DISTRO;
    if (process.platform === "win32") {
        for (const script of await buildParentScripts(prefixes)) {
            if (sameBuildScript(script, selectedScript)) { continue; }
            log(`Sourcing recorded external build underlay: ${script}`);
            // Missing/broken external parents remain fatal; never treat them as self overlays.
            sourced = await ros_utils.sourceSetupFile(script, { ...sourced }, true);
            rejectSelfOverlay(sourced, script);
            if (sourced.ROS_VERSION !== "2" || sourced.ROS_DISTRO !== selectedDistro) {
                throw new Error(`External underlay ${script} changed the selected ROS distro (${selectedDistro}). Use compatible external dependencies; no build was started.`);
            }
        }
    }
    sourced = cleanBuildEnvironment(sourced, prefixes);
    log(`[Build recovery] Skipping current-workspace install overlay: ${prefixes.join(", ")}. It may be absent or incomplete. Using fresh ROS/external underlays; colcon will source individual workspace dependencies. Rebuild to regenerate setup hooks; runtime/debug setup errors are not ignored.`);
    if (options.args?.some(arg => arg === "--packages-select" || arg.startsWith("--packages-select="))) {
        log("[Build recovery] Keeping --packages-select unchanged. Installed workspace dependencies are loaded by colcon per package; if a dependency is missing/incomplete, rerun with --packages-up-to <package>. Explicit skip/ignore filters are still honored.");
    }
    const execFile = promisify(child_process.execFile);
    for (const tool of ["ros2", "colcon"]) {
        const command = process.platform === "win32" ? `${tool}.exe` : tool;
        try {
            await execFile(command, ["--help"], {
                env: sourced, cwd: options.cwd ?? vscode.workspace.rootPath, timeout: 30000,
                maxBuffer: 1024 * 1024, windowsHide: true,
            });
        } catch (error) {
            throw new Error(`ROS build preflight failed running ${command} --help: ${error instanceof Error ? error.message : String(error)}`);
        }
    }
    return sourced;
}

/** A recovery build environment is not a runtime environment. Load the repaired
 * local overlay strictly before executing/debugging a newly built test.
 */
export async function prepareRosTestEnvironment(baseEnv: NodeJS.ProcessEnv, workspace: string): Promise<NodeJS.ProcessEnv> {
    const script = path.join(workspace, "install", `local_setup${ros_utils.getSetupScriptExtension()}`);
    return ros_utils.sourceSetupFile(script, { ...baseEnv }, true);
}
