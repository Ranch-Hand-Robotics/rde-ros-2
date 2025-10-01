// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as child_process from "child_process";
import * as os from "os";
import * as port_finder from "portfinder";
import * as util from "util";

import * as extension from "../../../extension";

import * as process_picker from "../../process-picker/process-picker";
import * as picker_items_provider_factory from "../../process-picker/process-items-provider";
import * as requests from "../../requests";
import * as utils from "../../utils";

import * as vscode_utils from "../../../vscode-utils";

const promisifiedExec = util.promisify(child_process.exec);

// Reuse the same terminal for elevated commands
let elevatedCommandTerminal: vscode.Terminal | undefined;

/**
 * Executes a command that requires elevated privileges using VS Code terminal
 */
async function executeElevatedCommand(command: string, processOptions: child_process.ExecOptions): Promise<{ stdout: string; stderr: string }> {
    if (os.platform() === "win32") {
        // On Windows, try without elevation first, then provide helpful error message
        try {
            return await promisifiedExec(command, processOptions);
        } catch (error) {
            const errorMessage = `Command requires administrator privileges. Please run VS Code as administrator or use the terminal method.\n\nCommand: ${command}`;
            throw new Error(errorMessage);
        }
    } else {
        // On Linux/macOS, use sudo in terminal - simple and reliable
        const sudoCommand = `sudo ${command}`;
        
        // Reuse existing terminal or create a new one
        if (!elevatedCommandTerminal || elevatedCommandTerminal.exitStatus !== undefined) {
            elevatedCommandTerminal = vscode.window.createTerminal({
                name: 'ROS Debug - Elevated Commands',
                hideFromUser: false
            });
        }
        
        elevatedCommandTerminal.show();
        elevatedCommandTerminal.sendText(sudoCommand);
        
        // Show a message to the user about what's happening
        const message = `Running elevated command in terminal. Please enter your password if prompted.\n\nCommand: ${sudoCommand}`;
        const result = await vscode.window.showInformationMessage(
            message,
            'Continue',
            'Cancel'
        );
        
        if (result === 'Cancel') {
            throw new Error('User cancelled elevated command execution');
        }
        
        // Return success indicator - the actual command execution happens in the terminal
        return { stdout: 'Command sent to terminal', stderr: '' };
    }
}

export interface IResolvedAttachRequest extends requests.IAttachRequest {
    runtime: string;
    processId: number;
    commandLine: string;
}

export class AttachResolver implements vscode.DebugConfigurationProvider {
    private readonly supportedRuntimeTypes = [
        "C++",
        "Python",
    ];

    public async resolveDebugConfigurationWithSubstitutedVariables(folder: vscode.WorkspaceFolder | undefined, config: requests.IAttachRequest, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration> {
        // ${command:} variables only get resolved before passed to debug adapter, need to be manually resolve here
        // all ${action:} variables need to be resolved before our resolver propagates the configuration to actual debugger

        await this.resolveRuntimeIfNeeded(this.supportedRuntimeTypes, config);
        await this.resolveProcessIdIfNeeded(config);
        await this.resolveCommandLineIfNeeded(config);

        // propagate debug configuration to Python or C++ debugger depending on the chosen runtime type
        this.launchAttachSession(config as IResolvedAttachRequest);

        // Return null as we have spawned new debug session
        return null;
    }

    private async launchAttachSession(config: IResolvedAttachRequest) {
        if (!config.runtime || !config.processId) {
            return;
        }

        let debugConfig: ICppvsdbgAttachConfiguration | ICppdbgAttachConfiguration | IPythonAttachConfiguration | any;
        if (config.runtime === "C++") {
            const isLldbInstalled = vscode_utils.isLldbExtensionInstalled();
            if (isLldbInstalled) {
                const lldbAttachConfig: any = {
                    name: `C++: ${config.processId}`,
                    type: "lldb",
                    request: "attach",
                    processId: config.processId,
                };
                debugConfig = lldbAttachConfig;
            } else if (os.platform() === "win32") {
                const cppvsdbgAttachConfig: ICppvsdbgAttachConfiguration = {
                    name: `C++: ${config.processId}`,
                    type: "cppvsdbg",
                    request: "attach",
                    processId: config.processId,
                };
                debugConfig = cppvsdbgAttachConfig;
            } else {
                const cppdbgAttachConfig: ICppdbgAttachConfiguration = {
                    name: `C++: ${config.processId}`,
                    type: "cppdbg",
                    request: "attach",
                    program: config.commandLine,
                    processId: config.processId,
                    setupCommands: [
                        {
                            text: "-enable-pretty-printing",
                            description: "Enable pretty-printing for gdb",
                            ignoreFailures: true
                        }
                    ]
                };
                debugConfig = cppdbgAttachConfig;
            }

        } else if (config.runtime === "Python") {
            const host = "localhost";
            const port = await port_finder.getPortPromise();
            const ptvsdInjectCommand = await utils.getPtvsdInjectCommand(host, port, config.processId);
            
            // Log the command being executed for debugging
            extension.outputChannel.appendLine(`Attempting to inject Python debugger into process ${config.processId}`);
            extension.outputChannel.appendLine(`Command: ${ptvsdInjectCommand}`);
            extension.outputChannel.appendLine(`Target process: ${config.processId} (${config.commandLine || 'unknown command'})`);
            
            try {
                const processOptions: child_process.ExecOptions = {
                    cwd: vscode.workspace.rootPath,
                    env: await extension.resolvedEnv(),
                };

                if (os.platform() === "win32") {
                    // "ptvsd --pid" works with child_process.exec() on Windows
                    const result = await promisifiedExec(ptvsdInjectCommand, processOptions);
                } else {
                    // "ptvsd --pid" requires elevated permission on Linux/macOS
                    // Use our VS Code-friendly elevated command execution
                    const result = await executeElevatedCommand(ptvsdInjectCommand, processOptions);
                }

            } catch (error) {
                const errorMsg = `Python debugger injection failed: ${error.message}`;
                
                // Provide helpful troubleshooting information
                const troubleshootMsg = `Python debugger injection failed. This could be due to:\n\n` +
                    `1. The process is not a Python process\n` +
                    `2. The Python extension needs to be updated\n` +
                    `3. The debugger is already attached\n` +
                    `4. Permission issues\n\n` +
                    `Note: GDB warnings (like ".gnu_debugaltlink") are normal - debugpy uses GDB internally.\n` +
                    `Look for actual error messages, not GDB warnings.\n\n` +
                    `Command that failed: ${ptvsdInjectCommand}\n\n` +
                    `Please check the Output panel for more details.`;
                extension.outputChannel.appendLine(troubleshootMsg);
                extension.outputChannel.show();
                    
                vscode.window.showErrorMessage(errorMsg, 'Open Output').then(selection => {
                    if (selection === 'Open Output') {
                        extension.outputChannel.show();
                    }
                });
                
                throw new Error(errorMsg);
            }

            let statusMsg = `Python debugger injection completed for process [${config.processId}].\n`;
            statusMsg += `Attempting to connect to debug server at ${host}:${port}\n\n`;
            extension.outputChannel.appendLine(statusMsg);
            extension.outputChannel.show(true);
            vscode.window.showInformationMessage(`Connecting to Python debugger at ${host}:${port}...`);

            const pythonattachdebugconfiguration: IPythonAttachConfiguration = {
                name: `Python: ${config.processId}`,
                type: "python",
                request: "attach",
                port: port,
                host: host,
            };
            
            extension.outputChannel.appendLine(`Created Python attach configuration:`);
            extension.outputChannel.appendLine(`  Name: ${pythonattachdebugconfiguration.name}`);
            extension.outputChannel.appendLine(`  Type: ${pythonattachdebugconfiguration.type}`);
            extension.outputChannel.appendLine(`  Host: ${pythonattachdebugconfiguration.host}`);
            extension.outputChannel.appendLine(`  Port: ${pythonattachdebugconfiguration.port}`);
            
            debugConfig = pythonattachdebugconfiguration;
        }

        if (!debugConfig) {
            extension.outputChannel.appendLine(`Error: No debug configuration created!`);
            return;
        }
        
        extension.outputChannel.appendLine(`Starting debug session with configuration: ${debugConfig.name}`);
        
        // For Python debugging, add a small delay to let the debug server start
        if (debugConfig.type === 'python') {
            extension.outputChannel.appendLine(`Waiting 2 seconds for Python debug server to start...`);
            await new Promise(resolve => setTimeout(resolve, 2000));
            
            // Try to test if the port is accessible (basic connection test)
            try {
                const net = require('net');
                const socket = new net.Socket();
                const connected = await new Promise((resolve) => {
                    socket.setTimeout(1000);
                    socket.on('connect', () => {
                        socket.destroy();
                        resolve(true);
                    });
                    socket.on('timeout', () => {
                        socket.destroy();
                        resolve(false);
                    });
                    socket.on('error', () => {
                        socket.destroy();
                        resolve(false);
                    });
                    socket.connect(debugConfig.port, debugConfig.host || 'localhost');
                });
                
                if (connected) {
                    extension.outputChannel.appendLine(`✅ Debug server is responding on ${debugConfig.host}:${debugConfig.port}`);
                } else {
                    extension.outputChannel.appendLine(`⚠️ Debug server not responding on ${debugConfig.host}:${debugConfig.port}`);
                    extension.outputChannel.appendLine(`This might indicate the debugger injection failed or the process exited.`);
                }
            } catch (netError) {
                extension.outputChannel.appendLine(`Connection test failed: ${netError.message}`);
            }
        }
        
        try {
            const launched = await vscode.debug.startDebugging(undefined, debugConfig);
            if (!launched) {
                const errorMsg = `Failed to start debug session for ${debugConfig.name}. This could be due to:\n` +
                                `1. Debug server not responding on ${debugConfig.host || 'localhost'}:${debugConfig.port || 'unknown'}\n` +
                                `2. Python extension issues\n` +
                                `3. The injected debugger process exited\n` +
                                `4. Firewall blocking the connection\n\n` +
                                `Check the terminal for debugger injection results.`;
                extension.outputChannel.appendLine(errorMsg);
                extension.outputChannel.show();
                vscode.window.showErrorMessage(`Debug session failed to start`, 'Open Output').then(selection => {
                    if (selection === 'Open Output') {
                        extension.outputChannel.show();
                    }
                });
                throw new Error(`Failed to start debug session!`);
            } else {
                extension.outputChannel.appendLine(`✅ Debug session started successfully: ${debugConfig.name}`);
                vscode.window.showInformationMessage(`Python debugger attached successfully!`);
            }
        } catch (error) {
            extension.outputChannel.appendLine(`Error starting debug session: ${error.message}`);
            extension.outputChannel.show();
            throw error;
        }
    }

    private async resolveRuntimeIfNeeded(supportedRuntimeTypes: string[], config: requests.IAttachRequest) {
        if (config.runtime && config.runtime !== "${action:pick}") {
            return;
        }

        const chooseRuntimeOptions: vscode.QuickPickOptions = {
            placeHolder: "Choose runtime type of node to attach to.",
        };
        config.runtime = await vscode.window.showQuickPick(supportedRuntimeTypes, chooseRuntimeOptions).then((runtime): string => {
            if (!runtime) {
                throw new Error("Runtime type not chosen!");
            }
            return runtime;
        });
    }

    private async resolveProcessIdIfNeeded(config: requests.IAttachRequest) {
        if (config.processId && config.processId !== "${action:pick}") {
            return;
        }

        const processItemsProvider = picker_items_provider_factory.LocalProcessItemsProviderFactory.Get();
        const processPicker = new process_picker.LocalProcessPicker(processItemsProvider);
        const process = await processPicker.pick();
        config.processId = process.pid;
    }

    private async resolveCommandLineIfNeeded(config: requests.IAttachRequest) {
        // this step is only needed on Ubuntu when user has specified PID of C++ executable to attach to
        if (os.platform() === "win32" || config.commandLine || config.runtime !== "C++") {
            return;
        }

        if (!config.processId) {
            throw (new Error("No PID specified!"));
        }
        try {
            const result = await promisifiedExec(`ls -l /proc/${config.processId}/exe`);

            // contains a space
            const searchTerm = "-> ";
            const indexOfFirst = result.stdout.indexOf(searchTerm);
            config.commandLine = result.stdout.substring(indexOfFirst + searchTerm.length).trim();
        } catch (error) {
            throw (new Error(`Failed to resolve command line for process [${config.processId}]!`));
        }
    }
}
