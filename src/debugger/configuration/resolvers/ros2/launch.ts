// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as fsp from "fs/promises";
import * as yaml from "js-yaml";
import * as os from "os";
import * as path from "path";
import * as readline from "readline";
import * as shell_quote from "shell-quote";
import * as tmp from "tmp";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../../../extension";
import * as vscode_utils from "../../../../vscode-utils";
import * as requests from "../../../requests";
import * as utils from "../../../utils";
import { rosApi } from "../../../../ros/ros";
import * as lifecycle from "../../../../ros/ros2/lifecycle";

const promisifiedExec = util.promisify(child_process.exec);



interface ILaunchRequest {
    nodeName: string;
    executable: string;
    arguments: string[];
    cwd: string;
    env: { [key: string]: string };
    symbolSearchPath?: string;
    additionalSOLibSearchPath?: string;
    sourceFileMap?: { [key: string]: string };
    launch?: string[];    // Scripts or executables to just launch without attaching a debugger
    attachDebugger?: string[];    // If specified, Scripts or executables to debug; otherwise attaches to everything not ignored
}

interface IJsonLaunchData {
    version: string;
    processes: IJsonProcess[];
    lifecycle_nodes: IJsonLifecycleNode[];
    warnings?: string[];
    errors?: string[];
    info?: string[];
}

interface IJsonProcess {
    type: string;
    command: string;
    node_name?: string;
    executable: string;
    arguments: string[];
}

interface IJsonLifecycleNode {
    type: string;
    node_name: string;
    namespace: string;
    package: string;
    executable: string;
    command?: string;  // Full command line (like ExecuteProcess)
    arguments?: string[];  // Command-line arguments (like ExecuteProcess)
    parameters: { [key: string]: any };  // ROS parameters (specific to lifecycle nodes)
}

interface ICppEnvConfig {
    name: string;
    value: string;
}

interface IPythonLaunchConfiguration {
    name: string;
    type: string;
    request: string;
    program: string;
    args: string[];
    env: { [key: string]: string };
    stopOnEntry: boolean;
    justMyCode: boolean;
}

interface ICppvsdbgLaunchConfiguration {
    name: string;
    type: string;
    request: string;
    cwd: string;
    program: string;
    args: string[];
    environment: ICppEnvConfig[];
    stopAtEntry: boolean;
    symbolSearchPath?: string;
    sourceFileMap?: { [key: string]: string };
}

interface ICppdbgLaunchConfiguration {
    name: string;
    type: string;
    request: string;
    cwd: string;
    program: string;
    args: string[];
    environment: ICppEnvConfig[];
    stopAtEntry: boolean;
    additionalSOLibSearchPath?: string;
    sourceFileMap?: { [key: string]: string };
    setupCommands: Array<{
        text: string;
        description: string;
        ignoreFailures: boolean;
    }>;
}

export interface ILldbLaunchConfiguration {
    type: "lldb";
    request: "launch" | "attach";
    name: string;
    cwd?: string;
    program?: string;
    args?: string[];
    env?:  { [key: string]: string };
    initCommands?: string[];
    targetCreateCommands?: string[];
    preRunCommands?: string[];
    processCreateCommands?: string[];
    postRunCommands?: string[];
    preTerminateCommands?: string[];
    exitCommands?: string[];
    expressions?: "simple" | "python" | "native";
    sourceMap?: { [key: string]: string; };
    relativePathBase?: string;
    breakpointMode?: "path" | "file";
    sourceLanguages?: string[]; 
    reverseDebugging?: boolean;
    stopAtEntry?: boolean;
    pid?: number;
}


function getExtensionFilePath(extensionFile: string): string {
    return path.resolve(extension.extPath, extensionFile);
}

/**
 * LaunchResolver for ROS 2 launch files with lifecycle node support.
 * 
 * Features:
 * - Cross-platform support (Windows and Linux)
 * - JSON and legacy output formats from dumper
 * - Lifecycle node detection and debugging
 * - Automatic ROS environment setup
 * - Robust error handling and logging
 */
export class LaunchResolver implements vscode.DebugConfigurationProvider {
    // tslint:disable-next-line: max-line-length
    public async resolveDebugConfigurationWithSubstitutedVariables(folder: vscode.WorkspaceFolder | undefined, config: requests.ILaunchRequest, token?: vscode.CancellationToken) {
        await fsp.access(config.target, fs.constants.R_OK);

        if (path.extname(config.target) !== ".py" && path.extname(config.target) !== ".xml" && path.extname(config.target) !== ".yaml") {
            throw new Error("Launch request requires an extension '.py', '.xml' or '.yaml'.");
        }

        const delay = ms => new Promise(res => setTimeout(res, ms));

        // Manage the status of the ROS2 Daemon, starting one if not present
        if (await rosApi.getCoreStatus() == false) {
            extension.outputChannel.appendLine("ROS Daemon is not active, attempting to start automatically");
            rosApi.startCore();

            // Wait for the core to start up to a timeout
            const timeout_ms: number = 30000;
            const interval_ms: number = 100;
            let timeWaited: number = 0;
            while (await rosApi.getCoreStatus() == false && 
                timeWaited < timeout_ms) {
                timeWaited += interval_ms;
                await delay(interval_ms);
            }

            extension.outputChannel.appendLine("Waited " + timeWaited + " for ROS 2 Daemon to start. Proceeding without the Daemon.");
        }

        const rosExecOptions: child_process.ExecOptions = {
            env: {
                ...await extension.resolvedEnv(),
                ...config.env,
            },
        };

        let ros2_launch_dumper = getExtensionFilePath(path.join("assets", "scripts", "ros2_launch_dumper.py"));

        let args = []
        if (config.arguments) {
            for (let arg of config.arguments) {
                args.push(`${arg}`);
            }
        }
        let flatten_args = args.join(' ')
        
        // Use the detected Python command for better virtual environment support
        let ros2_launch_dumper_cmdLine = `python3 ${ros2_launch_dumper} "${config.target}" ${flatten_args}`;

        let result = await promisifiedExec(ros2_launch_dumper_cmdLine, rosExecOptions);

        if (result.stderr) {
            // Having stderr output is not nessesarily a problem, but it is useful for debugging
            extension.outputChannel.appendLine(`ROS2 launch processor produced stderr output:\r\n ${result.stderr}`);
            // Show output channel when there's stderr output
            vscode_utils.showOutputPanel(extension.outputChannel);
        }        

        if (result.stdout.length == 0) {
            throw (new Error(`ROS2 launch processor was unable to produce a node list.\r\n ${result.stderr}`));
        }

        // Try to parse as JSON first, fall back to legacy format
        try {
            const launchData = JSON.parse(result.stdout);
            await this.processJsonLaunchData(launchData, config, rosExecOptions);
        } catch (jsonError) {
            extension.outputChannel.appendLine(`JSON parsing failed: ${jsonError.message}`);
            extension.outputChannel.appendLine(result.stdout);
            // Show output channel when JSON parsing fails
            vscode_utils.showOutputPanel(extension.outputChannel);
        }

        // @todo: error handling for Promise.all

        // Return null as we have spawned new debug requests
        return null;
    }

    private generateLaunchRequest(nodeName: string, command: string, config: requests.ILaunchRequest): ILaunchRequest {
        let parsedArgs: shell_quote.ParseEntry[];

        parsedArgs = shell_quote.parse(command);

        let executable = parsedArgs.shift().toString();

         // return rviz instead of rviz.exe, or spawner instead of spawner.py
         // This allows the user to run filter out genericly. 
        let executableName = path.basename(executable, path.extname(executable));

        // If this executable is just launched, don't attach a debugger.
        if (config.launch && 
            config.launch.indexOf(executableName) != -1) {
          return null;
        }

        // Filter shell scripts - just launch them
        //  https://github.com/ranchhandrobotics/rde-ros-2/issues/474 
        let executableExt = path.extname(executable);
        if (executableExt && 
            ["bash", "sh", "bat", "cmd", "ps1"].includes(executableExt)) {
          return null;
        }

        // If a specific list of nodes is specified, then determine if this is one of them.
        // If no specific nodes specifed, attach to all unless specifically ignored.
        if (config.attachDebugger == null ||
          config.attachDebugger.indexOf(executableName) != -1) {

          const envConfig: { [key: string]: string; } = config.env;

          const request: ILaunchRequest = {
              nodeName: nodeName,
              executable: executable,
              arguments: parsedArgs.map((arg) => {
                  return arg.toString();
              }),
              cwd: ".",
              env: {
                  ...extension.env,
                  ...envConfig,
              },
              symbolSearchPath: config.symbolSearchPath, 
              additionalSOLibSearchPath: config.additionalSOLibSearchPath, 
              sourceFileMap: config.sourceFileMap
          };

          return request;
        }

        return null;
    }

    private createPythonLaunchConfig(request: ILaunchRequest, stopOnEntry: boolean): IPythonLaunchConfiguration {
        const pythonLaunchConfig: IPythonLaunchConfiguration = {
            name: request.nodeName,
            type: "python",
            request: "launch",
            program: request.executable,
            args: request.arguments,
            env: request.env,
            stopOnEntry: stopOnEntry,
            justMyCode: false,
        };

        return pythonLaunchConfig;
    }

    private createCppLaunchConfig(request: ILaunchRequest, stopOnEntry: boolean): ICppvsdbgLaunchConfiguration | ICppdbgLaunchConfiguration | ILldbLaunchConfiguration {
        const envConfigs: ICppEnvConfig[] = [];
        for (const key in request.env) {
            if (request.env.hasOwnProperty(key)) {
                envConfigs.push({
                    name: key,
                    value: request.env[key],
                });
            }
        }

        const isCursor = vscode_utils.isRunningInCursor();
        if (isCursor) {
            const lldbLaunchConfig: ILldbLaunchConfiguration = {
                name: request.nodeName,
                type: "lldb",
                request: "launch",
                program: request.executable,
                args: request.arguments,
                cwd: ".",
                env: request.env,
                stopAtEntry: stopOnEntry
            };

            return lldbLaunchConfig;
        } else if (os.platform() === "win32") {
                const cppvsdbgLaunchConfig: ICppvsdbgLaunchConfiguration = {
                name: request.nodeName,
                type: "cppvsdbg",
                request: "launch",
                cwd: ".",
                program: request.executable,
                args: request.arguments,
                environment: envConfigs,
                stopAtEntry: stopOnEntry,
                symbolSearchPath: request.symbolSearchPath,
                sourceFileMap: request.sourceFileMap

            };

            return cppvsdbgLaunchConfig;
        } else {
            const cppdbgLaunchConfig: ICppdbgLaunchConfiguration = {
                name: request.nodeName,
                type: "cppdbg",
                request: "launch",
                cwd: ".",
                program: request.executable,
                args: request.arguments,
                environment: envConfigs,
                stopAtEntry: stopOnEntry,
                additionalSOLibSearchPath: request.additionalSOLibSearchPath,
                sourceFileMap: request.sourceFileMap,
                setupCommands: [
                    {
                        text: "-enable-pretty-printing",
                        description: "Enable pretty-printing for gdb",
                        ignoreFailures: true
                    }
                ]
            };
            
            return cppdbgLaunchConfig;
        }
    }

    private async executeLaunchRequest(request: ILaunchRequest, stopOnEntry: boolean) {
        let debugConfig: ICppvsdbgLaunchConfiguration | ICppdbgLaunchConfiguration | IPythonLaunchConfiguration | ILldbLaunchConfiguration;

        if (os.platform() === "win32") {
            let nodePath = path.parse(request.executable);

            if (nodePath.ext.toLowerCase() === ".exe") {

                // On Windows, colcon will compile Python scripts, C# and Rust programs to .exe. 
                // Discriminate between different runtimes by introspection.

                // Python
                // rosnode.py will be compiled into install\rosnode\Lib\rosnode\rosnode.exe
                // rosnode.py will also be copied into install\rosnode\Lib\site-packages\rosnode.py

                let pythonPath = path.join(nodePath.dir, "..", "site-packages", nodePath.name, nodePath.name + ".py");

                try {
                    await fsp.access(pythonPath, fs.constants.R_OK);

                    // If the python file is available, then treat it as python and fall through.
                    request.executable = pythonPath;
                    debugConfig = this.createPythonLaunchConfig(request, stopOnEntry);
                } catch {
                    // The python file is not available then this must be...

                    // C#? Todo

                    // Rust? Todo

                    // C++
                    debugConfig = this.createCppLaunchConfig(request, stopOnEntry);
                }
            } else if (nodePath.ext.toLowerCase() === ".py") {
                debugConfig = this.createPythonLaunchConfig(request, stopOnEntry);
            }

            if (!debugConfig) {
                throw (new Error(`Failed to create a debug configuration!`));
            }
            const launched = await vscode.debug.startDebugging(undefined, debugConfig);
            if (!launched) {
                throw (new Error(`Failed to start debug session!`));
            }
        } else {
            // this should be guaranteed by roslaunch
            await fsp.access(request.executable, fs.constants.X_OK | fs.constants.R_OK);

            const fileStream = fs.createReadStream(request.executable);
            const rl = readline.createInterface({
                input: fileStream,
                crlfDelay: Infinity,
            });

            // we only want to read 1 line to check for shebang line
            let linesToRead: number = 1;
            rl.on("line", async (line) => {
                if (linesToRead <= 0) {
                    return;
                }
                linesToRead--;
                if (!linesToRead) {
                    rl.close();
                }

                // look for Python in shebang line
                if (line.startsWith("#!") && line.toLowerCase().indexOf("python") !== -1) {
                    debugConfig = this.createPythonLaunchConfig(request, stopOnEntry);
                } else {
                    debugConfig = this.createCppLaunchConfig(request, stopOnEntry);
                }

                if (!debugConfig) {
                    throw (new Error(`Failed to create a debug configuration!`));
                }
                const launched = await vscode.debug.startDebugging(undefined, debugConfig);
                if (!launched) {
                    throw (new Error(`Failed to start debug session!`));
                }
            });
        }
    }

    private async processJsonLaunchData(launchData: IJsonLaunchData, config: requests.ILaunchRequest, rosExecOptions: child_process.ExecOptions): Promise<void> {
        const launchPromises: Promise<void>[] = [];

        // Output info messages from the launch dumper
        if (launchData.info && launchData.info.length > 0) {
            extension.outputChannel.appendLine("Launch dumper info:");
            for (const info of launchData.info) {
                extension.outputChannel.appendLine(`  Info: ${info}`);
            }
        }

        // Output warnings and errors from the launch dumper
        if (launchData.warnings && launchData.warnings.length > 0) {
            extension.outputChannel.appendLine("Launch dumper warnings:");
            for (const warning of launchData.warnings) {
                extension.outputChannel.appendLine(`  Warning: ${warning}`);
            }
        }

        if (launchData.errors && launchData.errors.length > 0) {
            extension.outputChannel.appendLine("Launch dumper errors:");
            for (const error of launchData.errors) {
                extension.outputChannel.appendLine(`  Error: ${error}`);
            }
            // Show output channel when there are errors
            vscode_utils.showOutputPanel(extension.outputChannel);
        }

        // Process regular processes
        for (const process of launchData.processes) {
            const launchRequest = this.generateLaunchRequest(
                process.node_name || path.basename(process.executable),
                process.command,
                config
            );

            if (launchRequest) {
                launchPromises.push(this.executeLaunchRequest(launchRequest, config.stopOnEntry || false));
            }
        }

        // Process lifecycle nodes
        for (const lifecycleNode of launchData.lifecycle_nodes) {
            // Use the command from dumper if available (includes proper path resolution),
            // otherwise fall back to basic executable with node name override
            const command = lifecycleNode.command || 
                           `${lifecycleNode.executable} --ros-args -r __node:=${lifecycleNode.node_name}`;
            const launchRequest = this.generateLaunchRequest(lifecycleNode.node_name, command, config);

            if (launchRequest) {
                if (lifecycleNode.namespace) {
                    launchRequest.env.ROS_NAMESPACE = lifecycleNode.namespace;
                }
                launchPromises.push(this.executeLaunchRequest(launchRequest, config.stopOnEntry || false));
            }
        }

        // Wait for all debuggers to start
        await Promise.all(launchPromises);
    }

    private async processLegacyLaunchData(stdout: string, config: requests.ILaunchRequest, rosExecOptions: child_process.ExecOptions): Promise<void> {
        const launchPromises: Promise<void>[] = [];
        const lines = stdout.split('\n');

        for (const line of lines) {
            if (line.trim().length === 0) {
                continue;
            }

            // Legacy format: tab-prefixed command
            if (line.startsWith('\t')) {
                const command = line.substring(1); // Remove tab
                const launchRequest = this.generateLaunchRequest(
                    path.basename(command.split(' ')[0]),
                    command,
                    config
                );

                if (launchRequest) {
                    launchPromises.push(this.executeLaunchRequest(launchRequest, config.stopOnEntry || false));
                }
            }
        }

        await Promise.all(launchPromises);
    }

    private getRosSetupCommand(env: { [key: string]: string } | undefined): string {
        // Try to detect ROS distribution from environment
        const rosDistro = env?.ROS_DISTRO;
        
        if (os.platform() === "win32") {
            // Windows: Use PowerShell or batch setup
            if (rosDistro) {
                const setupPath = `C:\\opt\\ros\\${rosDistro}\\setup.bat`;
                if (fs.existsSync(setupPath)) {
                    return `call "${setupPath}" && `;
                }
            }
            
            // Try common Windows ROS locations
            const commonDistros = ['jazzy', 'iron', 'humble', 'galactic', 'foxy'];
            for (const distro of commonDistros) {
                const setupPath = `C:\\opt\\ros\\${distro}\\setup.bat`;
                if (fs.existsSync(setupPath)) {
                    return `call "${setupPath}" && `;
                }
            }
            
            extension.outputChannel.appendLine("Warning: Could not detect ROS setup on Windows, assuming environment is already configured");
            // Show output channel when there's a ROS setup warning
            vscode_utils.showOutputPanel(extension.outputChannel);
            return '';
        } else {
            // Linux/Unix: Use bash setup scripts
            if (rosDistro) {
                const setupPath = `/opt/ros/${rosDistro}/setup.bash`;
                if (fs.existsSync(setupPath)) {
                    return `source "${setupPath}" && `;
                }
            }
            
            // Common ROS distributions (latest first)
            const commonDistros = ['jazzy', 'iron', 'humble', 'galactic', 'foxy'];
            for (const distro of commonDistros) {
                const setupPath = `/opt/ros/${distro}/setup.bash`;
                if (fs.existsSync(setupPath)) {
                    return `source "${setupPath}" && `;
                }
            }
            
            extension.outputChannel.appendLine("Warning: Could not detect ROS setup on Linux, assuming environment is already configured");
            return '';
        }
    }
}
