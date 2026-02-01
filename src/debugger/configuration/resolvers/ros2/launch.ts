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
import * as rust_utils from "../../../../ros/rust-utils";

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
    pathMappings?: Array<{ localRoot: string; remoteRoot: string; }>;
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
 * Finds all package.xml files in a workspace recursively.
 * 
 * @param workspaceRoot Root directory of the workspace
 * @returns Array of package.xml file paths
 */
function findPackageXmlFiles(workspaceRoot: string): string[] {
    const packageXmlFiles: string[] = [];
    
    function searchDirectory(dirPath: string) {
        try {
            const entries = fs.readdirSync(dirPath, { withFileTypes: true });
            
            for (const entry of entries) {
                const fullPath = path.join(dirPath, entry.name);
                
                if (entry.isDirectory()) {
                    // Skip common directories that won't contain packages
                    if (['build', 'install', 'log', '.git', 'node_modules', '__pycache__'].includes(entry.name)) {
                        continue;
                    }
                    searchDirectory(fullPath);
                } else if (entry.isFile() && entry.name === 'package.xml') {
                    packageXmlFiles.push(fullPath);
                }
            }
        } catch (error) {
            // Ignore directories we can't read
        }
    }
    
    searchDirectory(workspaceRoot);
    return packageXmlFiles;
}

/**
 * Parses a package.xml file to extract package information.
 * 
 * @param packageXmlPath Path to the package.xml file
 * @returns Package information or null if parsing fails
 */
function parsePackageXml(packageXmlPath: string): { name: string; packageRoot: string } | null {
    try {
        const xmlContent = fs.readFileSync(packageXmlPath, 'utf8');
        const packageRoot = path.dirname(packageXmlPath);
        
        // Simple XML parsing to extract package name
        const nameMatch = xmlContent.match(/<name>\s*([^<]+)\s*<\/name>/);
        if (nameMatch && nameMatch[1]) {
            return {
                name: nameMatch[1].trim(),
                packageRoot: packageRoot
            };
        }
    } catch (error) {
        extension.outputChannel.appendLine(`Error parsing package.xml at ${packageXmlPath}: ${String(error)}`);
    }
    
    return null;
}

/**
 * Recursively searches for Python files in a directory.
 * 
 * @param dirPath Directory to search
 * @param fileName Name of the Python file to find (without .py extension)
 * @returns Array of matching file paths
 */
function findPythonFiles(dirPath: string, fileName: string): string[] {
    const pythonFiles: string[] = [];
    
    function searchDirectory(currentPath: string) {
        try {
            const entries = fs.readdirSync(currentPath, { withFileTypes: true });
            
            for (const entry of entries) {
                const fullPath = path.join(currentPath, entry.name);
                
                if (entry.isDirectory()) {
                    // Skip common build/cache directories
                    if (['__pycache__', '.git', 'build', 'install', 'log'].includes(entry.name)) {
                        continue;
                    }
                    searchDirectory(fullPath);
                } else if (entry.isFile() && entry.name === `${fileName}.py`) {
                    pythonFiles.push(fullPath);
                }
            }
        } catch (error) {
            // Ignore directories we can't read
        }
    }
    
    searchDirectory(dirPath);
    return pythonFiles;
}

/**
 * Attempts to find the source Python file corresponding to an installed executable.
 * Returns the source file path and path mappings for debugging.
 * 
 * @param executablePath Path to the installed executable
 * @returns Object containing source file path and path mappings, or null if not found
 */
function findSourcePythonFile(executablePath: string): { sourcePath: string; pathMappings: Array<{ localRoot: string; remoteRoot: string; }>; } | null {
    try {
        // Parse the executable path to understand the structure
        // Example executable: /path/to/workspace/install/package_name/lib/package_name/node_name
        const parsedPath = path.parse(executablePath);
        const pathParts = parsedPath.dir.split(path.sep);
        
        // Find the 'install' directory in the path
        const installIndex = pathParts.lastIndexOf('install');
        if (installIndex === -1) {
            extension.outputChannel.appendLine(`No 'install' directory found in path: ${executablePath}`);
            return null;
        }
        
        // Find the package name (should be the directory after 'install')
        if (installIndex + 1 >= pathParts.length) {
            extension.outputChannel.appendLine(`Cannot determine package name from path: ${executablePath}`);
            return null;
        }
        
        const packageName = pathParts[installIndex + 1];
        const workspaceRoot = pathParts.slice(0, installIndex).join(path.sep);
        const nodeName = parsedPath.name;
        
        extension.outputChannel.appendLine(`Searching for source file: package=${packageName}, node=${nodeName}, workspace=${workspaceRoot}`);
        
        // Find all package.xml files in the workspace
        const packageXmlFiles = findPackageXmlFiles(workspaceRoot);
        extension.outputChannel.appendLine(`Found ${packageXmlFiles.length} package.xml files in workspace`);
        
        // Find the package.xml that matches our package name
        let targetPackageRoot: string | null = null;
        for (const packageXmlPath of packageXmlFiles) {
            const packageInfo = parsePackageXml(packageXmlPath);
            if (packageInfo && packageInfo.name === packageName) {
                targetPackageRoot = packageInfo.packageRoot;
                extension.outputChannel.appendLine(`Found matching package.xml: ${packageXmlPath}`);
                break;
            }
        }
        
        if (!targetPackageRoot) {
            extension.outputChannel.appendLine(`Could not find package.xml for package: ${packageName}`);
            return null;
        }
        
        // Search for Python files with the node name within the package directory
        const pythonFiles = findPythonFiles(targetPackageRoot, nodeName);
        extension.outputChannel.appendLine(`Found ${pythonFiles.length} potential Python files: ${pythonFiles.join(', ')}`);
        
        if (pythonFiles.length === 0) {
            extension.outputChannel.appendLine(`No Python file found for node: ${nodeName} in package: ${packageName}`);
            return null;
        }
        
        // Use the first matching Python file (could be enhanced with better selection logic)
        const sourcePath = pythonFiles[0];
        if (pythonFiles.length > 1) {
            extension.outputChannel.appendLine(`Multiple Python files found, using: ${sourcePath}`);
        }
        
        // Create path mappings to redirect from install to source
        const installPackageRoot = path.join(workspaceRoot, 'install', packageName);
        const sourcePackageDir = path.dirname(sourcePath);
        
        const pathMappings = [
            {
                localRoot: path.resolve(targetPackageRoot),
                remoteRoot: path.resolve(installPackageRoot)
            }
        ];
        
        // Add specific mapping for the Python module directory if different from package root
        if (sourcePackageDir !== targetPackageRoot) {
            const installLibDir = path.join(installPackageRoot, 'lib', packageName);
            pathMappings.push({
                localRoot: path.resolve(sourcePackageDir),
                remoteRoot: path.resolve(installLibDir)
            });
        }
        
        // Add additional mappings for site-packages (Windows case)
        if (executablePath.includes('site-packages')) {
            const sitePackagesRoot = path.dirname(executablePath);
            pathMappings.push({
                localRoot: path.resolve(sourcePackageDir),
                remoteRoot: path.resolve(sitePackagesRoot)
            });
        }
        
        extension.outputChannel.appendLine(`Found source file for debugging: ${sourcePath}`);
        extension.outputChannel.appendLine(`Package root: ${targetPackageRoot}`);
        extension.outputChannel.appendLine(`Install package root: ${installPackageRoot}`);
        extension.outputChannel.appendLine(`Path mappings: ${JSON.stringify(pathMappings, null, 2)}`);
        
        return {
            sourcePath: sourcePath,
            pathMappings: pathMappings
        };
        
    } catch (error) {
        extension.outputChannel.appendLine(`Error finding source file for ${executablePath}: ${String(error)}`);
    }
    
    return null;
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

        // Merge environment variables from envFile and env
        const mergedEnv = utils.mergeEnvFile(config.env, config.envFile, folder);
        
        const rosExecOptions: child_process.ExecOptions = {
            env: {
                ...await extension.resolvedEnv(),
                ...mergedEnv,
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

        // Remove LD_DEBUG environment variable for the launch dumper to prevent conflicts
        // but preserve them in the original environment for the actual processes
        const cleanedEnv = { ...rosExecOptions.env };
        if (cleanedEnv.LD_DEBUG) {
            delete cleanedEnv.LD_DEBUG;
        }
        if (cleanedEnv.LD_DEBUG_OUTPUT) {
            delete cleanedEnv.LD_DEBUG_OUTPUT;
        }

        // Use cleaned environment for the dumper
        const dumperExecOptions: child_process.ExecOptions = {
            ...rosExecOptions,
            env: cleanedEnv
        };

        let result = await promisifiedExec(ros2_launch_dumper_cmdLine, dumperExecOptions);

        if (result.stderr) {
            // Having stderr output is not necessarily a problem, but it is useful for debugging
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
        let programPath = request.executable;
        let pathMappings: Array<{ localRoot: string; remoteRoot: string; }> | undefined;
        
        // Check if Python source file debugging is enabled
        const useSourceFiles = vscode.workspace.getConfiguration('ROS2').get<boolean>('debugPythonUsingSourceFiles', false);
        
        if (useSourceFiles) {
            // Try to find the source Python file for better debugging experience
            const sourceInfo = findSourcePythonFile(request.executable);
            if (sourceInfo) {
                // Use the source file as the program to launch
                programPath = sourceInfo.sourcePath;
                
                extension.outputChannel.appendLine(`Python source file debugging enabled - found source file:`);
                extension.outputChannel.appendLine(`  Original executable: ${request.executable}`);
                extension.outputChannel.appendLine(`  Using source file: ${sourceInfo.sourcePath}`);
                
                // Only add path mappings if we think they're needed for complex scenarios
                // For now, let's try without them since we're launching the source directly
            } else {
                extension.outputChannel.appendLine(`Python source file debugging enabled, but no source file found for: ${request.executable}`);
                extension.outputChannel.appendLine(`Using installed executable for debugging.`);
            }
        }
        
        const pythonLaunchConfig: IPythonLaunchConfiguration = {
            name: request.nodeName,
            type: "python",
            request: "launch",
            program: programPath,
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

        const isCppToolsInstalled = vscode_utils.isCppToolsExtensionInstalled();
        const isLldbInstalled = vscode_utils.isLldbExtensionInstalled();
        const isCursor = vscode_utils.isCursorEditor();

        if (!isCppToolsInstalled && !isLldbInstalled) {
            let message: string;
            if (isCursor) {
                message = "LLDB is required for C++ debugging. Install the LLDB extension.";
            } else {
                message = "C++ debugging requires the Microsoft C/C++ extension (ms-vscode.cpptools) or LLDB extension.";
            }
            vscode.window.showErrorMessage(message);
            throw new Error(message);
        }
        if (isCppToolsInstalled) {
            if (os.platform() === "win32") {
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
        } else if (isLldbInstalled) {
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
        } else {
            throw new Error("No C++ debugger available");
        }
    }

    private createRustLaunchConfig(request: ILaunchRequest, stopOnEntry: boolean): ILldbLaunchConfiguration {
        // Rust debugging uses LLDB (CodeLLDB extension)
        const isLldbInstalled = vscode_utils.isRustDebuggerExtensionInstalled();
        
        if (!isLldbInstalled) {
            const message = "Rust debugging requires the CodeLLDB extension (vadimcn.vscode-lldb). Please install it from the marketplace.";
            vscode.window.showErrorMessage(message);
            throw new Error(message);
        }
        
        const rustLaunchConfig: ILldbLaunchConfiguration = {
            name: request.nodeName,
            type: "lldb",
            request: "launch",
            program: request.executable,
            args: request.arguments,
            cwd: ".",
            env: request.env,
            stopAtEntry: stopOnEntry,
            sourceLanguages: ["rust"]
        };
        
        return rustLaunchConfig;
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

                    // Check if it's a Rust executable
                    if (rust_utils.isRustExecutable(request.executable)) {
                        debugConfig = this.createRustLaunchConfig(request, stopOnEntry);
                    } else {
                        // C#? Todo

                        // Default to C++
                        debugConfig = this.createCppLaunchConfig(request, stopOnEntry);
                    }
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
                } else if (rust_utils.isRustExecutable(request.executable)) {
                    // Check if it's a Rust executable
                    debugConfig = this.createRustLaunchConfig(request, stopOnEntry);
                } else {
                    // Default to C++
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
