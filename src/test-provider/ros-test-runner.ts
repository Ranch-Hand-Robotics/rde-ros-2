// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as extension from "../extension";
import { rosApi } from "../ros/ros";
import * as ros_utils from "../ros/utils";
import * as vscode_utils from "../vscode-utils";
import { TestType, RosTestData } from "./ros-test-provider";
import { TestDiscoveryUtils } from "./test-discovery-utils";

/**
 * Handles running and debugging ROS 2 tests using existing launch mechanisms
 */
export class RosTestRunner {
    
    constructor(private context: vscode.ExtensionContext) {}
    
    /**
     * Run a test using the appropriate ROS 2 mechanism
     */
    async runTest(
        testData: RosTestData,
        debug: boolean = false,
        run?: vscode.TestRun,
        testItem?: vscode.TestItem
    ): Promise<void> {
        switch (testData.type) {
            case TestType.LaunchTest:
                return this.runLaunchTest(testData, debug, run, testItem);
            case TestType.PythonUnitTest:
            case TestType.PythonPytest:
                return this.runPythonTest(testData, debug, run, testItem);
            case TestType.CppGtest:
                return this.runCppTest(testData, debug, run, testItem);
            case TestType.Integration:
                return this.runIntegrationTest(testData, debug, run, testItem);
            default:
                throw new Error(`Unsupported test type: ${testData.type}`);
        }
    }
    
    /**
     * Run launch test using existing ROS 2 launch debugging infrastructure
     */
    private async runLaunchTest(
        testData: RosTestData,
        debug: boolean,
        run?: vscode.TestRun,
        testItem?: vscode.TestItem
    ): Promise<void> {
        if (!testData.launchFile) {
            throw new Error("Launch file not specified for launch test");
        }
        
        if (debug) {
            // Use existing ROS 2 debug configuration for launch files
            const debugConfig: vscode.DebugConfiguration = {
                name: `Debug ${path.basename(testData.launchFile)}`,
                type: 'ros2',
                request: 'launch',
                target: testData.launchFile,
                // Add test-specific environment variables to the ROS environment
                env: {
                    ...await extension.resolvedEnv(), // This provides the full ROS environment
                    ROS_TESTING: '1',
                    PYTEST_CURRENT_TEST: testData.testMethod || ''
                }
            };

            await vscode.debug.startDebugging(
                vscode.workspace.workspaceFolders?.[0],
                debugConfig
            );
            return Promise.resolve(); // Debug sessions don't have completion callbacks
        } else {
            // Use rostest command through rosApi for non-debug execution
            const terminal = rosApi.activateRostest(testData.launchFile, "");
            if (terminal) {
                terminal.show();
                
                // Monitor terminal output if we have a test run
                if (run && testItem) {
                    return this.monitorTerminalOutput(terminal, run, testItem);
                } else {
                    return Promise.resolve();
                }
            } else {
                return Promise.resolve();
            }
        }
    }
    
    /**
     * Run Python test using pytest or unittest
     */
    private async runPythonTest(
        testData: RosTestData,
        debug: boolean,
        run?: vscode.TestRun,
        testItem?: vscode.TestItem
    ): Promise<void> {
        const testCommand = TestDiscoveryUtils.getPythonTestCommand(testData);
        const env = await extension.resolvedEnv();
        
        if (debug) {
            // Create Python debug configuration with ROS environment
            const debugConfig: vscode.DebugConfiguration = {
                name: `Debug ${testData.testMethod || path.basename(testData.filePath)}`,
                type: 'python',
                request: 'launch',
                module: 'pytest',
                args: [
                    '-v', 
                    '-s', // Don't capture output for debugging
                    this.buildPythonTestSelector(testData)
                ],
                env: env, // This already contains the resolved ROS environment
                console: 'integratedTerminal',
                cwd: vscode.workspace.workspaceFolders?.[0].uri.fsPath
            };

            await vscode.debug.startDebugging(
                vscode.workspace.workspaceFolders?.[0],
                debugConfig
            );
            return Promise.resolve(); // Debug sessions don't have completion callbacks
        } else {
            // Use ROS task execution for better integration with VS Code task system
            const taskDefinition = {
                type: 'ROS2',
                command: 'python3',
                args: testCommand.slice(1) // Remove 'python3' from testCommand as it's the command
            };
            
            const task = new vscode.Task(
                taskDefinition,
                vscode.TaskScope.Workspace,
                `ROS 2 Test: ${testData.testMethod || path.basename(testData.filePath)}`,
                'ROS2'
            );
            
            task.execution = new vscode.ShellExecution('python3', taskDefinition.args, {
                env: env, // ROS environment
                cwd: vscode.workspace.workspaceFolders?.[0].uri.fsPath
            });
            
            // Execute the task
            vscode.tasks.executeTask(task);
            
            // Monitor task completion if we have a test run
            if (run && testItem) {
                return this.monitorTaskExecution(task, run, testItem);
            } else {
                return Promise.resolve();
            }
        }
    }
    
    /**
     * Run C++ test using ROS 2 testing infrastructure
     */
    private async runCppTest(
        testData: RosTestData,
        debug: boolean,
        run?: vscode.TestRun,
        testItem?: vscode.TestItem
    ): Promise<void> {
        if (!testData.packageName) {
            throw new Error("Package name not found for C++ test");
        }
        
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0].uri.fsPath;
        if (!workspaceRoot) {
            throw new Error("No workspace folder found");
        }
        
        const env = await extension.resolvedEnv();
        
        // First, build the test package
        try {
            await this.buildTestExecutable(testData.packageName, debug);
        } catch (buildError) {
            throw new Error(`Failed to build C++ test package ${testData.packageName}: ${buildError.message}`);
        }
        
        if (debug) {
            // For debugging, we still need to run the executable directly
            // Find the test executable with proper Windows support
            const executableName = TestDiscoveryUtils.getCppTestExecutable(testData.filePath, testData.packageName);
            if (!executableName) {
                throw new Error("Could not determine test executable name");
            }
            
            // Search for the test executable in the build directory
            const executablePath = this.findTestExecutable(workspaceRoot, testData.packageName, executableName);
            
            if (!executablePath) {
                throw new Error(`Test executable '${executableName}' not found in build directory for package '${testData.packageName}'. Build may have failed or executable name is incorrect.`);
            }
            
            // Create proper debug configuration following the existing ROS 2 debugger patterns
            const debugConfig = this.createCppDebugConfig(
                `Debug ${testData.testClass}.${testData.testMethod}`,
                executablePath,
                this.buildGTestArgs(testData),
                workspaceRoot,
                env,
                false // stopAtEntry
            );

            await vscode.debug.startDebugging(
                vscode.workspace.workspaceFolders?.[0],
                debugConfig
            );
        } else {
            // For C++ tests, run the executable directly since colcon test doesn't support
            // passing gtest filters through the -- separator
            const executableName = TestDiscoveryUtils.getCppTestExecutable(testData.filePath, testData.packageName);
            if (!executableName) {
                throw new Error("Could not determine test executable name");
            }
            
            const executablePath = this.findTestExecutable(workspaceRoot, testData.packageName, executableName);
            if (!executablePath) {
                throw new Error(`Test executable '${executableName}' not found in build directory for package '${testData.packageName}'. Build may have failed or executable name is incorrect.`);
            }
            
            // Build command line arguments for the test executable
            const testArgs = this.buildGTestArgs(testData);
            
            const taskDefinition = {
                type: 'ROS2',
                command: executablePath,
                args: testArgs
            };
            
            const task = new vscode.Task(
                taskDefinition,
                vscode.TaskScope.Workspace,
                `ROS 2 Test: ${testData.testClass}.${testData.testMethod}`,
                'ROS2'
            );
            
            task.execution = new vscode.ShellExecution(executablePath, testArgs, {
                env: env, // ROS environment
                cwd: workspaceRoot
            });
            
            // Execute the task
            vscode.tasks.executeTask(task);
            
            if (run && testItem) {
                return this.monitorTaskExecution(task, run, testItem);
            } else {
                return Promise.resolve();
            }
        }
    }
    
    /**
     * Run integration test (could be launch-based or custom)
     */
    private async runIntegrationTest(
        testData: RosTestData,
        debug: boolean,
        run?: vscode.TestRun,
        testItem?: vscode.TestItem
    ): Promise<void> {
        // Integration tests are typically launch-based
        if (testData.launchFile) {
            return this.runLaunchTest(testData, debug, run, testItem);
        } else {
            // Custom integration test handling
            throw new Error("Integration test without launch file not yet supported");
        }
    }
    
    /**
     * Build test executable using colcon via VS Code Task system
     */
    private async buildTestExecutable(packageName: string, debug: boolean): Promise<void> {
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0].uri.fsPath;
        if (!workspaceRoot) {
            throw new Error("No workspace folder found");
        }
        
        const buildType = debug ? 'Debug' : 'RelWithDebInfo';
        
        // Create a colcon build task using the same pattern as the extension's build system
        let installType = '--symlink-install';
        if (process.platform === "win32") {
            installType = '--merge-install';
        }
        
        const taskDefinition = {
            type: 'colcon',
            command: 'colcon',
            args: [
                'build',
                installType,
                '--packages-select', packageName,
                '--event-handlers', 'console_cohesion+',
                '--base-paths', workspaceRoot,
                '--cmake-args', `-DCMAKE_BUILD_TYPE=${buildType}`
            ]
        };
        
        const task = new vscode.Task(
            taskDefinition,
            vscode.TaskScope.Workspace,
            `Build ${packageName}`,
            'colcon'
        );
        
        task.execution = new vscode.ShellExecution('colcon', taskDefinition.args, {
            env: await extension.resolvedEnv(), // Use the ROS environment
        });
        
        task.problemMatchers = ["$colcon-gcc"];
        
        return new Promise<void>((resolve, reject) => {
            // Monitor task execution
            const disposable = vscode.tasks.onDidEndTask((e) => {
                if (e.execution.task === task) {
                    // Task completed - check process disposable for actual result
                    // Don't resolve here, let the process disposable handle success/failure
                    disposable.dispose();
                }
            });
            
            // Monitor for task process end to get actual exit code
            const processDisposable = vscode.tasks.onDidEndTaskProcess((e) => {
                if (e.execution.task === task) {
                    if (e.exitCode === 0) {
                        resolve(); // Build succeeded
                    } else {
                        reject(new Error(`Build failed for package ${packageName} with exit code ${e.exitCode}. ${this.getBuildFailureHelp(packageName)}`));
                    }
                    processDisposable.dispose();
                }
            });
            
            // Execute the task
            vscode.tasks.executeTask(task).then(
                () => {
                    // Task started successfully
                },
                (error) => {
                    disposable.dispose();
                    processDisposable.dispose();
                    reject(new Error(`Failed to start build task for package ${packageName}: ${error.message}`));
                }
            );
            
            // Set timeout for build process
            const timeout = setTimeout(() => {
                reject(new Error(`Build timed out for package ${packageName} after 10 minutes`));
                disposable.dispose();
                processDisposable.dispose();
            }, 600000); // 10 minute build timeout
            
            // Clean up timeout when task completes
            const originalDispose = disposable.dispose.bind(disposable);
            disposable.dispose = () => {
                clearTimeout(timeout);
                processDisposable.dispose();
                originalDispose();
            };
        });
    }
    
    /**
     * Build Python test selector string
     */
    private buildPythonTestSelector(testData: RosTestData): string {
        if (testData.testClass && testData.testMethod) {
            return `${testData.filePath}::${testData.testClass}::${testData.testMethod}`;
        } else if (testData.testMethod) {
            return `${testData.filePath}::${testData.testMethod}`;
        } else {
            return testData.filePath;
        }
    }
    
    /**
     * Build Google Test command line arguments
     */
    private buildGTestArgs(testData: RosTestData): string[] {
        const args: string[] = [];
        
        if (testData.testClass && testData.testMethod) {
            args.push(`--gtest_filter=${testData.testClass}.${testData.testMethod}`);
        } else if (testData.testClass) {
            args.push(`--gtest_filter=${testData.testClass}.*`);
        }
        
        // Add verbose output
        args.push('--gtest_output=xml'); // For parsing results
        args.push('--gtest_color=yes');
        
        return args;
    }
    
    /**
     * Monitor VS Code task execution to update test run status
     */
    private monitorTaskExecution(
        task: vscode.Task,
        run: vscode.TestRun,
        testItem: vscode.TestItem
    ): Promise<void> {
        return new Promise<void>((resolve) => {
            let taskEnded = false;
            let processEnded = false;
            let exitCode: number | undefined;

            const taskDisposable = vscode.tasks.onDidEndTask((e) => {
                if (e.execution.task === task) {
                    taskEnded = true;
                    // Don't mark as passed here - wait for process exit code
                    if (processEnded) {
                        // Process already ended, check final result
                        finalizeTestResult();
                    }
                    taskDisposable.dispose();
                }
            });
            
            const processDisposable = vscode.tasks.onDidEndTaskProcess((e) => {
                if (e.execution.task === task) {
                    processEnded = true;
                    exitCode = e.exitCode;
                    if (taskEnded) {
                        // Task already ended, check final result
                        finalizeTestResult();
                    }
                    processDisposable.dispose();
                }
            });

            const finalizeTestResult = () => {
                if (exitCode === 0) {
                    run.passed(testItem);
                } else {
                    const message = new vscode.TestMessage(`Test failed with exit code ${exitCode}`);
                    run.failed(testItem, message);
                }
                resolve(); // Resolve the promise when test is complete
            };
            
            // Set up timeout
            setTimeout(() => {
                if (!processEnded) {
                    const message = new vscode.TestMessage('Test timed out after 5 minutes');
                    run.failed(testItem, message);
                    resolve(); // Resolve even on timeout
                }
            }, 300000); // 5 minute timeout
        });
    }

    /**
     * Monitor terminal output to update test run status
     */
    private monitorTerminalOutput(
        terminal: vscode.Terminal,
        run: vscode.TestRun,
        testItem: vscode.TestItem
    ): Promise<void> {
        return new Promise<void>((resolve) => {
            const disposable = vscode.window.onDidCloseTerminal((closedTerminal) => {
                if (closedTerminal === terminal) {
                    // Check the exit status to determine success/failure
                    if (closedTerminal.exitStatus) {
                        const exitCode = closedTerminal.exitStatus.code;
                        if (exitCode === 0) {
                            run.passed(testItem);
                        } else {
                            const message = new vscode.TestMessage(`Test failed with exit code ${exitCode}`);
                            run.failed(testItem, message);
                        }
                    } else {
                        // No exit status available, check if process was killed
                        const message = new vscode.TestMessage('Test was terminated or interrupted');
                        run.failed(testItem, message);
                    }
                    disposable.dispose();
                    resolve();
                }
            });
            
            // Set up a timeout to handle hung processes
            const timeout = setTimeout(() => {
                if (!terminal.exitStatus) {
                    const message = new vscode.TestMessage('Test timed out after 5 minutes');
                    run.failed(testItem, message);
                    resolve();
                }
            }, 300000); // 5 minute timeout
            
            // Clean up timeout when terminal closes
            const originalDispose = disposable.dispose;
            disposable.dispose = () => {
                clearTimeout(timeout);
                originalDispose.call(disposable);
            };
        });
    }
    
    /**
     * Create a debug configuration for a specific test with ROS environment
     */
    async createDebugConfiguration(testData: RosTestData): Promise<vscode.DebugConfiguration> {
        const env = await extension.resolvedEnv();
        
        switch (testData.type) {
            case TestType.LaunchTest:
                return {
                    name: `Debug ${path.basename(testData.launchFile || testData.filePath)}`,
                    type: 'ros2',
                    request: 'launch',
                    target: testData.launchFile || testData.filePath,
                    env: {
                        ...env, // Include full ROS environment
                        ROS_TESTING: '1'
                    }
                };
                
            case TestType.PythonUnitTest:
            case TestType.PythonPytest:
                return {
                    name: `Debug ${testData.testMethod || path.basename(testData.filePath)}`,
                    type: 'python',
                    request: 'launch',
                    module: 'pytest',
                    args: ['-v', '-s', this.buildPythonTestSelector(testData)],
                    env: env, // Include ROS environment for Python debugging
                    console: 'integratedTerminal',
                    cwd: vscode.workspace.workspaceFolders?.[0].uri.fsPath
                };
                
            case TestType.CppGtest:
                const executableName = TestDiscoveryUtils.getCppTestExecutable(testData.filePath, testData.packageName || '');
                
                // For debug configuration templates, we use variable substitution
                // The actual executable finding will happen at debug time
                let programPath: string;
                if (process.platform === 'win32') {
                    programPath = `\${workspaceFolder}\\build\\${testData.packageName}\\Debug\\${executableName}.exe`;
                } else {
                    programPath = `\${workspaceFolder}/build/${testData.packageName}/${executableName}`;
                }

                return this.createCppDebugConfig(
                    `Debug ${testData.testClass}.${testData.testMethod}`,
                    programPath,
                    this.buildGTestArgs(testData),
                    '\${workspaceFolder}',
                    env,
                    false // stopAtEntry
                );
                
            default:
                throw new Error(`Unsupported test type for debugging: ${testData.type}`);
        }
    }
    
    /**
     * Find test executable by searching the build directory structure
     */
    private findTestExecutable(workspaceRoot: string, packageName: string, executableName: string): string | undefined {
        const isWindows = process.platform === 'win32';
        const executableExtensions = isWindows ? ['.exe', ''] : [''];
        
        // Search in build directory first
        const buildDir = path.join(workspaceRoot, 'build', packageName);
        const buildExecutable = this.searchForExecutable(buildDir, executableName, executableExtensions);
        if (buildExecutable) {
            return buildExecutable;
        }
        
        // Search in install directory as fallback
        const installDir = path.join(workspaceRoot, 'install', packageName);
        const installExecutable = this.searchForExecutable(installDir, executableName, executableExtensions);
        if (installExecutable) {
            return installExecutable;
        }
        
        return undefined;
    }

    /**
     * Recursively search for an executable file in a directory
     */
    private searchForExecutable(searchDir: string, executableName: string, extensions: string[]): string | undefined {
        if (!fs.existsSync(searchDir)) {
            return undefined;
        }
        
        try {
            const items = fs.readdirSync(searchDir, { withFileTypes: true });
            
            // First, check for exact matches in current directory
            for (const ext of extensions) {
                const fullName = executableName + ext;
                const exactMatch = items.find(item => 
                    item.isFile() && item.name === fullName
                );
                if (exactMatch) {
                    const fullPath = path.join(searchDir, exactMatch.name);
                    // Verify it's executable on Unix systems
                    if (process.platform !== 'win32') {
                        try {
                            const stats = fs.statSync(fullPath);
                            if (!(stats.mode & fs.constants.S_IXUSR)) {
                                continue; // Not executable
                            }
                        } catch {
                            continue;
                        }
                    }
                    return fullPath;
                }
            }
            
            // Then search subdirectories recursively
            for (const item of items) {
                if (item.isDirectory()) {
                    const subDirPath = path.join(searchDir, item.name);
                    const found = this.searchForExecutable(subDirPath, executableName, extensions);
                    if (found) {
                        return found;
                    }
                }
            }
        } catch (error) {
            // Ignore permission errors and continue searching
            console.warn(`Could not search directory ${searchDir}: ${error.message}`);
        }
        
        return undefined;
    }

    /**
     * Create a C++ debug configuration following ROS 2 debugger patterns
     */
    private createCppDebugConfig(
        name: string,
        program: string,
        args: string[],
        cwd: string,
        env: { [key: string]: string },
        stopAtEntry: boolean
    ): vscode.DebugConfiguration {
        // Convert environment to the format expected by C++ debuggers
        const envConfigs = Object.entries(env).map(([name, value]) => ({ name, value }));

        // Check for LLDB extension first (preferred)
        const isLldbInstalled = vscode_utils.isLldbExtensionInstalled();
        if (isLldbInstalled) {
            return {
                name: name,
                type: "lldb",
                request: "launch",
                program: program,
                args: args,
                cwd: cwd,
                env: env,
                stopAtEntry: stopAtEntry
            };
        } else if (process.platform === "win32") {
            // Windows: Use Visual Studio debugger
            return {
                name: name,
                type: "cppvsdbg",
                request: "launch",
                cwd: cwd,
                program: program,
                args: args,
                environment: envConfigs,
                stopAtEntry: stopAtEntry
            };
        } else {
            // Linux/Unix: Use GDB via cppdbg
            return {
                name: name,
                type: "cppdbg",
                request: "launch",
                cwd: cwd,
                program: program,
                args: args,
                environment: envConfigs,
                stopAtEntry: stopAtEntry,
                setupCommands: [
                    {
                        text: "-enable-pretty-printing",
                        description: "Enable pretty-printing for gdb",
                        ignoreFailures: true
                    }
                ]
            };
        }
    }

    /**
     * Provide helpful guidance for common build failures
     */
    private getBuildFailureHelp(packageName: string): string {
        if (process.platform === 'win32') {
            return `Common Windows build issues:
- Missing dependencies: Use ROS 2 with Pixi which includes required dependencies
- Missing Visual Studio: Ensure Visual Studio 2019/2022 with C++ tools is installed
- Environment issues: Make sure ROS 2 environment is properly sourced (check ROS2.pixiRoot setting)
Check the build terminal output for specific error details.`;
        } else {
            return `Common build issues:
- Missing dependencies: Run 'rosdep install --from-paths src --ignore-src -r -y' 
- Missing build tools: Ensure build-essential and cmake are installed
- Environment issues: Make sure ROS 2 environment is properly sourced
Check the build terminal output for specific error details.`;
        }
    }
}