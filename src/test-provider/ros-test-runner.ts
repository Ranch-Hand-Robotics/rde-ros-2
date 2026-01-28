// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as cp from "child_process";
import * as extension from "../extension";
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
            case TestType.PythonUnitTest:
            case TestType.PythonPytest:
                return this.runPythonTest(testData, debug, run, testItem);
            case TestType.CppGtest:
                return this.runCppTest(testData, debug, run, testItem);
            case TestType.LaunchTest:
            case TestType.Integration:
                throw new Error(`Test type ${testData.type} is currently disabled`);
            default:
                throw new Error(`Unsupported test type: ${testData.type}`);
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

            extension.outputChannel.appendLine(`  Starting debug session for Python test: ${testData.testMethod || path.basename(testData.filePath)}`);

            // Wait for the debug session to start and complete
            return new Promise<void>((resolve) => {
                const debugSessionPromise = vscode.debug.startDebugging(
                    vscode.workspace.workspaceFolders?.[0],
                    debugConfig
                );

                Promise.resolve(debugSessionPromise).then((debugSessionStarted) => {
                    if (!debugSessionStarted) {
                        extension.outputChannel.appendLine(`  Failed to start Python debug session`);
                        if (run && testItem) {
                            const message = new vscode.TestMessage('Failed to start debug session');
                            run.failed(testItem, message);
                        }
                        resolve();
                        return;
                    }

                    extension.outputChannel.appendLine(`  Python debug session started, waiting for completion...`);

                    // Wait for the debug session to end
                    const debugEndListener = vscode.debug.onDidTerminateDebugSession((session) => {
                        if (session.name === debugConfig.name) {
                            debugEndListener.dispose();
                            extension.outputChannel.appendLine(`  Python debug session ended for ${testData.testMethod || path.basename(testData.filePath)}`);
                            
                            if (run && testItem) {
                                // When debugging, we assume success if the session ended
                                run.passed(testItem);
                            }
                            resolve();
                        }
                    });

                    // Ensure we clean up after a timeout
                    const debugTimeout = setTimeout(() => {
                        debugEndListener.dispose();
                        extension.outputChannel.appendLine(`  Python debug session timeout`);
                        resolve();
                    }, 1800000); // 30 minute debug timeout

                    // Clean up timeout when session ends
                    vscode.debug.onDidTerminateDebugSession((session) => {
                        if (session.name === debugConfig.name) {
                            clearTimeout(debugTimeout);
                        }
                    });
                }).catch((error) => {
                    extension.outputChannel.appendLine(`  Error starting Python debug session: ${error.message}`);
                    if (run && testItem) {
                        const message = new vscode.TestMessage(`Error starting debug session: ${error.message}`);
                        run.failed(testItem, message);
                    }
                    resolve();
                });
            });
        } else {
            // Execute test directly with child_process instead of creating a task
            return this.executeTestCommand(testCommand, env, run, testItem);
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
        
        // Check if executable already exists before building
        const executableName = TestDiscoveryUtils.getCppTestExecutable(testData.filePath, testData.packageName);
        extension.outputChannel.appendLine(`  Looking for C++ test executable: ${executableName} in package ${testData.packageName}`);
        let executablePath = executableName ? this.findTestExecutable(workspaceRoot, testData.packageName, executableName) : undefined;
        extension.outputChannel.appendLine(`  Executable path: ${executablePath || 'not found'}`);
        
        // Only build if executable doesn't exist
        if (!executablePath) {
            extension.outputChannel.appendLine(`  Building test executable for package ${testData.packageName}...`);
            try {
                await this.buildTestExecutable(testData.packageName, debug);
                executablePath = executableName ? this.findTestExecutable(workspaceRoot, testData.packageName, executableName) : undefined;
                extension.outputChannel.appendLine(`  After build, executable path: ${executablePath || 'still not found'}`);
            } catch (buildError) {
                throw new Error(`Failed to build C++ test package ${testData.packageName}: ${buildError.message}`);
            }
        }
        
        if (debug) {
            // For debugging, we still need to run the executable directly
            if (!executablePath) {
                throw new Error(`Test executable not found in build directory for package '${testData.packageName}'.`);
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

            extension.outputChannel.appendLine(`  Starting debug session for ${testData.testClass}.${testData.testMethod}`);
            
            // Wait for the debug session to start and complete
            return new Promise<void>((resolve) => {
                // Start the debug session
                const debugSessionPromise = vscode.debug.startDebugging(
                    vscode.workspace.workspaceFolders?.[0],
                    debugConfig
                );

                // Handle the promise returned by startDebugging
                Promise.resolve(debugSessionPromise).then((debugSessionStarted) => {
                    if (!debugSessionStarted) {
                        extension.outputChannel.appendLine(`  Failed to start debug session`);
                        if (run && testItem) {
                            const message = new vscode.TestMessage('Failed to start debug session');
                            run.failed(testItem, message);
                        }
                        resolve();
                        return;
                    }

                    extension.outputChannel.appendLine(`  Debug session started, waiting for completion...`);

                    // Wait for the debug session to end
                    const debugEndListener = vscode.debug.onDidTerminateDebugSession((session) => {
                        // Check if this is our debug session by comparing names
                        if (session.name === debugConfig.name) {
                            debugEndListener.dispose();
                            extension.outputChannel.appendLine(`  Debug session ended for ${testData.testClass}.${testData.testMethod}`);
                            
                            if (run && testItem) {
                                // When debugging, we assume success if the session ended (user may have closed it)
                                // The actual test result would need to be parsed from debugger output
                                run.passed(testItem);
                            }
                            resolve();
                        }
                    });

                    // Ensure we clean up after a timeout
                    const debugTimeout = setTimeout(() => {
                        debugEndListener.dispose();
                        extension.outputChannel.appendLine(`  Debug session timeout for ${testData.testClass}.${testData.testMethod}`);
                        resolve();
                    }, 1800000); // 30 minute debug timeout

                    // Clean up timeout when session ends
                    vscode.debug.onDidTerminateDebugSession((session) => {
                        if (session.name === debugConfig.name) {
                            clearTimeout(debugTimeout);
                        }
                    });
                }).catch((error) => {
                    extension.outputChannel.appendLine(`  Error starting debug session: ${error.message}`);
                    if (run && testItem) {
                        const message = new vscode.TestMessage(`Error starting debug session: ${error.message}`);
                        run.failed(testItem, message);
                    }
                    resolve();
                });
            });
        } else {
            // For C++ tests, run the executable directly since colcon test doesn't support
            // passing gtest filters through the -- separator
            if (!executablePath) {
                throw new Error(`Test executable not found in build directory for package '${testData.packageName}'.`);
            }
            
            // Build command line arguments for the test executable
            const testArgs = this.buildGTestArgs(testData);
            
            // Execute test directly with child_process instead of creating a task
            return this.executeTestCommand([executablePath, ...testArgs], env, run, testItem, workspaceRoot);
        }
    }
    
    /**
     * Build test executable using colcon directly (no visible terminals)
     */
    private async buildTestExecutable(packageName: string, debug: boolean): Promise<void> {
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0].uri.fsPath;
        if (!workspaceRoot) {
            throw new Error("No workspace folder found");
        }
        
        const env = await extension.resolvedEnv();
        const buildType = debug ? 'Debug' : 'RelWithDebInfo';
        
        let installType = '--symlink-install';
        if (process.platform === "win32") {
            installType = '--merge-install';
        }
        
        const args = [
            'build',
            installType,
            '--packages-select', packageName,
            '--event-handlers', 'console_cohesion+',
            '--base-paths', workspaceRoot,
            '--cmake-args', `-DCMAKE_BUILD_TYPE=${buildType}`
        ];
        
        return new Promise<void>((resolve, reject) => {
            const proc = cp.spawn('colcon', args, {
                env: env,
                cwd: workspaceRoot,
                stdio: 'pipe'
            });

            let buildOutput = '';

            if (proc.stdout) {
                proc.stdout.on('data', (data) => {
                    buildOutput += data.toString();
                });
            }

            if (proc.stderr) {
                proc.stderr.on('data', (data) => {
                    buildOutput += data.toString();
                });
            }

            proc.on('close', (code) => {
                if (code === 0) {
                    resolve();
                } else {
                    reject(new Error(`Build failed for package ${packageName} with exit code ${code}.\n${buildOutput}\n${this.getBuildFailureHelp(packageName)}`));
                }
            });

            proc.on('error', (error) => {
                reject(new Error(`Failed to start colcon build: ${error.message}`));
            });

            // Set timeout for build process
            const timeout = setTimeout(() => {
                proc.kill();
                reject(new Error(`Build timed out for package ${packageName} after 10 minutes`));
            }, 600000); // 10 minute build timeout

            // Clean up timeout when process ends
            proc.on('close', () => {
                clearTimeout(timeout);
            });
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
     * Execute a test command directly using child_process, monitoring output and results
     */
    private executeTestCommand(
        command: string[],
        env: { [key: string]: string },
        run?: vscode.TestRun,
        testItem?: vscode.TestItem,
        cwd?: string
    ): Promise<void> {
        return new Promise<void>((resolve) => {
            const proc = cp.spawn(command[0], command.slice(1), {
                env: env,
                cwd: cwd || vscode.workspace.workspaceFolders?.[0].uri.fsPath,
                stdio: 'pipe',
                shell: false
            });

            let exitCode: number | undefined;
            let output = '';
            let resolved = false;

            if (proc.stdout) {
                proc.stdout.on('data', (data) => {
                    output += data.toString();
                });
                proc.stdout.on('end', () => {
                    // Ensure stdout is closed
                });
            }

            if (proc.stderr) {
                proc.stderr.on('data', (data) => {
                    output += data.toString();
                });
                proc.stderr.on('end', () => {
                    // Ensure stderr is closed
                });
            }

            proc.on('close', (code) => {
                if (resolved) {
                    return; // Already handled by timeout or error
                }
                resolved = true;
                exitCode = code;
                extension.outputChannel.appendLine(`Test completed with exit code: ${exitCode}`);
                if (run && testItem) {
                    if (exitCode === 0) {
                        run.passed(testItem);
                    } else {
                        const message = new vscode.TestMessage(`Test failed with exit code ${exitCode}\n${output}`);
                        run.failed(testItem, message);
                    }
                }
                resolve();
            });

            proc.on('error', (error) => {
                if (resolved) {
                    return; // Already handled
                }
                resolved = true;
                extension.outputChannel.appendLine(`Test execution error: ${error.message}`);
                if (run && testItem) {
                    const message = new vscode.TestMessage(`Test execution error: ${error.message}`);
                    run.failed(testItem, message);
                }
                resolve();
            });

            // Set timeout for test execution
            const timeout = setTimeout(() => {
                if (!resolved && proc && !exitCode) {
                    resolved = true;
                    extension.outputChannel.appendLine(`Test timed out after 5 minutes, force killing process`);
                    proc.kill('SIGKILL');
                    if (run && testItem) {
                        const message = new vscode.TestMessage('Test timed out after 5 minutes');
                        run.failed(testItem, message);
                    }
                    resolve();
                }
            }, 300000); // 5 minute timeout

            // Clean up timeout when process ends
            proc.on('close', () => {
                clearTimeout(timeout);
            });
        });
    }

    /**
     * Create a debug configuration for a specific test with ROS environment
     */
    async createDebugConfiguration(testData: RosTestData): Promise<vscode.DebugConfiguration> {
        const env = await extension.resolvedEnv();
        
        switch (testData.type) {
            case TestType.LaunchTest:
                throw new Error(`Test type ${testData.type} is currently disabled`);
                
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

        // Prefer cpptools if available, fall back to LLDB
        const isCppToolsInstalled = vscode_utils.isCppToolsExtensionInstalled();
        const isLldbInstalled = vscode_utils.isLldbExtensionInstalled();
        const isCursor = vscode_utils.isCursorEditor();

        if (!isCppToolsInstalled && !isLldbInstalled) {
            let message: string;
            if (isCursor) {
                message = "LLDB is required for C++ test debugging. Install the LLDB extension.";
            } else {
                message = "C++ test debugging requires the Microsoft C/C++ extension (ms-vscode.cpptools) or LLDB extension.";
            }
            vscode.window.showErrorMessage(message);
            throw new Error(message);
        }
        if (isCppToolsInstalled) {
            return {
                name: name,
                type: process.platform === "win32" ? "cppvsdbg" : "cppdbg",
                request: "launch",
                program: program,
                args: args,
                cwd: cwd,
                environment: envConfigs,
                stopAtEntry: stopAtEntry,
                ...(process.platform !== "win32" && {
                    setupCommands: [
                        {
                            text: "-enable-pretty-printing",
                            description: "Enable pretty-printing for gdb",
                            ignoreFailures: true
                        }
                    ]
                })
            };
        } else if (isLldbInstalled) {
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
        } else {
            // Should not reach here due to check above, but for safety
            throw new Error("No C++ debugger available");
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