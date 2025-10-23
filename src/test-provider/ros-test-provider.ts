// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import * as extension from "../extension";
import { rosApi } from "../ros/ros";
import * as debug_manager from "../debugger/manager";
import { TestDiscoveryUtils } from "./test-discovery-utils";
import { RosTestRunner } from "./ros-test-runner";

/**
 * Represents different types of ROS 2 tests
 */
export enum TestType {
    PythonUnitTest = "python_unittest",
    PythonPytest = "python_pytest", 
    CppGtest = "cpp_gtest",
    LaunchTest = "launch_test",
    Integration = "integration"
}

/**
 * Data structure for ROS 2 test items
 */
export interface RosTestData {
    type: TestType;
    filePath: string;
    packageName?: string;
    launchFile?: string;
    testClass?: string;
    testMethod?: string;
    executable?: string;
}

/**
 * ROS 2 Test Provider for VS Code Test Explorer
 * Discovers and runs ROS 2 tests using existing launch mechanisms
 */
export class RosTestProvider {
    private readonly testController: vscode.TestController;
    private readonly disposables: vscode.Disposable[] = [];
    private testItemMap = new Map<string, vscode.TestItem>();
    private testDataMap = new Map<string, RosTestData>();
    private readonly testRunner: RosTestRunner;

    constructor(private context: vscode.ExtensionContext) {
        this.testController = vscode.tests.createTestController(
            'ros2-test-provider',
            'ROS 2 Tests'
        );
        
        this.testRunner = new RosTestRunner(context);
        
        this.testController.createRunProfile(
            'Run ROS 2 Tests',
            vscode.TestRunProfileKind.Run,
            this.runTests.bind(this),
            true,
            undefined
        );

        this.testController.createRunProfile(
            'Debug ROS 2 Tests', 
            vscode.TestRunProfileKind.Debug,
            this.debugTests.bind(this),
            true,
            undefined
        );

        this.disposables.push(this.testController);
        
        // Watch for file changes to refresh test discovery
        const watcher = vscode.workspace.createFileSystemWatcher('**/{test_*,*_test,*Test}.{py,cpp,launch.py}');
        this.disposables.push(watcher);
        this.disposables.push(watcher.onDidCreate(() => this.discoverTests()));
        this.disposables.push(watcher.onDidChange(() => this.discoverTests()));
        this.disposables.push(watcher.onDidDelete(() => this.discoverTests()));
        
        // Initial test discovery
        this.discoverTests();
    }

    /**
     * Discover all ROS 2 tests in the workspace
     */
    private async discoverTests(): Promise<void> {
        if (!vscode.workspace.workspaceFolders) {
            return;
        }

        // Clear existing tests
        this.testController.items.replace([]);
        this.testItemMap.clear();
        this.testDataMap.clear();

        try {
            extension.outputChannel.appendLine(`Discovering ROS 2 tests...`);
            const workspaceRoot = vscode.workspace.workspaceFolders[0].uri.fsPath;
            
            // Discover different types of tests
            await Promise.all([
                this.discoverPythonTests(workspaceRoot),
                this.discoverCppTests(workspaceRoot),
                this.discoverLaunchTests(workspaceRoot)
            ]);
            extension.outputChannel.appendLine(`Discovered ${this.testItemMap.size} ROS 2 tests.`);
        } catch (error) {
            extension.outputChannel.appendLine(`Failed to discover ROS 2 tests: ${error.message}`);
        }
    }

    /**
     * Discover Python tests (unittest and pytest)
     */
    private async discoverPythonTests(workspaceRoot: string): Promise<void> {
        const testFiles = await vscode.workspace.findFiles(
            '**/test_*.py', 
            '**/build/**'
        );

        for (const fileUri of testFiles) {
            const filePath = fileUri.fsPath;
            const relativePath = path.relative(workspaceRoot, filePath);
            
            // Extract package name from path structure
            const packageName = TestDiscoveryUtils.findPackageName(filePath);
            
            const fileItem = this.testController.createTestItem(
                fileUri.toString(),
                path.basename(filePath),
                fileUri
            );

            const testData: RosTestData = {
                type: TestType.PythonUnitTest,
                filePath: filePath,
                packageName: packageName
            };

            this.testDataMap.set(fileItem.id, testData);
            this.testItemMap.set(fileItem.id, fileItem);
            
            // Parse file for individual test methods using utility
            this.parsePythonTestFile(fileItem, filePath);
            
            this.testController.items.add(fileItem);
        }
    }

    /**
     * Discover C++ Google Test files
     */
    private async discoverCppTests(workspaceRoot: string): Promise<void> {
        const testFiles = await vscode.workspace.findFiles(
            '**/{test_*,*_test,*Test}.cpp',
            '**/build/**'
        );

        for (const fileUri of testFiles) {
            const filePath = fileUri.fsPath;
            const packageName = TestDiscoveryUtils.findPackageName(filePath);

            const fileItem = this.testController.createTestItem(
                fileUri.toString(),
                path.basename(filePath),
                fileUri
            );

            const testData: RosTestData = {
                type: TestType.CppGtest,
                filePath: filePath,
                packageName: packageName
            };

            this.testDataMap.set(fileItem.id, testData);
            this.testItemMap.set(fileItem.id, fileItem);
            
            // Parse file for individual test cases
            await this.parseCppTestFile(fileItem, filePath);
            
            this.testController.items.add(fileItem);
        }
    }

    /**
     * Discover launch test files
     */
    private async discoverLaunchTests(workspaceRoot: string): Promise<void> {
        const launchFiles = await vscode.workspace.findFiles(
            '**/test_*.launch.py',
            '**/build/**'
        );

        for (const fileUri of launchFiles) {
            const filePath = fileUri.fsPath;
            const packageName = TestDiscoveryUtils.findPackageName(filePath);

            const fileItem = this.testController.createTestItem(
                fileUri.toString(),
                path.basename(filePath),
                fileUri
            );

            const testData: RosTestData = {
                type: TestType.LaunchTest,
                filePath: filePath,
                packageName: packageName,
                launchFile: filePath
            };

            this.testDataMap.set(fileItem.id, testData);
            this.testItemMap.set(fileItem.id, fileItem);
            
            this.testController.items.add(fileItem);
        }
    }

    /**
     * Parse Python test file to extract individual test methods
     */
    private parsePythonTestFile(fileItem: vscode.TestItem, filePath: string): void {
        try {
            const testElements = TestDiscoveryUtils.parsePythonTestFile(filePath);
            
            for (const element of testElements) {
                if (element.type === 'method') {
                    const testId = `${fileItem.id}::${element.parent || ''}::${element.name}`;
                    
                    const methodItem = this.testController.createTestItem(
                        testId,
                        element.name,
                        vscode.Uri.file(filePath)
                    );
                    
                    const testData: RosTestData = {
                        type: TestType.PythonUnitTest,
                        filePath: filePath,
                        packageName: this.testDataMap.get(fileItem.id)?.packageName,
                        testClass: element.parent || undefined,
                        testMethod: element.name
                    };
                    
                    this.testDataMap.set(methodItem.id, testData);
                    methodItem.range = new vscode.Range(element.line, 0, element.line + 1, 0);
                    this.testItemMap.set(methodItem.id, methodItem);
                    
                    fileItem.children.add(methodItem);
                }
            }
        } catch (error) {
            extension.outputChannel.appendLine(`Failed to parse Python test file ${filePath}: ${error.message}`);
        }
    }

    /**
     * Parse C++ test file to extract individual test cases
     */
    private parseCppTestFile(fileItem: vscode.TestItem, filePath: string): void {
        try {
            const testCases = TestDiscoveryUtils.parseCppTestFile(filePath);
            
            for (const testCase of testCases) {
                const testId = `${fileItem.id}::${testCase.suite}::${testCase.name}`;
                
                const testItem = this.testController.createTestItem(
                    testId,
                    `${testCase.suite}.${testCase.name}`,
                    vscode.Uri.file(filePath)
                );
                
                const testData: RosTestData = {
                    type: TestType.CppGtest,
                    filePath: filePath,
                    packageName: this.testDataMap.get(fileItem.id)?.packageName,
                    testClass: testCase.suite,
                    testMethod: testCase.name
                };
                
                this.testDataMap.set(testItem.id, testData);
                testItem.range = new vscode.Range(testCase.line, 0, testCase.line + 1, 0);
                this.testItemMap.set(testItem.id, testItem);
                
                fileItem.children.add(testItem);
            }
        } catch (error) {
            extension.outputChannel.appendLine(`Failed to parse C++ test file ${filePath}: ${error.message}`);
        }
    }



    /**
     * Run tests using VS Code Test API
     */
    private async runTests(
        request: vscode.TestRunRequest,
        cancellation: vscode.CancellationToken
    ): Promise<void> {
        const run = this.testController.createTestRun(request);
        const completionPromises: Promise<void>[] = [];
        
        try {
            const testItems = request.include || [];
            
            for (const testItem of testItems) {
                if (cancellation.isCancellationRequested) {
                    break;
                }
                
                const completionPromise = this.runSingleTest(testItem, run, false);
                completionPromises.push(completionPromise);
            }
            
            // Wait for all tests to complete before ending the run
            await Promise.all(completionPromises);
        } finally {
            run.end();
        }
    }

    /**
     * Debug tests using existing ROS 2 debugging infrastructure
     */
    private async debugTests(
        request: vscode.TestRunRequest,
        cancellation: vscode.CancellationToken
    ): Promise<void> {
        const run = this.testController.createTestRun(request);
        const completionPromises: Promise<void>[] = [];
        
        try {
            const testItems = request.include || [];
            
            for (const testItem of testItems) {
                if (cancellation.isCancellationRequested) {
                    break;
                }
                
                const completionPromise = this.runSingleTest(testItem, run, true);
                completionPromises.push(completionPromise);
            }
            
            // Wait for all tests to complete before ending the run
            await Promise.all(completionPromises);
        } finally {
            run.end();
        }
    }

    /**
     * Run a single test using appropriate ROS 2 mechanisms
     */
    private async runSingleTest(
        testItem: vscode.TestItem,
        run: vscode.TestRun,
        debug: boolean
    ): Promise<void> {
        const testData = this.testDataMap.get(testItem.id);
        if (!testData) {
            run.skipped(testItem);
            return;
        }

        run.started(testItem);

        try {
            await this.testRunner.runTest(testData, debug, run, testItem);
            // Test result is handled by the monitoring in runTest
        } catch (error) {
            const message = new vscode.TestMessage(error.message);
            run.failed(testItem, message);
        }
    }



    /**
     * Dispose of resources
     */
    public dispose(): void {
        this.disposables.forEach(d => d.dispose());
    }

    /**
     * Manually refresh test discovery
     */
    public refresh(): void {
        this.discoverTests();
    }
}