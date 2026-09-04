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
    private isDiscovering = false;

    constructor(private context: vscode.ExtensionContext) {
        this.testController = vscode.tests.createTestController(
            'ros2-test-provider',
            'ROS 2 Tests'
        );
        
        // Set up refresh handler for test explorer refresh button
        this.testController.resolveHandler = async (item) => {
            if (!item) {
                // Refresh all tests when clicking the refresh button
                await this.discoverTests();
            }
        };
        
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
        
        // Initial test discovery (fire and forget, but will complete asynchronously)
        this.discoverTests().catch((error) => {
            extension.outputChannel.appendLine(`Error during initial test discovery: ${error.message}`);
        });
    }

    /**
     * Check if a file path should be excluded based on testExcludeFolders setting
     */
    private isPathExcluded(filePath: string, workspaceRoot: string): boolean {
        const config = vscode.workspace.getConfiguration('ROS2');
        const excludeFolders: string[] = config.get('testExcludeFolders', []);
        
        if (excludeFolders.length === 0) {
            return false;
        }
        
        // Resolve variables in exclude paths
        const resolvedExcludeFolders = excludeFolders.map(excludePath => {
            // Replace ${workspaceFolder} with actual workspace root
            let resolved = excludePath.replace(/\$\{workspaceFolder\}/g, workspaceRoot);
            
            // If path is not absolute, make it relative to workspace root
            if (!path.isAbsolute(resolved)) {
                resolved = path.join(workspaceRoot, resolved);
            }
            
            // Normalize path for comparison
            return path.normalize(resolved);
        });
        
        const normalizedFilePath = path.normalize(filePath);
        
        // Check if file is under any excluded folder
        for (const excludeFolder of resolvedExcludeFolders) {
            // Ensure we match complete folder boundaries by checking for path separator
            if (normalizedFilePath === excludeFolder || 
                normalizedFilePath.startsWith(excludeFolder + path.sep)) {
                return true;
            }
        }
        
        return false;
    }

    /**
     * Log that a test file was skipped due to exclusion
     */
    private logExcludedFile(filePath: string, workspaceRoot: string): void {
        extension.outputChannel.appendLine(
            `  Skipped ${path.relative(workspaceRoot, filePath)} - path excluded by testExcludeFolders setting`
        );
    }

    /**
     * Discover all ROS 2 tests in the workspace
     */
    private async discoverTests(): Promise<void> {
        if (this.isDiscovering) {
            extension.outputChannel.appendLine(`Test discovery already in progress, skipping...`);
            return; // Skip if already discovering
        }

        if (!vscode.workspace.workspaceFolders) {
            extension.outputChannel.appendLine(`No workspace folders found - skipping test discovery`);
            return;
        }

        this.isDiscovering = true;
        
        try {
            extension.outputChannel.appendLine(`Discovering ROS 2 tests...`);
            const workspaceRoot = vscode.workspace.workspaceFolders[0].uri.fsPath;
            extension.outputChannel.appendLine(`  Workspace root: ${workspaceRoot}`);
            
            // Build new test collections WITHOUT modifying state yet
            const newTestItemMap = new Map<string, vscode.TestItem>();
            const newTestDataMap = new Map<string, RosTestData>();
            const newRootItems: vscode.TestItem[] = [];
            
            // Discover different types of tests
            extension.outputChannel.appendLine(`  Searching for Python tests...`);
            await this.discoverPythonTests(workspaceRoot, newTestItemMap, newTestDataMap, newRootItems);
            extension.outputChannel.appendLine(`  Searching for C++ tests...`);
            await this.discoverCppTests(workspaceRoot, newTestItemMap, newTestDataMap, newRootItems);
            
            // SUCCESS: Now update state with discovered tests
            this.testItemMap = newTestItemMap;
            this.testDataMap = newTestDataMap;
            this.testController.items.replace(newRootItems);
            
            extension.outputChannel.appendLine(`Discovered ${this.testItemMap.size} ROS 2 tests.`);
        } catch (error) {
            extension.outputChannel.appendLine(`Failed to discover ROS 2 tests: ${error.message}`);
            extension.outputChannel.appendLine(`Stack trace: ${error.stack}`);
            // NOTE: Do NOT clear items on error - keep the previous state
        } finally {
            this.isDiscovering = false;
        }
    }

    /**
     * Discover Python tests (unittest and pytest)
     */
    private async discoverPythonTests(
        workspaceRoot: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>,
        rootItems: vscode.TestItem[]
    ): Promise<void> {
        const testFiles = await vscode.workspace.findFiles(
            '**/test_*.py'
        );

        for (const fileUri of testFiles) {
            const filePath = fileUri.fsPath;
            
            // Skip if path is excluded
            if (this.isPathExcluded(filePath, workspaceRoot)) {
                this.logExcludedFile(filePath, workspaceRoot);
                continue;
            }
            
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

            testDataMap.set(fileItem.id, testData);
            testItemMap.set(fileItem.id, fileItem);
            
            // Parse file for individual test methods using utility
            this.parsePythonTestFile(fileItem, filePath, testItemMap, testDataMap);
            
            // Only add to root items if the file contains tests
            if (fileItem.children.size > 0) {
                extension.outputChannel.appendLine(`  Found Python test file: ${path.basename(filePath)} (${fileItem.children.size} tests)`);
                rootItems.push(fileItem);
            } else {
                extension.outputChannel.appendLine(`  Skipped ${path.basename(filePath)} - no tests found`);
                // Clean up if no tests found
                testDataMap.delete(fileItem.id);
                testItemMap.delete(fileItem.id);
            }
        }
    }

    /**
     * Discover C++ Google Test files
     */
    private async discoverCppTests(
        workspaceRoot: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>,
        rootItems: vscode.TestItem[]
    ): Promise<void> {
        const testFiles = await vscode.workspace.findFiles(
            '**/{test_*,*_test,*Test}.cpp'
        );

        for (const fileUri of testFiles) {
            const filePath = fileUri.fsPath;
            
            // Skip if path is excluded
            if (this.isPathExcluded(filePath, workspaceRoot)) {
                this.logExcludedFile(filePath, workspaceRoot);
                continue;
            }
            
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

            testDataMap.set(fileItem.id, testData);
            testItemMap.set(fileItem.id, fileItem);
            
            // Parse file for individual test cases
            await this.parseCppTestFile(fileItem, filePath, testItemMap, testDataMap);
            
            // Only add to root items if the file contains tests
            if (fileItem.children.size > 0) {
                extension.outputChannel.appendLine(`  Found C++ test file: ${path.basename(filePath)} (${fileItem.children.size} tests)`);
                rootItems.push(fileItem);
            } else {
                extension.outputChannel.appendLine(`  Skipped ${path.basename(filePath)} - no tests found`);
                // Clean up if no tests found
                testDataMap.delete(fileItem.id);
                testItemMap.delete(fileItem.id);
            }
        }
    }

    /**
     * Parse Python test file to extract individual test methods
     */
    private parsePythonTestFile(
        fileItem: vscode.TestItem,
        filePath: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>
    ): void {
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
                        packageName: testDataMap.get(fileItem.id)?.packageName,
                        testClass: element.parent || undefined,
                        testMethod: element.name
                    };
                    
                    testDataMap.set(methodItem.id, testData);
                    methodItem.range = new vscode.Range(element.line, 0, element.line + 1, 0);
                    testItemMap.set(methodItem.id, methodItem);
                    
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
    private parseCppTestFile(
        fileItem: vscode.TestItem,
        filePath: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>
    ): void {
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
                    packageName: testDataMap.get(fileItem.id)?.packageName,
                    testClass: testCase.suite,
                    testMethod: testCase.name
                };
                
                testDataMap.set(testItem.id, testData);
                testItem.range = new vscode.Range(testCase.line, 0, testCase.line + 1, 0);
                testItemMap.set(testItem.id, testItem);
                
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
            // Collect tests to run - either from request or all tests
            const testItems = request.include || this.gatherAllTests();
            extension.outputChannel.appendLine(`Running ${testItems.length} test item(s)...`);
            
            // Run tests in parallel with a single shared terminal
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
        
        try {
            // Collect tests to debug - either from request or all tests
            const testItems = request.include || this.gatherAllTests();
            
            // Only allow debugging a single test at a time
            if (testItems.length > 1) {
                const message = 'Debugging multiple tests is not supported. Please select a single test to debug.';
                vscode.window.showErrorMessage(message);
                extension.outputChannel.appendLine(`Debug failed: ${message}`);
                run.end();
                return;
            }
            
            if (testItems.length === 0) {
                const message = 'No tests selected for debugging.';
                vscode.window.showWarningMessage(message);
                run.end();
                return;
            }
            
            extension.outputChannel.appendLine(`Debugging ${testItems.length} test item(s)...`);
            
            // Debug the single test
            const testItem = testItems[0];
            await this.runSingleTest(testItem, run, true);
        } finally {
            run.end();
        }
    }

    /**
     * Gather all test items from the controller
     */
    private gatherAllTests(): vscode.TestItem[] {
        const allTests: vscode.TestItem[] = [];
        this.testController.items.forEach(item => {
            allTests.push(item);
            // Also include child test items
            item.children.forEach(child => {
                allTests.push(child);
            });
        });
        return allTests;
    }

    /**
     * Run a single test using appropriate ROS 2 mechanisms
     */
    private async runSingleTest(
        testItem: vscode.TestItem,
        run: vscode.TestRun,
        debug: boolean
    ): Promise<void> {
        // If this test item has children, run all children in parallel
        if (testItem.children.size > 0) {
            extension.outputChannel.appendLine(`  Running ${testItem.children.size} child tests in ${testItem.label}...`);
            const childPromises: Promise<void>[] = [];
            testItem.children.forEach(child => {
                childPromises.push(this.runSingleTest(child, run, debug));
            });
            await Promise.all(childPromises);
            return;
        }

        const testData = this.testDataMap.get(testItem.id);
        if (!testData) {
            extension.outputChannel.appendLine(`  Skipping ${testItem.label} - no test data found`);
            run.skipped(testItem);
            return;
        }

        extension.outputChannel.appendLine(`  ${debug ? 'Debugging' : 'Running'}: ${testItem.label} (${testData.type})`);
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