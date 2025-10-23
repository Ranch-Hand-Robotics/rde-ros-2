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
    private discoveryInProgress = false;
    private testsRunning = false;
    private discoveryDebounceTimer?: NodeJS.Timeout;
    private readonly discoveryDebounceDelay = 500; // ms
    private pendingDiscovery = false;

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
        
        // Watch for file changes to refresh test discovery (debounced)
        // Include .test.py for launch_testing files
        const watcher = vscode.workspace.createFileSystemWatcher('**/{test_*,*_test,*Test}.{py,cpp,test.py,launch.py}');
        this.disposables.push(watcher);
        this.disposables.push(watcher.onDidCreate(() => this.scheduleDiscovery()));
        this.disposables.push(watcher.onDidChange(() => this.scheduleDiscovery()));
        this.disposables.push(watcher.onDidDelete(() => this.scheduleDiscovery()));
        
        // Initial test discovery
        this.discoverTests();
    }

    /**
     * Schedule test discovery with debouncing to prevent rapid re-discoveries
     */
    private scheduleDiscovery(): void {
        // If tests are running, defer discovery until they complete
        if (this.testsRunning) {
            this.pendingDiscovery = true;
            extension.outputChannel.appendLine(`Tests are running, deferring discovery...`);
            return;
        }
        
        // Clear any existing timer
        if (this.discoveryDebounceTimer) {
            clearTimeout(this.discoveryDebounceTimer);
        }
        
        // Schedule discovery after debounce delay
        this.discoveryDebounceTimer = setTimeout(() => {
            this.discoverTests();
        }, this.discoveryDebounceDelay);
    }

    /**
     * Discover all ROS 2 tests in the workspace
     */
    private async discoverTests(): Promise<void> {
        if (!vscode.workspace.workspaceFolders) {
            extension.outputChannel.appendLine(`No workspace folders found, skipping test discovery`);
            return;
        }

        // Prevent concurrent discovery operations
        if (this.discoveryInProgress) {
            extension.outputChannel.appendLine(`Test discovery already in progress, skipping...`);
            return;
        }

        this.discoveryInProgress = true;
        const startTime = Date.now();

        try {
            extension.outputChannel.appendLine(`[${new Date().toLocaleTimeString()}] Starting ROS 2 test discovery...`);
            const workspaceRoot = vscode.workspace.workspaceFolders[0].uri.fsPath;
            
            // Create temporary maps to store new test data
            const newTestItemMap = new Map<string, vscode.TestItem>();
            const newTestDataMap = new Map<string, RosTestData>();
            const newTestItems: vscode.TestItem[] = [];
            
            // Discover different types of tests into temporary storage
            await Promise.all([
                this.discoverPythonTestsToMap(workspaceRoot, newTestItemMap, newTestDataMap, newTestItems),
                this.discoverCppTestsToMap(workspaceRoot, newTestItemMap, newTestDataMap, newTestItems),
                this.discoverLaunchTestsToMap(workspaceRoot, newTestItemMap, newTestDataMap, newTestItems)
            ]);
            
            const elapsed = Date.now() - startTime;
            extension.outputChannel.appendLine(`Test discovery completed in ${elapsed}ms, found ${newTestItems.length} test files`);
            
            // Only update if we have tests or if this is the initial discovery
            if (newTestItems.length > 0 || this.testItemMap.size === 0) {
                // Only update the test controller once all discovery is complete
                this.testItemMap = newTestItemMap;
                this.testDataMap = newTestDataMap;
                this.testController.items.replace(newTestItems);
                
                extension.outputChannel.appendLine(`Updated test explorer with ${this.testItemMap.size} total test items`);
            } else {
                extension.outputChannel.appendLine(`No tests found, keeping existing ${this.testItemMap.size} test items`);
            }
        } catch (error) {
            extension.outputChannel.appendLine(`Failed to discover ROS 2 tests: ${error.message}`);
            extension.outputChannel.appendLine(`Stack: ${error.stack}`);
            // Don't clear existing tests on error - keep what we had
        } finally {
            this.discoveryInProgress = false;
            extension.outputChannel.appendLine(`[${new Date().toLocaleTimeString()}] Discovery lock released`);
        }
    }

    /**
     * Discover Python tests (unittest and pytest) into provided maps
     */
    private async discoverPythonTestsToMap(
        workspaceRoot: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>,
        testItems: vscode.TestItem[]
    ): Promise<void> {
        const testFiles = await vscode.workspace.findFiles(
            '**/test_*.py', 
            '{**/build/**,**/install/**,**/log/**}'
        );

        for (const fileUri of testFiles) {
            const filePath = fileUri.fsPath;
            const fileName = path.basename(filePath);
            
            // Skip launch test files (*.test.py, *.launch.py, *.test.launch.py)
            if (fileName.endsWith('.test.py') || 
                fileName.endsWith('.launch.py') || 
                fileName.includes('.launch.')) {
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
            this.parsePythonTestFileToMap(fileItem, filePath, testItemMap, testDataMap);
            
            testItems.push(fileItem);
        }
    }

    /**
     * Discover C++ Google Test files into provided maps
     */
    private async discoverCppTestsToMap(
        workspaceRoot: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>,
        testItems: vscode.TestItem[]
    ): Promise<void> {
        const testFiles = await vscode.workspace.findFiles(
            '**/{test_*,*_test,*Test}.cpp',
            '{**/build/**,**/install/**,**/log/**}'
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

            testDataMap.set(fileItem.id, testData);
            testItemMap.set(fileItem.id, fileItem);
            
            // Parse file for individual test cases
            await this.parseCppTestFileToMap(fileItem, filePath, testItemMap, testDataMap);
            
            testItems.push(fileItem);
        }
    }

    /**
     * Discover launch test files into provided maps
     */
    private async discoverLaunchTestsToMap(
        workspaceRoot: string,
        testItemMap: Map<string, vscode.TestItem>,
        testDataMap: Map<string, RosTestData>,
        testItems: vscode.TestItem[]
    ): Promise<void> {
        // Find .launch.py files
        const launchPyFiles = await vscode.workspace.findFiles(
            '**/*.launch.py',
            '{**/build/**,**/install/**,**/log/**}'
        );
        
        // Find .test.py files (launch_testing convention)
        const testPyFiles = await vscode.workspace.findFiles(
            '**/*.test.py',
            '{**/build/**,**/install/**,**/log/**}'
        );
        
        // Combine both file lists
        const allLaunchFiles = [...launchPyFiles, ...testPyFiles];

        for (const fileUri of allLaunchFiles) {
            const filePath = fileUri.fsPath;
            const fileName = path.basename(filePath);
            
            // Only include test files (should start with test_ or end with _test before the extension)
            const baseName = fileName.replace(/\.(test\.py|launch\.py)$/, '');
            if (!baseName.startsWith('test_') && !baseName.endsWith('_test')) {
                continue;
            }
            
            const packageName = TestDiscoveryUtils.findPackageName(filePath);

            const fileItem = this.testController.createTestItem(
                fileUri.toString(),
                fileName,
                fileUri
            );

            const testData: RosTestData = {
                type: TestType.LaunchTest,
                filePath: filePath,
                packageName: packageName,
                launchFile: filePath
            };

            testDataMap.set(fileItem.id, testData);
            testItemMap.set(fileItem.id, fileItem);
            
            testItems.push(fileItem);
        }
    }

    /**
     * Parse Python test file to extract individual test methods into provided maps
     */
    private parsePythonTestFileToMap(
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
     * Parse C++ test file to extract individual test cases into provided maps
     */
    private parseCppTestFileToMap(
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
     * Collect all runnable test items from the request
     * Handles both specific test selection and "run all" scenarios
     */
    private collectTestItems(request: vscode.TestRunRequest): vscode.TestItem[] {
        const testItems: vscode.TestItem[] = [];
        
        if (!request.include || request.include.length === 0) {
            // No specific tests requested - run all tests
            this.testController.items.forEach(item => {
                this.collectTestItemsRecursive(item, testItems, request.exclude);
            });
        } else {
            // Specific tests requested - collect them and their children
            for (const item of request.include) {
                this.collectTestItemsRecursive(item, testItems, request.exclude);
            }
        }
        
        return testItems;
    }
    
    /**
     * Recursively collect test items, expanding parent items to their children
     */
    private collectTestItemsRecursive(
        item: vscode.TestItem, 
        collected: vscode.TestItem[],
        excluded?: readonly vscode.TestItem[]
    ): void {
        // Check if this item is excluded
        if (excluded && excluded.includes(item)) {
            return;
        }
        
        // If item has children, recursively collect them
        if (item.children.size > 0) {
            item.children.forEach(child => {
                this.collectTestItemsRecursive(child, collected, excluded);
            });
        } else {
            // Leaf node - this is an actual test to run
            collected.push(item);
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
        
        this.testsRunning = true;
        extension.outputChannel.appendLine(`[${new Date().toLocaleTimeString()}] Starting test run, discovery blocked`);
        
        try {
            const testItems = this.collectTestItems(request);
            extension.outputChannel.appendLine(`Collected ${testItems.length} test items to run`);
            
            for (const testItem of testItems) {
                if (cancellation.isCancellationRequested) {
                    extension.outputChannel.appendLine(`Test run cancelled`);
                    break;
                }
                
                const completionPromise = this.runSingleTest(testItem, run, false);
                completionPromises.push(completionPromise);
            }
            
            // Wait for all tests to complete before ending the run
            await Promise.all(completionPromises);
        } catch (error) {
            extension.outputChannel.appendLine(`Error during test run: ${error.message}`);
        } finally {
            run.end();
            this.testsRunning = false;
            extension.outputChannel.appendLine(`[${new Date().toLocaleTimeString()}] Test run completed, discovery unblocked`);
            
            // If discovery was requested while tests were running, run it now
            if (this.pendingDiscovery) {
                this.pendingDiscovery = false;
                extension.outputChannel.appendLine(`Running pending discovery after test completion...`);
                this.scheduleDiscovery();
            }
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
        
        this.testsRunning = true;
        extension.outputChannel.appendLine(`[${new Date().toLocaleTimeString()}] Starting debug session, discovery blocked`);
        
        try {
            const testItems = this.collectTestItems(request);
            extension.outputChannel.appendLine(`Collected ${testItems.length} test items to debug`);
            
            for (const testItem of testItems) {
                if (cancellation.isCancellationRequested) {
                    extension.outputChannel.appendLine(`Debug session cancelled`);
                    break;
                }
                
                const completionPromise = this.runSingleTest(testItem, run, true);
                completionPromises.push(completionPromise);
            }
            
            // Wait for all tests to complete before ending the run
            await Promise.all(completionPromises);
        } catch (error) {
            extension.outputChannel.appendLine(`Error during debug session: ${error.message}`);
        } finally {
            run.end();
            this.testsRunning = false;
            extension.outputChannel.appendLine(`[${new Date().toLocaleTimeString()}] Debug session completed, discovery unblocked`);
            
            // If discovery was requested while tests were running, run it now
            if (this.pendingDiscovery) {
                this.pendingDiscovery = false;
                extension.outputChannel.appendLine(`Running pending discovery after debug completion...`);
                this.scheduleDiscovery();
            }
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
        // Clear debounce timer
        if (this.discoveryDebounceTimer) {
            clearTimeout(this.discoveryDebounceTimer);
        }
        this.disposables.forEach(d => d.dispose());
    }

    /**
     * Manually refresh test discovery
     */
    public refresh(): void {
        extension.outputChannel.appendLine(`Manual refresh requested`);
        
        // Clear any pending timers
        if (this.discoveryDebounceTimer) {
            clearTimeout(this.discoveryDebounceTimer);
            this.discoveryDebounceTimer = undefined;
        }
        
        // If tests are running, mark for later
        if (this.testsRunning) {
            extension.outputChannel.appendLine(`Tests running, will refresh after completion`);
            this.pendingDiscovery = true;
        } else {
            // Run discovery immediately
            extension.outputChannel.appendLine(`Starting immediate discovery`);
            this.discoverTests();
        }
    }
}