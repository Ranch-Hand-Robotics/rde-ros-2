// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as fs from "fs";
import * as path from "path";
import * as vscode from "vscode";
import { TestType, RosTestData } from "./ros-test-provider";

/**
 * Utilities for discovering and parsing ROS 2 test files
 */
export class TestDiscoveryUtils {
    
    /**
     * Parse Python test file to find test classes and methods
     */
    static parsePythonTestFile(filePath: string): Array<{
        name: string;
        type: 'class' | 'method';
        line: number;
        parent?: string;
    }> {
        const results: Array<{
            name: string;
            type: 'class' | 'method';
            line: number;
            parent?: string;
        }> = [];
        
        try {
            const content = fs.readFileSync(filePath, 'utf8');
            const lines = content.split('\n');
            
            let currentClass: string | null = null;
            
            for (let i = 0; i < lines.length; i++) {
                const line = lines[i];
                const trimmedLine = line.trim();
                
                // Skip empty lines and comments
                if (!trimmedLine || trimmedLine.startsWith('#')) {
                    continue;
                }
                
                // Detect test classes (class names containing 'Test' or ending with 'Test')
                const classMatch = trimmedLine.match(/^class\s+(\w*[Tt]est\w*)\s*\(/);
                if (classMatch) {
                    currentClass = classMatch[1];
                    results.push({
                        name: currentClass,
                        type: 'class',
                        line: i
                    });
                    continue;
                }
                
                // Detect test methods (starting with 'test_')
                const methodMatch = trimmedLine.match(/^def\s+(test_\w+)\s*\(/);
                if (methodMatch) {
                    const methodName = methodMatch[1];
                    results.push({
                        name: methodName,
                        type: 'method',
                        line: i,
                        parent: currentClass || undefined
                    });
                }
                
                // Reset current class when we encounter a new top-level class
                if (trimmedLine.startsWith('class ') && !classMatch) {
                    currentClass = null;
                }
            }
        } catch (error) {
            console.warn(`Failed to parse Python test file ${filePath}: ${error.message}`);
        }
        
        return results;
    }
    
    /**
     * Parse C++ test file to find TEST and TEST_F cases
     */
    static parseCppTestFile(filePath: string): Array<{
        name: string;
        suite: string;
        line: number;
        isFixture: boolean;
    }> {
        const results: Array<{
            name: string;
            suite: string;
            line: number;
            isFixture: boolean;
        }> = [];
        
        try {
            const content = fs.readFileSync(filePath, 'utf8');
            const lines = content.split('\n');
            
            for (let i = 0; i < lines.length; i++) {
                const line = lines[i];
                const trimmedLine = line.trim();
                
                // Skip empty lines and comments
                if (!trimmedLine || trimmedLine.startsWith('//') || trimmedLine.startsWith('/*')) {
                    continue;
                }
                
                // Look for TEST and TEST_F macros
                const testMatch = trimmedLine.match(/^TEST\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)/);
                const testFMatch = trimmedLine.match(/^TEST_F\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)/);
                
                if (testMatch) {
                    const [, testSuite, testName] = testMatch;
                    results.push({
                        name: testName,
                        suite: testSuite,
                        line: i,
                        isFixture: false
                    });
                } else if (testFMatch) {
                    const [, testSuite, testName] = testFMatch;
                    results.push({
                        name: testName,
                        suite: testSuite,
                        line: i,
                        isFixture: true
                    });
                }
            }
        } catch (error) {
            console.warn(`Failed to parse C++ test file ${filePath}: ${error.message}`);
        }
        
        return results;
    }
    
    /**
     * Parse launch test file to extract test information
     */
    static parseLaunchTestFile(filePath: string): {
        hasTests: boolean;
        testNodes: string[];
        description?: string;
    } {
        const result = {
            hasTests: false,
            testNodes: [] as string[],
            description: undefined as string | undefined
        };
        
        try {
            const content = fs.readFileSync(filePath, 'utf8');
            
            // Look for test-related patterns in launch files
            if (content.includes('test_') || 
                content.includes('TestNode') || 
                content.includes('launch_testing') ||
                content.includes('pytest')) {
                result.hasTests = true;
            }
            
            // Extract test node names (basic pattern matching)
            const nodeMatches = content.match(/name=['"]([^'"]*test[^'"]*)['"]/gi);
            if (nodeMatches) {
                result.testNodes = nodeMatches.map(match => {
                    const nameMatch = match.match(/name=['"]([^'"]*)['"]/i);
                    return nameMatch ? nameMatch[1] : '';
                }).filter(name => name);
            }
            
            // Look for description in docstring or comments
            const descMatch = content.match(/['"]{3}([^'"]*)['"]{3}/) || 
                             content.match(/##?\s*(.+)/);
            if (descMatch) {
                result.description = descMatch[1].trim();
            }
            
        } catch (error) {
            console.warn(`Failed to parse launch test file ${filePath}: ${error.message}`);
        }
        
        return result;
    }
    
    /**
     * Find package.xml file in parent directories to determine package name
     */
    static findPackageName(filePath: string): string | undefined {
        let currentPath = path.dirname(filePath);
        const root = path.parse(currentPath).root;
        
        while (currentPath && currentPath !== root) {
            const packageXmlPath = path.join(currentPath, 'package.xml');
            if (fs.existsSync(packageXmlPath)) {
                try {
                    const packageXmlContent = fs.readFileSync(packageXmlPath, 'utf8');
                    const nameMatch = packageXmlContent.match(/<name>([^<]+)<\/name>/);
                    if (nameMatch) {
                        return nameMatch[1].trim();
                    }
                    // Fallback to directory name
                    return path.basename(currentPath);
                } catch (error) {
                    console.warn(`Failed to parse package.xml at ${packageXmlPath}: ${error.message}`);
                    return path.basename(currentPath);
                }
            }
            currentPath = path.dirname(currentPath);
        }
        
        return undefined;
    }
    
    /**
     * Check if a file is likely a ROS 2 test file based on naming conventions
     */
    static isTestFile(filePath: string): { isTest: boolean; type?: TestType } {
        const fileName = path.basename(filePath);
        const dirName = path.basename(path.dirname(filePath));
        
        // Python test files
        if (fileName.endsWith('.py')) {
            if (fileName.startsWith('test_') || 
                fileName.endsWith('_test.py') ||
                dirName === 'test' ||
                dirName === 'tests') {
                return { isTest: true, type: TestType.PythonUnitTest };
            }
        }
        
        // C++ test files
        if (fileName.endsWith('.cpp') || fileName.endsWith('.cc')) {
            if (fileName.startsWith('test_') || 
                fileName.includes('_test.') ||
                fileName.includes('Test.') ||
                dirName === 'test' ||
                dirName === 'tests') {
                return { isTest: true, type: TestType.CppGtest };
            }
        }
        
        // Launch test files
        if (fileName.endsWith('.launch.py')) {
            if (fileName.startsWith('test_') || 
                fileName.includes('_test.') ||
                dirName === 'test' ||
                dirName === 'tests') {
                return { isTest: true, type: TestType.LaunchTest };
            }
        }
        
        return { isTest: false };
    }
    
    /**
     * Get test executable name for C++ tests by checking build directory
     */
    static getCppTestExecutable(filePath: string, packageName: string): string | undefined {
        const fileName = path.basename(filePath, path.extname(filePath));
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
        
        if (!workspaceRoot || !packageName) {
            return undefined;
        }
        
        const buildDir = path.join(workspaceRoot, 'build', packageName);
        
        // Common patterns for test executable names
        const possibleNames = [
            `test_${packageName}`,
            `${packageName}_test`,
            fileName,
            fileName.replace(/[._]/g, '_'),
            `${fileName}_test`,
            `test_${fileName}`
        ];
        
        // Check if any of these executables actually exist in the build directory
        for (const execName of possibleNames) {
            const execPath = path.join(buildDir, execName);
            const execPathWithExt = path.join(buildDir, `${execName}.exe`);
            
            if (fs.existsSync(execPath) || fs.existsSync(execPathWithExt)) {
                return execName;
            }
        }
        
        // If no executable found, try to parse CMakeLists.txt for the actual target name
        const packageDir = this.findPackageDirectory(filePath);
        if (packageDir) {
            const cmakeListsPath = path.join(packageDir, 'CMakeLists.txt');
            if (fs.existsSync(cmakeListsPath)) {
                const executableName = this.parseTestExecutableFromCMake(cmakeListsPath, fileName);
                if (executableName) {
                    return executableName;
                }
            }
        }
        
        // Fallback to most common pattern
        return `test_${packageName}`;
    }
    
    /**
     * Find the package directory containing the test file
     */
    private static findPackageDirectory(filePath: string): string | undefined {
        let currentPath = path.dirname(filePath);
        const root = path.parse(currentPath).root;
        
        while (currentPath && currentPath !== root) {
            const packageXmlPath = path.join(currentPath, 'package.xml');
            const cmakeListsPath = path.join(currentPath, 'CMakeLists.txt');
            
            if (fs.existsSync(packageXmlPath) && fs.existsSync(cmakeListsPath)) {
                return currentPath;
            }
            currentPath = path.dirname(currentPath);
        }
        
        return undefined;
    }
    
    /**
     * Parse CMakeLists.txt to find the actual test executable name
     */
    private static parseTestExecutableFromCMake(cmakeListsPath: string, testFileName: string): string | undefined {
        try {
            const content = fs.readFileSync(cmakeListsPath, 'utf8');
            const lines = content.split('\n');
            
            for (const line of lines) {
                const trimmedLine = line.trim();
                
                // Look for add_executable or ament_add_gtest with our test file
                const execMatch = trimmedLine.match(/(?:add_executable|ament_add_gtest)\s*\(\s*(\w+)\s+.*?${testFileName}/);
                if (execMatch) {
                    return execMatch[1];
                }
                
                // Look for target names that include our test file in sources
                const targetMatch = trimmedLine.match(/(?:add_executable|ament_add_gtest)\s*\(\s*(\w+)/);
                if (targetMatch) {
                    const targetName = targetMatch[1];
                    // Return target name if it looks like a test and might be related to our file
                    if ((targetName.includes('test') || targetName.includes('Test')) && 
                        (targetName.toLowerCase().includes(testFileName.toLowerCase().replace(/[._]/g, '')) ||
                         testFileName.toLowerCase().includes(targetName.toLowerCase()))) {
                        return targetName;
                    }
                }
            }
        } catch (error) {
            console.warn(`Failed to parse CMakeLists.txt at ${cmakeListsPath}: ${error.message}`);
        }
        
        return undefined;
    }
    
    /**
     * Extract test command from Python test file
     */
    static getPythonTestCommand(testData: RosTestData): string[] {
        const args = ['python', '-m', 'pytest', '-v'];
        
        if (testData.testClass && testData.testMethod) {
            args.push(`${testData.filePath}::${testData.testClass}::${testData.testMethod}`);
        } else if (testData.testMethod) {
            args.push(`${testData.filePath}::${testData.testMethod}`);
        } else {
            args.push(testData.filePath);
        }
        
        return args;
    }
    
    /**
     * Check if required test dependencies are available
     */
    static async checkTestDependencies(): Promise<{
        pytest: boolean;
        gtest: boolean;
        launchTesting: boolean;
    }> {
        const result = {
            pytest: false,
            gtest: false,
            launchTesting: false
        };
        
        // Check for pytest availability
        try {
            const util = require('util');
            const exec = util.promisify(require('child_process').exec);
            await exec('python -c "import pytest"');
            result.pytest = true;
        } catch (error) {
            console.log('pytest not available:', error.message);
        }
        
        // Check if we can run ROS 2 tests (which handles gtest internally)
        try {
            const util = require('util');
            const exec = util.promisify(require('child_process').exec);
            const extension = await import('../extension');
            
            // Use the ROS environment
            const rosEnv = await extension.resolvedEnv();
            
            // Check if colcon test command is available
            try {
                await exec('colcon test --help', { env: rosEnv });
                result.gtest = true; // colcon can handle gtest
            } catch {
                // Fallback: check if ros2 command is available
                try {
                    await exec('ros2 --help', { env: rosEnv });
                    result.gtest = true; // ros2 ecosystem includes testing support
                } catch {
                    result.gtest = false;
                }
            }
        } catch (error) {
            console.log('ROS 2 test infrastructure check failed:', error.message);
            result.gtest = false;
        }
        
        // Check for launch_testing by attempting to import it
        try {
            const util = require('util');
            const exec = util.promisify(require('child_process').exec);
            await exec('python -c "import launch_testing"');
            result.launchTesting = true;
        } catch (error) {
            console.log('launch_testing not available:', error.message);
        }
        
        return result;
    }
}