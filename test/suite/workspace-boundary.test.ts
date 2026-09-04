/**
 * Workspace Boundary Test
 * 
 * Tests to ensure the extension respects workspace boundaries and doesn't
 * scan parent directories outside the workspace.
 */

import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';
import * as os from 'os';
import { TestDiscoveryUtils } from '../../src/test-provider/test-discovery-utils';

describe('Workspace Boundary Tests', () => {
    let testDir: string;
    let workspaceDir: string;
    let packageDir: string;
    let testFile: string;

    before(() => {
        // Create a temporary directory structure for testing
        testDir = fs.mkdtempSync(path.join(os.tmpdir(), 'ros2-test-'));
        
        // Create a workspace directory
        workspaceDir = path.join(testDir, 'workspace');
        fs.mkdirSync(workspaceDir, { recursive: true });
        
        // Create a package directory inside workspace
        packageDir = path.join(workspaceDir, 'src', 'my_package');
        fs.mkdirSync(packageDir, { recursive: true });
        
        // Create package.xml in package directory
        const packageXml = `<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
</package>`;
        fs.writeFileSync(path.join(packageDir, 'package.xml'), packageXml);
        
        // Create a test subdirectory
        const testDirPath = path.join(packageDir, 'test');
        fs.mkdirSync(testDirPath, { recursive: true });
        
        // Create a test file
        testFile = path.join(testDirPath, 'test_example.py');
        fs.writeFileSync(testFile, 'def test_something(): pass');
        
        // Create a package.xml in a parent directory outside workspace
        // This should NOT be found when searching from within workspace
        const parentPackageXml = `<?xml version="1.0"?>
<package format="3">
  <name>parent_package</name>
  <version>0.0.1</version>
  <description>Parent package that should not be found</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
</package>`;
        fs.writeFileSync(path.join(testDir, 'package.xml'), parentPackageXml);
    });

    after(() => {
        // Clean up temporary directory
        if (fs.existsSync(testDir)) {
            fs.rmSync(testDir, { recursive: true, force: true });
        }
    });

    describe('findPackageName', () => {
        it('should find package.xml within the same directory tree', () => {
            const packageName = TestDiscoveryUtils.findPackageName(testFile);
            assert.strictEqual(packageName, 'my_package', 
                'Should find the package.xml in the package directory');
        });
    });

    describe('isTestFile', () => {
        it('should identify Python test files correctly', () => {
            const result = TestDiscoveryUtils.isTestFile(testFile);
            assert.strictEqual(result.isTest, true, 'Should identify as test file');
        });

        it('should identify test files by directory name', () => {
            const testInTestDir = path.join(packageDir, 'test', 'example.py');
            fs.writeFileSync(testInTestDir, 'def example(): pass');
            
            const result = TestDiscoveryUtils.isTestFile(testInTestDir);
            assert.strictEqual(result.isTest, true, 
                'Should identify files in test directory as test files');
            
            fs.unlinkSync(testInTestDir);
        });

        it('should not identify non-test files as tests', () => {
            const nonTestFile = path.join(packageDir, 'src', 'node.py');
            fs.mkdirSync(path.dirname(nonTestFile), { recursive: true });
            fs.writeFileSync(nonTestFile, 'def main(): pass');
            
            const result = TestDiscoveryUtils.isTestFile(nonTestFile);
            assert.strictEqual(result.isTest, false, 
                'Should not identify regular Python files as tests');
            
            fs.unlinkSync(nonTestFile);
        });
    });

    describe('parsePythonTestFile', () => {
        it('should parse test classes and methods', () => {
            const testContent = `
import unittest

class TestExample(unittest.TestCase):
    def test_method1(self):
        pass
    
    def test_method2(self):
        pass

def test_standalone():
    pass
`;
            const tempTestFile = path.join(packageDir, 'test', 'test_parse.py');
            fs.writeFileSync(tempTestFile, testContent);
            
            const results = TestDiscoveryUtils.parsePythonTestFile(tempTestFile);
            
            // Should find the test class
            const testClass = results.find(r => r.name === 'TestExample' && r.type === 'class');
            assert.ok(testClass, 'Should find TestExample class');
            
            // Should find test methods
            const method1 = results.find(r => r.name === 'test_method1' && r.type === 'method');
            assert.ok(method1, 'Should find test_method1');
            assert.strictEqual(method1.parent, 'TestExample', 'Method should have correct parent');
            
            // Should find standalone test function
            const standalone = results.find(r => r.name === 'test_standalone' && r.type === 'method');
            assert.ok(standalone, 'Should find standalone test function');
            
            fs.unlinkSync(tempTestFile);
        });
    });

    describe('parseCppTestFile', () => {
        it('should parse C++ gtest TEST and TEST_F macros', () => {
            const cppContent = `
#include <gtest/gtest.h>

TEST(TestSuite, TestCase1) {
  EXPECT_TRUE(true);
}

TEST_F(TestFixture, TestCase2) {
  EXPECT_EQ(1, 1);
}
`;
            const tempCppFile = path.join(packageDir, 'test', 'test_cpp.cpp');
            fs.writeFileSync(tempCppFile, cppContent);
            
            const results = TestDiscoveryUtils.parseCppTestFile(tempCppFile);
            
            // Should find TEST macro
            const test1 = results.find(r => r.name === 'TestCase1' && r.suite === 'TestSuite');
            assert.ok(test1, 'Should find TEST(TestSuite, TestCase1)');
            assert.strictEqual(test1.isFixture, false, 'TEST should not be a fixture');
            
            // Should find TEST_F macro
            const test2 = results.find(r => r.name === 'TestCase2' && r.suite === 'TestFixture');
            assert.ok(test2, 'Should find TEST_F(TestFixture, TestCase2)');
            assert.strictEqual(test2.isFixture, true, 'TEST_F should be a fixture');
            
            fs.unlinkSync(tempCppFile);
        });
    });
});
