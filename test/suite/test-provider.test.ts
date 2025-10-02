// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';
import { TestDiscoveryUtils } from '../../src/test-provider/test-discovery-utils';
import { TestType } from '../../src/test-provider/ros-test-provider';

suite('ROS 2 Test Provider Tests', () => {
    
    test('TestDiscoveryUtils.isTestFile identifies Python test files correctly', () => {
        // Positive cases
        assert.strictEqual(TestDiscoveryUtils.isTestFile('test_example.py').isTest, true);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('example_test.py').isTest, true);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('test/my_file.py').isTest, true);
        
        // Negative cases
        assert.strictEqual(TestDiscoveryUtils.isTestFile('example.py').isTest, false);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('src/main.py').isTest, false);
    });
    
    test('TestDiscoveryUtils.isTestFile identifies C++ test files correctly', () => {
        // Positive cases
        assert.strictEqual(TestDiscoveryUtils.isTestFile('test_example.cpp').isTest, true);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('exampleTest.cpp').isTest, true);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('test/my_file.cpp').isTest, true);
        
        // Negative cases
        assert.strictEqual(TestDiscoveryUtils.isTestFile('example.cpp').isTest, false);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('src/main.cpp').isTest, false);
    });
    
    test('TestDiscoveryUtils.isTestFile identifies launch test files correctly', () => {
        // Positive cases
        assert.strictEqual(TestDiscoveryUtils.isTestFile('test_example.launch.py').isTest, true);
        assert.strictEqual(TestDiscoveryUtils.isTestFile('example_test.launch.py').isTest, true);
        
        // Negative cases
        assert.strictEqual(TestDiscoveryUtils.isTestFile('example.launch.py').isTest, false);
    });
    
    test('TestDiscoveryUtils.parsePythonTestFile parses test methods correctly', () => {
        // Create a temporary test file
        const testContent = `
import unittest

class TestExample(unittest.TestCase):
    def test_simple(self):
        self.assertTrue(True)
    
    def test_with_params(self):
        self.assertEqual(1, 1)

def test_standalone():
    assert True

class NotATestClass:
    def not_a_test_method(self):
        pass
`;
        
        const tempFile = path.join(__dirname, 'temp_test.py');
        fs.writeFileSync(tempFile, testContent);
        
        try {
            const results = TestDiscoveryUtils.parsePythonTestFile(tempFile);
            
            // Should find the test class and methods
            const testClass = results.find(r => r.name === 'TestExample');
            assert.ok(testClass, 'Should find TestExample class');
            assert.strictEqual(testClass.type, 'class');
            
            const testMethods = results.filter(r => r.type === 'method');
            assert.strictEqual(testMethods.length, 3, 'Should find 3 test methods');
            
            const simpleTest = testMethods.find(m => m.name === 'test_simple');
            assert.ok(simpleTest, 'Should find test_simple method');
            assert.strictEqual(simpleTest.parent, 'TestExample');
            
            const standaloneTest = testMethods.find(m => m.name === 'test_standalone');
            assert.ok(standaloneTest, 'Should find test_standalone method');
            assert.strictEqual(standaloneTest.parent, undefined);
            
        } finally {
            // Clean up
            if (fs.existsSync(tempFile)) {
                fs.unlinkSync(tempFile);
            }
        }
    });
    
    test('TestDiscoveryUtils.parseCppTestFile parses GTest cases correctly', () => {
        const testContent = `
#include <gtest/gtest.h>

TEST(ExampleTest, SimpleTest) {
    EXPECT_TRUE(true);
}

TEST_F(ExampleFixture, FixtureTest) {
    EXPECT_EQ(1, 1);
}

// Not a test
void regular_function() {
    // Nothing
}
`;
        
        const tempFile = path.join(__dirname, 'temp_test.cpp');
        fs.writeFileSync(tempFile, testContent);
        
        try {
            const results = TestDiscoveryUtils.parseCppTestFile(tempFile);
            
            assert.strictEqual(results.length, 2, 'Should find 2 test cases');
            
            const simpleTest = results.find(r => r.name === 'SimpleTest');
            assert.ok(simpleTest, 'Should find SimpleTest');
            assert.strictEqual(simpleTest.suite, 'ExampleTest');
            assert.strictEqual(simpleTest.isFixture, false);
            
            const fixtureTest = results.find(r => r.name === 'FixtureTest');
            assert.ok(fixtureTest, 'Should find FixtureTest');
            assert.strictEqual(fixtureTest.suite, 'ExampleFixture');
            assert.strictEqual(fixtureTest.isFixture, true);
            
        } finally {
            // Clean up
            if (fs.existsSync(tempFile)) {
                fs.unlinkSync(tempFile);
            }
        }
    });
    
    test('TestDiscoveryUtils.getPythonTestCommand builds correct command', () => {
        const testData = {
            type: TestType.PythonUnitTest,
            filePath: '/path/to/test_file.py',
            testClass: 'TestClass',
            testMethod: 'test_method'
        };
        
        const command = TestDiscoveryUtils.getPythonTestCommand(testData);
        
        assert.ok(command.includes('python'), 'Should include python');
        assert.ok(command.includes('pytest'), 'Should include pytest');
        assert.ok(command.includes('/path/to/test_file.py::TestClass::test_method'), 'Should include correct test selector');
    });
});