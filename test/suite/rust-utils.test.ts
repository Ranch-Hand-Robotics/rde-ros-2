/**
 * Rust Utils Test
 * 
 * Test Rust package detection and utility functions.
 */

import * as assert from 'assert';
import * as fs from 'fs';
import * as path from 'path';
import * as os from 'os';
import { isRustPackage } from '../../src/ros/rust-utils';

describe('Rust Utils', () => {
    let tempDir: string;

    beforeEach(() => {
        // Create a temporary directory for testing
        tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'rust-utils-test-'));
    });

    afterEach(() => {
        // Clean up temporary directory
        if (fs.existsSync(tempDir)) {
            fs.rmSync(tempDir, { recursive: true, force: true });
        }
    });

    describe('isRustPackage', () => {
        it('should return true when Cargo.toml exists', () => {
            // Create a Cargo.toml file
            const cargoTomlPath = path.join(tempDir, 'Cargo.toml');
            fs.writeFileSync(cargoTomlPath, '[package]\nname = "test_package"\nversion = "0.1.0"\n');

            const result = isRustPackage(tempDir);
            assert.strictEqual(result, true);
        });

        it('should return false when Cargo.toml does not exist', () => {
            // Don't create Cargo.toml, just check empty directory
            const result = isRustPackage(tempDir);
            assert.strictEqual(result, false);
        });

        it('should return false for C++ package (has CMakeLists.txt)', () => {
            // Create a CMakeLists.txt file (typical for C++ ROS packages)
            const cmakePath = path.join(tempDir, 'CMakeLists.txt');
            fs.writeFileSync(cmakePath, 'cmake_minimum_required(VERSION 3.8)\nproject(test_package)\n');

            const result = isRustPackage(tempDir);
            assert.strictEqual(result, false);
        });

        it('should return true even when package.xml exists alongside Cargo.toml', () => {
            // Create both Cargo.toml and package.xml (valid Rust ROS package)
            const cargoTomlPath = path.join(tempDir, 'Cargo.toml');
            fs.writeFileSync(cargoTomlPath, '[package]\nname = "test_package"\nversion = "0.1.0"\n');

            const packageXmlPath = path.join(tempDir, 'package.xml');
            fs.writeFileSync(packageXmlPath, '<?xml version="1.0"?>\n<package format="3">\n  <name>test_package</name>\n</package>\n');

            const result = isRustPackage(tempDir);
            assert.strictEqual(result, true);
        });

        it('should return false for non-existent directory', () => {
            const nonExistentDir = path.join(tempDir, 'non-existent');
            const result = isRustPackage(nonExistentDir);
            assert.strictEqual(result, false);
        });
    });
});
