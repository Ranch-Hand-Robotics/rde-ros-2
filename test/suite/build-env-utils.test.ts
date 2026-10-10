/**
 * Build Environment Utils Test
 * 
 * Test path conversion from absolute to relative paths for VS Code configuration.
 */

import * as assert from 'assert';
import * as path from 'path';
import * as os from 'os';
import { promises as fs } from 'fs';
import * as vscode from 'vscode';
import { parse } from 'jsonc-parser';
import * as ros from '../../src/ros/ros';
import { makeWorkspaceRelative, createConfigFiles } from '../../src/ros/build-env-utils';

describe('ROS C++ configuration refresh', () => {
    let workspace: string;
    let filename: string;
    let restore: (() => void)[];
    let includeRoot: string;

    function replace(object: object, key: string, value: unknown): void {
        const descriptor = Object.getOwnPropertyDescriptor(object, key);
        Object.defineProperty(object, key, { configurable: true, value });
        restore.push(() => Object.defineProperty(object, key, descriptor));
    }

    beforeEach(async () => {
        restore = [];
        workspace = await fs.mkdtemp(path.join(os.tmpdir(), 'rde-cpp-refresh-'));
        filename = path.join(workspace, '.vscode', 'c_cpp_properties.json');
        includeRoot = path.join(os.tmpdir(), 'rolling', 'include');
        replace(vscode.workspace, 'rootPath', workspace);
        replace(vscode.workspace, 'getConfiguration', () => ({ get: () => ['existing-python-path'] }));
        replace(ros, 'rosApi', {
            getIncludeDirs: async () => [includeRoot],
            getWorkspaceIncludeDirs: async () => [path.join(workspace, 'src')],
        });
        await fs.mkdir(path.dirname(filename));
    });

    afterEach(async () => {
        restore.reverse().forEach(undo => undo());
        await fs.rm(workspace, { recursive: true, force: true });
    });

    it('refreshes existing ROS paths and preserves comments, compiler settings and other configurations', async () => {
        await fs.writeFile(filename, `{
  // Keep custom settings
  "configurations": [
    { "name": "custom", "includePath": ["custom-include"] },
    { "name": "ros2", "includePath": ["stale-ros-include"], "compilerPath": "/custom/compiler", "defines": ["USER_DEFINE"], "cppStandard": "c++20", "browse": { "limitSymbolsToIncludedHeaders": true } },
  ],
  "version": 4
}`);
        await createConfigFiles();
        includeRoot = path.join(os.tmpdir(), 'lyrical', 'include');
        await createConfigFiles();
        const content = await fs.readFile(filename, 'utf8');
        const configurations = parse(content).configurations;
        assert.ok(content.includes('// Keep custom settings'));
        assert.deepStrictEqual(configurations[0], { name: 'custom', includePath: ['custom-include'] });
        assert.ok(configurations[1].includePath.includes(path.join(includeRoot, '**')));
        assert.ok(configurations[1].includePath.includes('${workspaceFolder}/src/**'));
        assert.ok(!content.includes('stale-ros-include'));
        assert.ok(!configurations[1].includePath.some((entry: string) => entry.includes('rolling')));
        assert.strictEqual(configurations[1].compilerPath, '/custom/compiler');
        assert.strictEqual(configurations[1].cppStandard, 'c++20');
        assert.deepStrictEqual(configurations[1].defines, ['USER_DEFINE']);
        assert.deepStrictEqual(configurations[1].browse, { limitSymbolsToIncludedHeaders: true });
    });

    it('leaves unmanaged and invalid configuration files untouched', async () => {
        const custom = '{ "configurations": [{ "name": "custom" }], "version": 4 }';
        await fs.writeFile(filename, custom);
        await createConfigFiles();
        assert.strictEqual(await fs.readFile(filename, 'utf8'), custom);
        await fs.writeFile(filename, '{ invalid');
        await assert.rejects(createConfigFiles(), /invalid C\+\+ configuration/);
        assert.strictEqual(await fs.readFile(filename, 'utf8'), '{ invalid');
    });

    it('awaits creation of a missing ROS configuration and skips empty windows', async () => {
        await createConfigFiles();
        assert.strictEqual(parse(await fs.readFile(filename, 'utf8')).configurations[0].name, 'ros2');
        replace(vscode.workspace, 'rootPath', undefined);
        await createConfigFiles();
    });
});

describe('Build Environment Utils - Path Conversion', () => {
    describe('makeWorkspaceRelative', () => {
        it('should convert workspace paths to relative', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws/build/my_package';
            
            const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
            
            assert.strictEqual(relativePath, 'build/my_package');
        });

        it('should handle install directory paths', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws/install/my_package/lib/python3.12/site-packages';
            
            const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
            
            assert.strictEqual(relativePath, 'install/my_package/lib/python3.12/site-packages');
        });

        it('should keep external paths as absolute', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/opt/ros/humble/lib/python3.12/site-packages';
            
            const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
            
            // External paths should remain unchanged
            assert.strictEqual(relativePath, absolutePath);
        });

        it('should handle workspace root itself', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws';
            
            const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
            
            // Workspace root should return empty string
            assert.strictEqual(relativePath, '');
        });

        it('should handle null/undefined inputs', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            
            assert.strictEqual(makeWorkspaceRelative(null, workspaceRoot), "");
            assert.strictEqual(makeWorkspaceRelative(undefined, workspaceRoot), "");
            assert.strictEqual(makeWorkspaceRelative('/some/path', null), '/some/path');
            assert.strictEqual(makeWorkspaceRelative('/some/path', undefined), '/some/path');
        });

        it('should handle paths with same prefix but different root', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_workspace/build/my_package';
            
            const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
            
            // These should not be considered the same workspace
            assert.strictEqual(relativePath, absolutePath);
        });

        it('should handle Windows paths', () => {
            if (process.platform === 'win32') {
                const workspaceRoot = 'C:\\Users\\user\\ros2_ws';
                const absolutePath = 'C:\\Users\\user\\ros2_ws\\build\\my_package';
                
                const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
                
                assert.strictEqual(relativePath, 'build\\my_package');
            } else {
                // Skip on non-Windows
                assert.ok(true);
            }
        });

        it('should normalize paths before comparison', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws//build/../build/my_package';
            
            const relativePath = makeWorkspaceRelative(absolutePath, workspaceRoot);
            
            // Should normalize and convert correctly
            assert.strictEqual(relativePath, 'build/my_package');
        });
    });
});
