/**
 * Build Environment Utils Test
 * 
 * Test path conversion from absolute to relative paths for VS Code configuration.
 */

import * as assert from 'assert';
import * as path from 'path';
import { makeWorkspaceRelative } from '../../src/ros/build-env-utils';

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
