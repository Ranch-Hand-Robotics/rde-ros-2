/**
 * Build Environment Utils Test
 * 
 * Test path conversion from absolute to relative paths for VS Code configuration.
 */

import * as assert from 'assert';
import * as path from 'path';

describe('Build Environment Utils - Path Conversion', () => {
    describe('makeWorkspaceRelative logic', () => {
        it('should convert workspace paths to relative', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws/build/my_package';
            
            const relativePath = path.relative(workspaceRoot, absolutePath);
            
            assert.strictEqual(relativePath, 'build/my_package');
        });

        it('should handle install directory paths', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws/install/my_package/lib/python3.12/site-packages';
            
            const relativePath = path.relative(workspaceRoot, absolutePath);
            
            assert.strictEqual(relativePath, 'install/my_package/lib/python3.12/site-packages');
        });

        it('should keep external paths as absolute', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/opt/ros/humble/lib/python3.12/site-packages';
            
            // For paths outside workspace, they should remain absolute
            // We can detect this by checking if the path starts with the workspace
            const isInWorkspace = absolutePath.startsWith(workspaceRoot);
            
            assert.strictEqual(isInWorkspace, false);
        });

        it('should handle C++ include paths with workspaceFolder variable', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws/install/my_package/include';
            
            const relativePath = path.relative(workspaceRoot, absolutePath);
            const workspaceFolderPath = '${workspaceFolder}/' + relativePath + '/**';
            
            assert.strictEqual(workspaceFolderPath, '${workspaceFolder}/install/my_package/include/**');
        });

        it('should handle Windows paths', () => {
            if (process.platform === 'win32') {
                const workspaceRoot = 'C:\\Users\\user\\ros2_ws';
                const absolutePath = 'C:\\Users\\user\\ros2_ws\\build\\my_package';
                
                const relativePath = path.relative(workspaceRoot, absolutePath);
                
                assert.strictEqual(relativePath, 'build\\my_package');
            } else {
                // Skip on non-Windows
                assert.ok(true);
            }
        });

        it('should handle paths with same prefix but different root', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_workspace/build/my_package';
            
            // These should not be considered the same workspace
            const workspaceWithSep = workspaceRoot.endsWith(path.sep) 
                ? workspaceRoot 
                : workspaceRoot + path.sep;
            const isInWorkspace = path.normalize(absolutePath).startsWith(workspaceWithSep) 
                || path.normalize(absolutePath) === path.normalize(workspaceRoot);
            
            assert.strictEqual(isInWorkspace, false);
        });

        it('should correctly identify workspace root itself', () => {
            const workspaceRoot = '/home/user/ros2_ws';
            const absolutePath = '/home/user/ros2_ws';
            
            // The workspace root itself should be considered "in workspace"
            const workspaceWithSep = workspaceRoot.endsWith(path.sep) 
                ? workspaceRoot 
                : workspaceRoot + path.sep;
            const isInWorkspace = path.normalize(absolutePath).startsWith(workspaceWithSep) 
                || path.normalize(absolutePath) === path.normalize(workspaceRoot);
            
            assert.strictEqual(isInWorkspace, true);
        });
    });
});
