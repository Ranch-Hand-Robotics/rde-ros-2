/**
 * Colcon Utils Test
 * 
 * Test the colcon utilities for package discovery and ignore management.
 */

import * as assert from 'assert';
import * as vscode from 'vscode';
import * as path from 'path';
import { getColconIgnoreConfig, updateColconIgnoreConfig } from '../../src/build-tool/colcon-utils';

describe('Colcon Utils Tests', () => {
    let testWorkspaceUri: vscode.Uri;

    before(async () => {
        // Get workspace folder
        const workspaceFolders = vscode.workspace.workspaceFolders;
        if (workspaceFolders && workspaceFolders.length > 0) {
            testWorkspaceUri = workspaceFolders[0].uri;
        }
    });

    after(async () => {
        // Clean up configuration
        const config = vscode.workspace.getConfiguration("ROS2");
        await config.update("colconIgnore", {}, vscode.ConfigurationTarget.Workspace);
    });

    describe('getColconIgnoreConfig', () => {
        it('should return empty object when no packages are ignored', () => {
            const ignoreConfig = getColconIgnoreConfig();
            assert.strictEqual(typeof ignoreConfig, 'object');
        });
    });

    describe('updateColconIgnoreConfig', () => {
        it('should add a package to ignore list', async () => {
            await updateColconIgnoreConfig('test_package', true);
            const ignoreConfig = getColconIgnoreConfig();
            assert.strictEqual(ignoreConfig['test_package'], true);
        });

        it('should remove a package from ignore list', async () => {
            await updateColconIgnoreConfig('test_package', true);
            let ignoreConfig = getColconIgnoreConfig();
            assert.strictEqual(ignoreConfig['test_package'], true);

            await updateColconIgnoreConfig('test_package', false);
            ignoreConfig = getColconIgnoreConfig();
            assert.strictEqual(ignoreConfig['test_package'], undefined);
        });

        it('should handle multiple packages', async () => {
            await updateColconIgnoreConfig('package1', true);
            await updateColconIgnoreConfig('package2', true);
            
            const ignoreConfig = getColconIgnoreConfig();
            assert.strictEqual(ignoreConfig['package1'], true);
            assert.strictEqual(ignoreConfig['package2'], true);
        });
    });
});
