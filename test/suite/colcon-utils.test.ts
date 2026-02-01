/**
 * Colcon Utils Test
 * 
 * Test the colcon utilities for package discovery and ignore management.
 */

import * as assert from 'assert';
import * as vscode from 'vscode';
import { getColconIgnoreConfig, updateColconIgnoreConfig, getPackages, findPackageForPath, getNonIgnoredPackages } from '../../src/build-tool/colcon-utils';

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

    describe('getPackages', () => {
        it('should return an array of packages', async function() {
            // Skip if no workspace or colcon not available
            if (!testWorkspaceUri) {
                this.skip();
                return;
            }

            const packages = await getPackages(testWorkspaceUri.fsPath);
            assert.ok(Array.isArray(packages));
            
            // Each package should have name and path
            packages.forEach(pkg => {
                assert.ok(pkg.name);
                assert.ok(pkg.path);
                assert.strictEqual(typeof pkg.name, 'string');
                assert.strictEqual(typeof pkg.path, 'string');
            });
        });

        it('should return empty array on error', async () => {
            // Use invalid path to trigger error
            const packages = await getPackages('/nonexistent/path/to/workspace');
            assert.ok(Array.isArray(packages));
            assert.strictEqual(packages.length, 0);
        });
    });

    describe('findPackageForPath', () => {
        it('should find package for valid path', async function() {
            // Skip if no workspace
            if (!testWorkspaceUri) {
                this.skip();
                return;
            }

            const packages = await getPackages(testWorkspaceUri.fsPath);
            if (packages.length === 0) {
                this.skip();
                return;
            }

            // Test with first package
            const firstPackage = packages[0];
            const packagePath = firstPackage.path.startsWith('/') 
                ? firstPackage.path 
                : `${testWorkspaceUri.fsPath}/${firstPackage.path}`;
            
            const foundPackage = await findPackageForPath(packagePath, testWorkspaceUri.fsPath);
            assert.strictEqual(foundPackage, firstPackage.name);
        });

        it('should return undefined for non-package path', async function() {
            // Skip if no workspace
            if (!testWorkspaceUri) {
                this.skip();
                return;
            }

            const foundPackage = await findPackageForPath('/tmp/not/a/package', testWorkspaceUri.fsPath);
            assert.strictEqual(foundPackage, undefined);
        });
    });

    describe('getNonIgnoredPackages', () => {
        it('should return all packages when none are ignored', async function() {
            // Skip if no workspace
            if (!testWorkspaceUri) {
                this.skip();
                return;
            }

            // Clear ignore list
            await updateColconIgnoreConfig('test_package', false);
            
            const allPackages = await getPackages(testWorkspaceUri.fsPath);
            const nonIgnored = await getNonIgnoredPackages(testWorkspaceUri.fsPath);
            
            assert.strictEqual(nonIgnored.length, allPackages.length);
        });

        it('should filter out ignored packages', async function() {
            // Skip if no workspace
            if (!testWorkspaceUri) {
                this.skip();
                return;
            }

            const allPackages = await getPackages(testWorkspaceUri.fsPath);
            if (allPackages.length === 0) {
                this.skip();
                return;
            }

            // Ignore first package
            const packageToIgnore = allPackages[0].name;
            await updateColconIgnoreConfig(packageToIgnore, true);
            
            const nonIgnored = await getNonIgnoredPackages(testWorkspaceUri.fsPath);
            
            assert.strictEqual(nonIgnored.length, allPackages.length - 1);
            assert.ok(!nonIgnored.includes(packageToIgnore));
            
            // Clean up
            await updateColconIgnoreConfig(packageToIgnore, false);
        });
    });
});
