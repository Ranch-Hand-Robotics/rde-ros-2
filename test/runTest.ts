/**
 * VS Code Extension Test Runner
 * 
 * This file is the entry point for running VS Code extension tests.
 * It uses @vscode/test-electron to download and run VS Code with the extension loaded.
 */

import * as path from 'path';
import { runTests } from '@vscode/test-electron';

async function main() {
    try {
        // The folder containing the Extension Manifest package.json
        // Passed to `--extensionDevelopmentPath`
        const extensionDevelopmentPath = path.resolve(__dirname, '../../');

        // The path to test runner
        // Passed to --extensionTestsPath
        const extensionTestsPath = path.resolve(__dirname, './suite/index');

        // Download VS Code, unzip it and run the integration test
        await runTests({ 
            extensionDevelopmentPath, 
            extensionTestsPath,
            launchArgs: [
                '--disable-extensions', // Disable other extensions during testing
                '--disable-workspace-trust' // Skip workspace trust dialog
            ]
        });
    } catch (err) {
        console.error('Failed to run tests');
        process.exit(1);
    }
}

main();
