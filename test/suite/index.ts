/**
 * Test Suite Index
 * 
 * This file configures and runs the Mocha test suite for the VS Code extension.
 */

import * as path from 'path';
import * as Mocha from 'mocha';
import { glob } from 'glob';

export function run(): Promise<void> {
    // Create the mocha test
    const mocha = new Mocha({
        ui: 'bdd',
        color: true,
        timeout: 20000, // 20 second timeout for tests
        reporter: 'mochawesome',
        reporterOptions: {
            reportDir: path.resolve(__dirname, '../../test-results'),
            reportFilename: 'mochawesome',
            overwrite: true,
            quiet: true,
            html: false,
            json: true
        }
    });

    const testsRoot = path.resolve(__dirname, '..');

    return new Promise(async (c, e) => {
        try {
            const files = await glob('**/**.test.js', { cwd: testsRoot });
            
            // Add files to the test suite
            files.forEach(f => mocha.addFile(path.resolve(testsRoot, f)));

            // Run the mocha test
            mocha.run(failures => {
                if (failures > 0) {
                    e(new Error(`${failures} tests failed.`));
                } else {
                    c();
                }
            });
        } catch (err) {
            console.error(err);
            e(err);
        }
    });
}
