/**
 * Test Suite Index
 * 
 * This file configures and runs the Mocha test suite for the VS Code extension.
 */

import * as path from 'path';
import * as fs from 'fs';
import * as Mocha from 'mocha';
import { glob } from 'glob';

export function run(): Promise<void> {
    // Ensure test-results directory exists in workspace root
    const testResultsDir = path.resolve(__dirname, '../../../test-results');
    fs.mkdirSync(testResultsDir, { recursive: true });
    
    // Create the mocha test
    const mocha = new Mocha({
        ui: 'bdd',
        color: true,
        timeout: 20000, // 20 second timeout for tests
        reporter: 'mocha-junit-reporter',
        reporterOptions: {
            mochaFile: path.resolve(testResultsDir, 'junit.xml'),
            toConsole: true
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
