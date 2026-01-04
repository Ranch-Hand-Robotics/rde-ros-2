/**
 * Launch Test
 * 
 * Simple test to verify the ROS 2 launch file dumper functionality.
 */

import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';
import { spawn } from 'child_process';

describe('Launch Dumper Test', () => {
    it('should run ros2_launch_dumper.py with test_launch.py and produce valid JSON output', async () => {
        // Get paths relative to workspace root
        // __dirname is out/test/suite, so we need to go up 3 levels to reach workspace root
        const workspaceRoot = path.join(__dirname, '../../../');
        const dumperScript = path.join(workspaceRoot, 'assets', 'scripts', 'ros2_launch_dumper.py');
        const testLaunchFile = path.join(workspaceRoot, 'test', 'launch', 'test_launch.py');

        // Verify files exist
        assert.ok(fs.existsSync(dumperScript), 'ros2_launch_dumper.py should exist');
        assert.ok(fs.existsSync(testLaunchFile), 'test_launch.py should exist');

        // Run the dumper script
        const result = await runDumper(dumperScript, testLaunchFile);
        
        // If ROS 2 is not available, skip the test with a warning
        if (result.exitCode !== 0 && result.stderr.includes('ModuleNotFoundError')) {
            // Check if it's specifically lark that's missing
            if (result.stderr.includes("No module named 'lark'")) {
                console.warn('lark-parser not available in ROS 2 environment. Install with:');
                console.warn('  sudo apt-get install -y python3-lark-parser');
                console.warn('  OR: pip install lark-parser');
            } else {
                console.warn('ROS 2 environment not available, skipping test');
                console.warn('stderr:', result.stderr);
            }
            return; // Skip test instead of failing
        }
        
        // Verify successful execution
        assert.strictEqual(result.exitCode, 0, `Dumper should exit successfully. stderr: ${result.stderr}`);
        assert.ok(result.stdout.length > 0, 'Should produce output');
        
        // Parse and verify JSON output
        let data;
        try {
            data = JSON.parse(result.stdout);
        } catch (error) {
            assert.fail(`Output should be valid JSON. Output: ${result.stdout}`);
        }
        
        // Verify expected JSON structure
        assert.ok(data.version, 'Should have version field');
        assert.ok(Array.isArray(data.processes), 'Should have processes array');
        assert.ok(Array.isArray(data.lifecycle_nodes), 'Should have lifecycle_nodes array');
        
        // Should have at least one process from test_launch.py
        assert.ok(data.processes.length > 0, 'Should have at least one process');
    });

    /**
     * Helper function to find the latest ROS 2 distribution
     */
    function findLatestRosDistro(): string | null {
        const fs = require('fs');
        const rosPath = '/opt/ros';
        
        try {
            if (!fs.existsSync(rosPath)) {
                return null;
            }
            
            const distros = fs.readdirSync(rosPath);
            // Return the last one alphabetically (typically the newest)
            return distros.length > 0 ? distros.sort().reverse()[0] : null;
        } catch (error) {
            return null;
        }
    }

    /**
     * Helper function to run the dumper script with ROS 2 environment sourced
     */
    async function runDumper(dumperScript: string, launchFile: string): Promise<{
        exitCode: number;
        stdout: string;
        stderr: string;
    }> {
        return new Promise((resolve, reject) => {
            const rosDistro = findLatestRosDistro();
            
            if (!rosDistro) {
                // If no ROS installation found, try running without sourcing
                console.warn('No ROS 2 installation found, attempting to run without sourcing');
            }
            
            // Create a bash command that sources ROS 2 and then runs the Python script
            const setupScript = rosDistro ? `/opt/ros/${rosDistro}/setup.bash` : '';
            const bashCommand = setupScript 
                ? `source ${setupScript} && python3 "${dumperScript}" "${launchFile}" --output-format json`
                : `python3 "${dumperScript}" "${launchFile}" --output-format json`;
            
            const process = spawn('bash', ['-c', bashCommand], {
                timeout: 30000 // 30 second timeout
            });

            let stdout = '';
            let stderr = '';

            process.stdout?.on('data', (data) => {
                stdout += data.toString();
            });

            process.stderr?.on('data', (data) => {
                stderr += data.toString();
            });

            process.on('close', (code) => {
                resolve({
                    exitCode: code || 0,
                    stdout,
                    stderr
                });
            });

            process.on('error', (error) => {
                reject(error);
            });
        });
    }
});
