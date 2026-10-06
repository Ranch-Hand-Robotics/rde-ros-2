/**
 * Launch Test
 * 
 * Simple test to verify the ROS 2 launch file dumper functionality.
 */

import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';
import { spawn } from 'child_process';
import { resolveRosPython } from '../../src/ros/python';

describe('Launch Dumper Test', () => {
    it('should run ros2_launch_dumper.py with test_launch.py and produce valid JSON output', async function () {
        // Windows test hosts must inherit a sourced native ROS/Pixi environment.
        // Never invoke a Windows Bash alias (which may silently enter WSL).
        if (process.platform === 'win32' && process.env.ROS_VERSION !== '2') {
            this.skip();
        }
        // Get paths relative to workspace root
        // __dirname is out/test/suite, so we need to go up 3 levels to reach workspace root
        const workspaceRoot = path.join(__dirname, '../../../');
        const dumperScript = path.join(workspaceRoot, 'assets', 'scripts', 'ros2_launch_dumper.py');
        const testLaunchFile = path.join(workspaceRoot, 'samples', 'launch_examples', 'launch','test_launch.py');

        // Verify files exist
        assert.ok(fs.existsSync(dumperScript), 'ros2_launch_dumper.py should exist');
        assert.ok(fs.existsSync(testLaunchFile), 'test_launch.py should exist');

        // Run the dumper script
        const result = await runDumper(dumperScript, testLaunchFile);
        
        // If ROS 2 is not available, skip the test with a warning
        if (process.platform !== 'win32' && result.exitCode !== 0 && result.stderr.includes('ModuleNotFoundError')) {
            // Check if it's specifically lark that's missing
            if (result.stderr.includes("No module named 'lark'")) {
                console.warn('lark-parser not available in ROS 2 environment. Install with:');
                console.warn('  sudo apt-get install -y python3-lark-parser');
                console.warn('  OR: pip install lark-parser');
            } else {
                console.warn('ROS 2 environment not available, skipping test');
                console.warn('stderr:', result.stderr);
            }
            this.skip();
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

    for (const format of ['json', 'legacy']) {
        for (const showLog of ['true', 'false']) {
            it(`keeps Unicode LogInfo and nested diagnostics out of ${format} stdout (show_log=${showLog})`, async function () {
                if (process.platform === 'win32' && process.env.ROS_VERSION !== '2') {
                    this.skip();
                }
                const root = path.join(__dirname, '../../../');
                const result = await runDumper(
                    path.join(root, 'assets/scripts/ros2_launch_dumper.py'),
                    path.join(root, 'test/launch/log_info.launch.py'), format, [`show_log:=${showLog}`]);
                if (process.platform !== 'win32' && result.exitCode !== 0 && result.stderr.includes('ModuleNotFoundError')) {
                    this.skip();
                }
                assert.strictEqual(result.exitCode, 0, result.stderr);
                if (format === 'json') {
                    const data = JSON.parse(result.stdout);
                    assert.strictEqual(data.version, '1.0');
                    assert.strictEqual(data.processes.length, 1);
                    assert.deepStrictEqual(data.errors, []);
                    assert.ok(data.processes[0].arguments.includes("print('must not execute')"));
                } else {
                    const lines = result.stdout.trimEnd().split(/\r?\n/);
                    assert.strictEqual(lines.length, 1, result.stdout);
                    assert.ok(lines[0].startsWith('\t'), result.stdout);
                }
                assert.ok(!result.stdout.includes('[INFO]'), result.stdout);
                assert.ok(!result.stdout.includes('diagnostic'), result.stdout);
                assert.ok(result.stderr.includes('🚀 Launching as Normal ROS Node'), result.stderr);
                assert.ok(result.stderr.includes(`nested diagnostic: ${showLog}`), result.stderr);
                assert.strictEqual(result.stderr.includes('conditional diagnostic'), showLog === 'true');
                for (const diagnostic of ['module', 'opaque', 'shutdown']) {
                    assert.ok(result.stderr.includes(`${diagnostic} diagnostic {not JSON}`), result.stderr);
                }
            });
        }
    }

    /**
     * Helper function to find the latest ROS 2 distribution
     */
    function findLatestRosDistro(): string | null {
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
    async function runDumper(dumperScript: string, launchFile: string, format = 'json', launchArgs: string[] = []): Promise<{
        exitCode: number;
        stdout: string;
        stderr: string;
    }> {
        const windows = process.platform === 'win32';
        const python = windows ? await resolveRosPython(process.env) : undefined;
        return new Promise((resolve, reject) => {
            const rosDistro = windows ? null : findLatestRosDistro();
            
            if (!windows && !rosDistro) {
                // If no ROS installation found, try running without sourcing
                console.warn('No ROS 2 installation found, attempting to run without sourcing');
            }
            
            // Create a bash command that sources ROS 2 and then runs the Python script
            const setupScript = rosDistro ? `/opt/ros/${rosDistro}/setup.bash` : '';
            const bashCommand = setupScript 
                ? `source ${setupScript} && python3 "${dumperScript}" "${launchFile}" ${launchArgs.join(' ')} --output-format ${format}`
                : `python3 "${dumperScript}" "${launchFile}" ${launchArgs.join(' ')} --output-format ${format}`;
            
            const child = spawn(windows ? python : 'bash', windows
                ? [dumperScript, launchFile, ...launchArgs, '--output-format', format] : ['-c', bashCommand], {
                env: process.env,
                windowsHide: true,
                timeout: 30000 // 30 second timeout
            });

            let stdout = '';
            let stderr = '';

            child.stdout?.setEncoding('utf8');
            child.stderr?.setEncoding('utf8');

            child.stdout?.on('data', (data) => {
                stdout += data.toString();
            });

            child.stderr?.on('data', (data) => {
                stderr += data.toString();
            });

            child.on('close', (code) => {
                resolve({
                    exitCode: code ?? -1,
                    stdout,
                    stderr
                });
            });

            child.on('error', (error) => {
                reject(error);
            });
        });
    }
});
