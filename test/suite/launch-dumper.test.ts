import * as assert from 'assert';
import * as cp from 'child_process';
import * as fs from 'fs';
import * as path from 'path';
import { promisify } from 'util';

const exec = promisify(cp.exec);

describe('ROS2 Launch Dumper Tests', () => {
    let dumperPath: string;
    let testDir: string;
    let rosAvailable = true;
    let rosMissingReason = '';

    before(async () => {
        // Get path to the launch dumper script
        const extensionRoot = path.join(__dirname, '../../../');
        dumperPath = path.join(extensionRoot, 'assets', 'scripts', 'ros2_launch_dumper.py');
        
        // Create temp directory for test files
        testDir = fs.mkdtempSync(path.join('/tmp', 'launch-dumper-tests-'));

        // Check if required ROS 2 Python packages are available
        try {
            await exec('python3 -c "import ament_index_python, launch"');
        } catch (error) {
            rosAvailable = false;
            rosMissingReason = error instanceof Error ? error.message : 'Unknown error';
            console.warn('ROS 2 Python packages not available, skipping launch dumper tests.');
            console.warn(rosMissingReason);
        }
    });

    after(() => {
        // Clean up test directory
        if (fs.existsSync(testDir)) {
            fs.rmSync(testDir, { recursive: true });
        }
    });

    /**
     * Create a simple test launch file
     */
    function createTestLaunchFile(content: string, name: string = 'test'): string {
        const filename = path.join(testDir, `${name}-${Date.now()}.launch.py`);
        fs.writeFileSync(filename, content);
        return filename;
    }

    /**
     * Test LogInfo actions are skipped and don't contaminate output
     */
    it('LogInfo actions should not produce output or side effects', async function () {
        if (!rosAvailable) {
            this.skip();
            return;
        }
        const launchContent = 'import launch\\nfrom launch import LaunchDescription\\n' +
            'from launch.actions import ExecuteProcess, LogInfo\\n\\n' +
            'def generate_launch_description():\\n' +
            '    return LaunchDescription([\\n' +
            '        LogInfo(msg="This should not be in JSON"),\\n' +
            '        ExecuteProcess(\\n' +
            '            cmd=["echo", "hello"],\\n' +
            '            name="test_process"\\n' +
            '        )\\n' +
            '    ])\\n';

        const launchFile = createTestLaunchFile(launchContent, 'loginfo');

        try {
            const cmd = `python3 "${dumperPath}" "${launchFile}" --output-format json`;
            const { stdout } = await exec(cmd, {
                timeout: 30000,
                env: { ...process.env, PYTHONUNBUFFERED: '1' }
            });

            const data = JSON.parse(stdout);

            // Verify JSON structure
            assert.ok(data.version, 'Should have version field');
            assert.ok(Array.isArray(data.processes), 'Should have processes array');

            // Verify LogInfo content is not in output
            assert.ok(!stdout.includes('This should not be in JSON'), 'LogInfo message should not appear in output');

            // Verify process was captured
            assert.strictEqual(data.processes.length, 1, 'Should have captured one process');
        } finally {
            fs.unlinkSync(launchFile);
        }
    });

    /**
     * Test ExecuteProcess commands are correctly extracted
     */
    it('ExecuteProcess commands should be extracted with arguments', async function () {
        if (!rosAvailable) {
            this.skip();
            return;
        }
        const launchContent = 'import launch\\nfrom launch import LaunchDescription\\n' +
            'from launch.actions import ExecuteProcess\\n\\n' +
            'def generate_launch_description():\\n' +
            '    return LaunchDescription([\\n' +
            '        ExecuteProcess(cmd=["echo", "hello", "world"], name="echo_test")\\n' +
            '    ])\\n';

        const launchFile = createTestLaunchFile(launchContent, 'extract');

        try {
            const cmd = `python3 "${dumperPath}" "${launchFile}" --output-format json`;
            const { stdout } = await exec(cmd, {
                timeout: 30000,
                env: { ...process.env, PYTHONUNBUFFERED: '1' }
            });

            const data = JSON.parse(stdout);

            assert.strictEqual(data.processes.length, 1, 'Should have captured one process');
            assert.ok(data.processes[0].executable.includes('echo'), 'Should have echo executable');
            assert.strictEqual(data.processes[0].arguments.length, 2, 'Should have 2 arguments');
        } finally {
            fs.unlinkSync(launchFile);
        }
    });

    /**
     * Test JSON output structure is valid
     */
    it('JSON output should have valid structure', async function () {
        if (!rosAvailable) {
            this.skip();
            return;
        }
        const launchContent = 'import launch\\nfrom launch import LaunchDescription\\n' +
            'from launch.actions import ExecuteProcess\\n\\n' +
            'def generate_launch_description():\\n' +
            '    return LaunchDescription([\\n' +
            '        ExecuteProcess(cmd=["echo", "test"], name="test")\\n' +
            '    ])\\n';

        const launchFile = createTestLaunchFile(launchContent, 'structure');

        try {
            const cmd = `python3 "${dumperPath}" "${launchFile}" --output-format json`;
            const { stdout } = await exec(cmd, {
                timeout: 30000,
                env: { ...process.env, PYTHONUNBUFFERED: '1' }
            });

            const data = JSON.parse(stdout);

            // Verify required fields
            assert.ok(data.version, 'Should have version');
            assert.ok(Array.isArray(data.processes), 'Should have processes array');
            assert.ok(Array.isArray(data.lifecycle_nodes), 'Should have lifecycle_nodes array');
            assert.ok(Array.isArray(data.warnings), 'Should have warnings array');
            assert.ok(Array.isArray(data.errors), 'Should have errors array');
            assert.ok(Array.isArray(data.info), 'Should have info array');
        } finally {
            fs.unlinkSync(launchFile);
        }
    });

    /**
     * Test output is pure JSON with no contamination
     */
    it('Output should be pure JSON with no spurious content', async function () {
        if (!rosAvailable) {
            this.skip();
            return;
        }
        const launchContent = 'import launch\\nfrom launch import LaunchDescription\\n' +
            'from launch.actions import ExecuteProcess\\n\\n' +
            'def generate_launch_description():\\n' +
            '    return LaunchDescription([\\n' +
            '        ExecuteProcess(cmd=["echo", "test"], name="test")\\n' +
            '    ])\\n';

        const launchFile = createTestLaunchFile(launchContent, 'purity');

        try {
            const cmd = `python3 "${dumperPath}" "${launchFile}" --output-format json`;
            const { stdout } = await exec(cmd, {
                timeout: 30000,
                env: { ...process.env, PYTHONUNBUFFERED: '1' }
            });

            // Output should be pure JSON
            const trimmed = stdout.trim();
            assert.ok(trimmed.startsWith('{'), 'Output should start with {');
            assert.ok(trimmed.endsWith('}'), 'Output should end with }');

            // Should parse without error
            const data = JSON.parse(stdout);
            assert.ok(data, 'Should be valid JSON');
        } finally {
            fs.unlinkSync(launchFile);
        }
    });

    /**
     * Test environment variables are properly handled
     */
    it('Environment variable actions should be processed correctly', async function () {
        if (!rosAvailable) {
            this.skip();
            return;
        }
        const launchContent = 'import launch\\nfrom launch import LaunchDescription\\n' +
            'from launch.actions import ExecuteProcess, SetEnvironmentVariable\\n\\n' +
            'def generate_launch_description():\\n' +
            '    return LaunchDescription([\\n' +
            '        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),\\n' +
            '        ExecuteProcess(cmd=["echo", "test"], name="test")\\n' +
            '    ])\\n';

        const launchFile = createTestLaunchFile(launchContent, 'envvar');

        try {
            const cmd = `python3 "${dumperPath}" "${launchFile}" --output-format json`;
            const { stdout } = await exec(cmd, {
                timeout: 30000,
                env: { ...process.env, PYTHONUNBUFFERED: '1' }
            });

            const data = JSON.parse(stdout);

            // Should still capture the process
            assert.strictEqual(data.processes.length, 1, 'Should have captured process despite env vars');
        } finally {
            fs.unlinkSync(launchFile);
        }
    });
});
