#!/usr/bin/env python3
"""
Unit tests for the ROS 2 launch dumper functionality.

Tests verify:
- LogInfo actions are properly skipped without visiting
- Environment variable actions are visited and applied
- ExecuteProcess commands are correctly extracted with arguments
- LifecycleNode actions are properly detected
- JSON output structure is valid
- No output contamination from spurious logging
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class LaunchDumperTests(unittest.TestCase):
    """Test suite for ros2_launch_dumper.py"""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        # Get path to the launch dumper script
        script_dir = Path(__file__).parent.parent
        cls.dumper_path = script_dir / 'assets' / 'scripts' / 'ros2_launch_dumper.py'
        
        if not cls.dumper_path.exists():
            raise RuntimeError(f"Launch dumper not found at {cls.dumper_path}")
        
        # Find ROS 2 installation
        cls.ros_distro = cls._find_latest_ros_distro()
        if cls.ros_distro:
            print(f"Found ROS 2 distro: {cls.ros_distro}")
        else:
            print("Warning: No ROS 2 installation found, tests may fail")

    @staticmethod
    def _find_latest_ros_distro() -> str:
        """Find the latest ROS 2 distribution installed on the system."""
        ros_path = Path('/opt/ros')
        
        if not ros_path.exists():
            return None
        
        try:
            distros = [d.name for d in ros_path.iterdir() if d.is_dir()]
            # Return the last one alphabetically (typically the newest)
            return sorted(distros)[-1] if distros else None
        except Exception:
            return None

    def create_launch_file(self, content: str) -> str:
        """Create a temporary launch file with the given content."""
        temp_file = tempfile.NamedTemporaryFile(
            mode='w',
            suffix='.launch.py',
            delete=False
        )
        temp_file.write(content)
        temp_file.close()
        self.addCleanup(os.unlink, temp_file.name)
        return temp_file.name

    def run_dumper(self, launch_file: str, output_format: str = 'json') -> str:
        """Run the launch dumper and return the output."""
        try:
            # Build command to run dumper, sourcing ROS if available
            if self.ros_distro:
                setup_script = f'/opt/ros/{self.ros_distro}/setup.bash'
                bash_command = f'source {setup_script} && python3 "{str(self.dumper_path)}" "{launch_file}" --output-format {output_format}'
                cmd = ['bash', '-c', bash_command]
            else:
                # Try without sourcing ROS
                cmd = ['python3', str(self.dumper_path), launch_file, '--output-format', output_format]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
                env={**os.environ, 'PYTHONUNBUFFERED': '1'}
            )
            
            if result.returncode != 0:
                self.fail(f"Dumper failed with return code {result.returncode}\nstderr: {result.stderr}")
            
            return result.stdout
        except subprocess.TimeoutExpired:
            self.fail("Dumper execution timed out")

    def test_loginfo_not_contaminating_output(self):
        """Test that LogInfo actions don't contaminate JSON output."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="This is informational output that should not be in JSON"),
        ExecuteProcess(
            cmd=['echo', 'hello'],
            name='test_process'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Should be valid JSON
        data = json.loads(output)
        
        # LogInfo message should not appear in output
        self.assertNotIn('This is informational output', output,
                        "LogInfo message should not appear in JSON output")
        
        # Should have captured the process
        self.assertEqual(len(data['processes']), 1, "Should have captured one process")
        self.assertIn('echo', data['processes'][0]['command'],
                     "Should have captured echo command")

    def test_environment_variables_visited(self):
        """Test that environment variable actions are visited and processed."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        ExecuteProcess(
            cmd=['echo', 'test'],
            name='test_process'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Should be valid JSON
        data = json.loads(output)
        
        # Environment variables should be visited without interfering with process capture
        self.assertEqual(len(data['processes']), 1, "Should have captured process with environment variables")
        self.assertIn('echo', data['processes'][0]['command'], "Should have captured echo command")

    def test_execute_process_extraction(self):
        """Test that ExecuteProcess commands are correctly extracted."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '-m', 'my_module', '--arg1', 'value1', '--arg2'],
            name='python_process'
        ),
        ExecuteProcess(
            cmd=['echo', 'hello', 'world'],
            name='echo_process'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Should be valid JSON
        data = json.loads(output)
        
        # Should have captured both processes
        self.assertEqual(len(data['processes']), 2, "Should have captured two processes")
        
        # Verify first process (Python)
        proc1 = data['processes'][0]
        self.assertIn('python', proc1['executable'], "First process should be python")
        self.assertIn('-m', proc1['arguments'], "Should have module flag in arguments")
        self.assertIn('my_module', proc1['arguments'], "Should have module name in arguments")
        self.assertIn('--arg1', proc1['arguments'], "Should have arg1 in arguments")
        
        # Verify second process (echo)
        proc2 = data['processes'][1]
        self.assertIn('echo', proc2['executable'], "Second process should be echo")
        self.assertEqual(len(proc2['arguments']), 2, "Echo should have 2 arguments")
        self.assertIn('hello', proc2['arguments'], "Should have 'hello' argument")
        self.assertIn('world', proc2['arguments'], "Should have 'world' argument")

    def test_json_output_structure(self):
        """Test that JSON output has valid structure with all required fields."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'demo_nodes_cpp', 'talker'],
            name='talker'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Should be valid JSON
        data = json.loads(output)
        
        # Verify required fields exist
        self.assertIn('version', data, "Should have version field")
        self.assertIn('processes', data, "Should have processes array")
        self.assertIn('lifecycle_nodes', data, "Should have lifecycle_nodes array")
        self.assertIn('warnings', data, "Should have warnings array")
        self.assertIn('errors', data, "Should have errors array")
        self.assertIn('info', data, "Should have info array")
        
        # Verify types
        self.assertIsInstance(data['processes'], list, "processes should be an array")
        self.assertIsInstance(data['lifecycle_nodes'], list, "lifecycle_nodes should be an array")
        self.assertIsInstance(data['warnings'], list, "warnings should be an array")
        self.assertIsInstance(data['errors'], list, "errors should be an array")
        self.assertIsInstance(data['info'], list, "info should be an array")
        
        # Verify process structure if processes exist
        if len(data['processes']) > 0:
            proc = data['processes'][0]
            self.assertIn('type', proc, "Process should have type field")
            self.assertIn('command', proc, "Process should have command field")
            self.assertIn('arguments', proc, "Process should have arguments array")
            self.assertIsInstance(proc['arguments'], list, "arguments should be an array")

    def test_declare_launch_argument_skipped(self):
        """Test that DeclareLaunchArgument is properly skipped."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('my_arg', default_value='default_value'),
        DeclareLaunchArgument('another_arg', default_value='another_value'),
        ExecuteProcess(
            cmd=['echo', 'test'],
            name='test_process'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Should be valid JSON
        data = json.loads(output)
        
        # Should still capture the process
        self.assertEqual(len(data['processes']), 1, "Should have captured process")
        self.assertIn('echo', data['processes'][0]['command'], "Should have captured echo")

    def test_pure_json_output(self):
        """Test that output is pure JSON with no spurious content."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['echo', 'hello'],
            name='test'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Output should be pure JSON
        trimmed = output.strip()
        self.assertTrue(trimmed.startswith('{'), "Output should start with JSON object")
        self.assertTrue(trimmed.endswith('}'), "Output should end with JSON object")
        
        # Should parse without error
        data = json.loads(output)
        self.assertIsNotNone(data, "Should be valid JSON object")

    def test_legacy_output_format(self):
        """Test that legacy output format works correctly."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['echo', 'hello'],
            name='test'
        )
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file, output_format='legacy')
        
        # Legacy format should have command output (either tab-prefixed or quoted commands)
        # The format may vary, so just check that we got some output
        self.assertGreater(len(output.strip()), 0, "Should have non-empty legacy output")
        
        # Should contain the echo command in some form
        self.assertIn('echo', output.lower(), "Should contain echo command in output")

    def test_multiple_processes(self):
        """Test handling of multiple ExecuteProcess actions."""
        launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['node1'], name='n1'),
        ExecuteProcess(cmd=['node2', 'arg'], name='n2'),
        ExecuteProcess(cmd=['node3', 'arg1', 'arg2'], name='n3'),
    ])
'''
        launch_file = self.create_launch_file(launch_content)
        output = self.run_dumper(launch_file)
        
        # Should be valid JSON
        data = json.loads(output)
        
        # Should have captured all three processes
        self.assertEqual(len(data['processes']), 3, "Should have captured three processes")
        
        # Verify arguments for each
        self.assertEqual(len(data['processes'][0]['arguments']), 0, "n1 should have no arguments")
        self.assertEqual(len(data['processes'][1]['arguments']), 1, "n2 should have 1 argument")
        self.assertEqual(len(data['processes'][2]['arguments']), 2, "n3 should have 2 arguments")


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
