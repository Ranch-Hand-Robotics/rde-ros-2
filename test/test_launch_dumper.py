#!/usr/bin/env python3
"""
Test script for the ROS 2 launch dumper functionality.
This script can be used to test the JSON output format and event emitter support.
"""

import tempfile
import os
import sys
import subprocess
import json

def create_test_launch_file():
    """Create a simple test launch file for testing."""
    launch_content = '''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import ProcessStarted, ProcessExited
from launch_ros.actions import LifecycleNode
from launch_ros.events import StateTransition

def generate_launch_description():
    # Simple process node
    talker_node = ExecuteProcess(
        cmd=['ros2', 'run', 'demo_nodes_cpp', 'talker'],
        name='talker'
    )
    
    # Lifecycle node
    lifecycle_talker = LifecycleNode(
        package='lifecycle_msgs',
        executable='lifecycle_talker',
        name='lifecycle_talker_node',
        namespace='test',
        parameters=[{'use_sim_time': False}]
    )
    
    # Event handler
    on_talker_start = RegisterEventHandler(
        OnProcessStart(
            target_action=talker_node,
            on_start=[
                ExecuteProcess(
                    cmd=['echo', 'Talker started!'],
                    name='echo_talker_start'
                )
            ]
        )
    )
    
    # Event emitter
    emit_transition = EmitEvent(
        event=StateTransition(
            lifecycle_node_matcher=lifecycle_talker,
            transition_id=1  # configure
        )
    )
    
    return LaunchDescription([
        talker_node,
        lifecycle_talker,
        on_talker_start,
        emit_transition
    ])
'''
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.launch.py', delete=False) as f:
        f.write(launch_content)
        return f.name

def test_dumper_json_output():
    """Test the dumper with JSON output format."""
    launch_file = create_test_launch_file()
    
    try:
        # Get the dumper script path
        dumper_script = os.path.join(os.path.dirname(__file__), '..', 'assets', 'scripts', 'ros2_launch_dumper.py')
        
        # Run the dumper
        result = subprocess.run([
            'python3', dumper_script, launch_file, '--output-format', 'json'
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print(f"Dumper failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            return False
        
        # Try to parse JSON output
        try:
            data = json.loads(result.stdout)
            print("✓ JSON output parsed successfully")
            print(f"  - Processes: {len(data.get('processes', []))}")
            print(f"  - Lifecycle nodes: {len(data.get('lifecycle_nodes', []))}")
            print(f"  - Event handlers: {len(data.get('event_handlers', []))}")
            print(f"  - Events: {len(data.get('events', []))}")
            
            # Pretty print for debugging
            print("\\nJSON Output:")
            print(json.dumps(data, indent=2))
            
            return True
        except json.JSONDecodeError as e:
            print(f"✗ Failed to parse JSON output: {e}")
            print(f"Raw output: {result.stdout}")
            return False
            
    except subprocess.TimeoutExpired:
        print("✗ Dumper timed out")
        return False
    except Exception as e:
        print(f"✗ Test failed with exception: {e}")
        return False
    finally:
        # Clean up
        if os.path.exists(launch_file):
            os.unlink(launch_file)

def test_dumper_legacy_output():
    """Test the dumper with legacy output format."""
    launch_file = create_test_launch_file()
    
    try:
        # Get the dumper script path
        dumper_script = os.path.join(os.path.dirname(__file__), '..', 'assets', 'scripts', 'ros2_launch_dumper.py')
        
        # Run the dumper with legacy format
        result = subprocess.run([
            'python3', dumper_script, launch_file, '--output-format', 'legacy'
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print(f"Dumper failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            return False
        
        # Check for tab-prefixed output
        lines = result.stdout.strip().split('\\n')
        tab_lines = [line for line in lines if line.startswith('\\t')]
        
        print(f"✓ Legacy output format test completed")
        print(f"  - Tab-prefixed lines: {len(tab_lines)}")
        
        if tab_lines:
            print("\\nLegacy Output:")
            for line in tab_lines:
                print(f"  {line}")
        
        return True
        
    except subprocess.TimeoutExpired:
        print("✗ Dumper timed out")
        return False
    except Exception as e:
        print(f"✗ Test failed with exception: {e}")
        return False
    finally:
        # Clean up
        if os.path.exists(launch_file):
            os.unlink(launch_file)

if __name__ == '__main__':
    print("Testing ROS 2 launch dumper...")
    
    json_success = test_dumper_json_output()
    print()
    legacy_success = test_dumper_legacy_output()
    
    if json_success and legacy_success:
        print("\\n✓ All tests passed!")
        sys.exit(0)
    else:
        print("\\n✗ Some tests failed!")
        sys.exit(1)
