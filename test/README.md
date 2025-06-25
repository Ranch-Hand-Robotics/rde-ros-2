# ROS 2 Extension Test Files

This directory contains test files for the ROS 2 VS Code extension.

## Directory Structure

- `test/` - Main test directory
  - `launch/` - Launch file tests for the dumper functionality
  - `test_launch_dumper.py` - Python test script for the launch dumper

## Launch Test Files

The `test/launch/` directory contains various ROS 2 launch files used to test the extension's launch file dumper functionality:

- **`simple_lifecycle_launch.py`** - Basic lifecycle node launch file
- **`test_basic_lifecycle_launch.py`** - Basic lifecycle node test case  
- **`test_events_launch.py`** - Launch file with event handlers and emitters
- **`test_launch.py`** - Standard ROS nodes (talker/listener demo)
- **`test_lifecycle_launch.py`** - Lifecycle node with event transitions
- **`test_lifecycle_simple.py`** - Simple lifecycle node test
- **`test_mixed_nodes.launch.py`** - Mixed regular and lifecycle nodes

## Running Tests

To test the launch dumper with these files:

```bash
# Test with a basic launch file
python3 assets/scripts/ros2_launch_dumper.py test/launch/test_launch.py --output-format json

# Test with a lifecycle node launch file  
python3 assets/scripts/ros2_launch_dumper.py test/launch/simple_lifecycle_launch.py --output-format json

# Test with mixed node types
python3 assets/scripts/ros2_launch_dumper.py test/launch/test_mixed_nodes.launch.py --output-format json
```

## Test Coverage

These launch files test:

- **Regular ROS nodes** (ExecuteProcess actions)
- **Lifecycle nodes** (LifecycleNode actions) 
- **Event handlers and emitters** (OnProcessStart, ChangeState)
- **Mixed node types** in a single launch file
- **Cross-platform compatibility** (Windows and Linux paths)
- **JSON and legacy output formats**
