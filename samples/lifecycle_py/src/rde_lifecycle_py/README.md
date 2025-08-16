# RDE Lifecycle Python Node

A simple ROS 2 lifecycle node example written in Python.

## Overview

This package contains a basic lifecycle node that demonstrates the ROS 2 lifecycle node interface using Python. The node implements all the required lifecycle callbacks and provides informative logging when state transitions occur.

## Building

To build this package, make sure you have a ROS 2 workspace set up and source it:

```bash
# Source your ROS 2 installation
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution

# Navigate to your workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select rde_lifecycle_py

# Source the workspace
source install/setup.bash
```

## Running

### Using the launch file (recommended)

```bash
ros2 launch rde_lifecycle_py rde_lifecycle_py.launch.py
```

### Running directly

```bash
ros2 run rde_lifecycle_py rde_lifecycle_py
```

## Lifecycle States

The node will start in the `unconfigured` state. You can use the following commands to transition between states:

- **Configure**: `ros2 lifecycle set /rde_lifecycle_py configure`
- **Activate**: `ros2 lifecycle set /rde_lifecycle_py activate`
- **Deactivate**: `ros2 lifecycle set /rde_lifecycle_py deactivate`
- **Cleanup**: `ros2 lifecycle set /rde_lifecycle_py cleanup`
- **Shutdown**: `ros2 lifecycle set /rde_lifecycle_py shutdown`

## Checking Node State

To see the current state of the lifecycle node:

```bash
ros2 lifecycle get /rde_lifecycle_py
```

## Available Transitions

To see what transitions are available from the current state:

```bash
ros2 lifecycle list /rde_lifecycle_py
```

## Package Structure

```
rde_lifecycle_py/
├── src/
│   └── rde_lifecycle_py/
│       ├── __init__.py
│       └── rde_lifecycle_py.py    # Main node implementation
├── launch/
│   └── rde_lifecycle_py.launch.py  # Launch file
├── resource/
│   └── rde_lifecycle_py            # Resource marker
├── package.xml                      # Package manifest
├── setup.py                         # Python package setup
└── README.md                        # This file
```

## Python vs C++

This package demonstrates the same lifecycle functionality as the C++ version but uses Python's `ManagedLifecycleNode` class, which provides a more Pythonic interface to the ROS 2 lifecycle system.
