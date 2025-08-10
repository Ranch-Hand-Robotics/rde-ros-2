# RDE Lifecycle C++ Node

A simple ROS 2 lifecycle node example written in C++.

## Overview

This package contains a basic lifecycle node that demonstrates the ROS 2 lifecycle node interface. The node implements all the required lifecycle callbacks and provides informative logging when state transitions occur.

## Building

To build this package, make sure you have a ROS 2 workspace set up and source it:

```bash
# Source your ROS 2 installation
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution

# Navigate to your workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select rde_lifecycle_cpp

# Source the workspace
source install/setup.bash
```

## Running

### Using the launch file (recommended)

```bash
ros2 launch rde_lifecycle_cpp rde_lifecycle_cpp.launch.py
```

### Running directly

```bash
ros2 run rde_lifecycle_cpp rde_lifecycle_cpp
```

## Lifecycle States

The node will start in the `unconfigured` state. You can use the following commands to transition between states:

- **Configure**: `ros2 lifecycle set /rde_lifecycle_cpp configure`
- **Activate**: `ros2 lifecycle set /rde_lifecycle_cpp activate`
- **Deactivate**: `ros2 lifecycle set /rde_lifecycle_cpp deactivate`
- **Cleanup**: `ros2 lifecycle set /rde_lifecycle_cpp cleanup`
- **Shutdown**: `ros2 lifecycle set /rde_lifecycle_cpp shutdown`

## Checking Node State

To see the current state of the lifecycle node:

```bash
ros2 lifecycle get /rde_lifecycle_cpp
```

## Available Transitions

To see what transitions are available from the current state:

```bash
ros2 lifecycle list /rde_lifecycle_cpp
```

## Package Structure

```
rde_lifecycle_cpp/
├── src/
│   └── rde_lifecycle_cpp.cpp    # Main node implementation
├── launch/
│   └── rde_lifecycle_cpp.launch.py  # Launch file
├── package.xml                   # Package manifest
├── CMakeLists.txt               # Build configuration
└── README.md                    # This file
```
