# Rust ROS 2 Sample Package

This is a sample ROS 2 package written in Rust, demonstrating a simple publisher-subscriber pattern.

## Package Contents

- `minimal_publisher` - A node that publishes string messages to the "topic" topic
- `minimal_subscriber` - A node that subscribes to the "topic" topic and prints received messages

## Requirements

1. ROS 2 (Humble, Jazzy, Kilted, or Rolling)
2. Rust (1.70.0 or higher)
3. colcon-cargo and colcon-ros-cargo plugins
4. rclrs (ROS 2 Rust client library)

## Building

This package requires the ROS 2 Rust workspace to be initialized. You can do this using the VS Code command:

```
ROS2: Initialize ROS 2 Rust Workspace
```

Or manually by following the instructions at: https://github.com/ros2-rust/ros2_rust

Then build with colcon:

```bash
colcon build --packages-select rde_rust
```

## Running

Source your workspace and run the nodes:

```bash
# Terminal 1
source install/setup.bash
ros2 run rde_rust minimal_publisher

# Terminal 2
source install/setup.bash
ros2 run rde_rust minimal_subscriber
```

## Debugging

To debug Rust nodes in VS Code:

1. Ensure the CodeLLDB extension is installed (`vadimcn.vscode-lldb`)
2. Set breakpoints in your Rust source files
3. Use the ROS 2 debugger with a launch file or attach to a running process

The extension will automatically detect that this is a Rust package and use the appropriate debugger.
