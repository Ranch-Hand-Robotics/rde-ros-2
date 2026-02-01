# Rust ROS 2 Support

The Robot Developer Extensions for ROS 2 provides comprehensive support for developing and debugging ROS 2 nodes written in Rust using the [rclrs](https://github.com/ros2-rust/ros2_rust) client library.

## Features

- **Automatic Rust Package Detection**: The extension automatically detects ROS 2 packages written in Rust by identifying `Cargo.toml` files.
- **Debugging Support**: Debug Rust ROS 2 nodes using the CodeLLDB extension with full breakpoint and inspection capabilities.
- **Workspace Initialization**: One-click setup of ROS 2 Rust workspace with all required dependencies.
- **Version Checking**: Automatic verification of Rust installation and minimum version requirements.

## Prerequisites

### Required Software

1. **ROS 2** (Humble, Jazzy, Kilted, Lyrical, or Rolling)
2. **Rust** (version 1.70.0 or higher)
   - Install from [https://rustup.rs/](https://rustup.rs/)
3. **colcon Cargo Plugins**
   ```bash
   pip install colcon-cargo colcon-ros-cargo
   ```

### Required VS Code Extensions

- **CodeLLDB** (`vadimcn.vscode-lldb`) - Required for Rust debugging
  - Install from the VS Code marketplace

Optionally, you can also install:
- **rust-analyzer** (`rust-lang.rust-analyzer`) - Provides IDE features for Rust

## Configuration

### Custom Rust Paths

If you have Rust installed in a non-standard location, you can configure the paths to `rustc` and `cargo`:

1. Open VS Code Settings (`Ctrl+,` or `Cmd+,`)
2. Search for "ROS2 Rust"
3. Set the following options:
   - **ROS2: Rustc Path** - Path to the `rustc` executable (default: `rustc`)
   - **ROS2: Cargo Path** - Path to the `cargo` executable (default: `cargo`)

Example configuration in `settings.json`:
```json
{
  "ROS2.rustcPath": "/usr/local/rust/bin/rustc",
  "ROS2.cargoPath": "/usr/local/rust/bin/cargo"
}
```

## Getting Started

### 1. Initialize ROS 2 Rust Workspace

The extension provides a command to set up your workspace with all necessary ROS 2 Rust dependencies:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Run: `ROS2: Initialize ROS 2 Rust Workspace`

This command will:
- Check if Rust is installed and meets minimum version requirements (1.70.0+)
- Distinguish between Rust not being installed vs. installed but wrong version
- Prompt you to install/update Rust if needed via a ROS terminal
- Clone required repositories based on your ROS distribution into the `src` directory:
  - For **Rolling/Kilted/Lyrical**: `rosidl_rust`
  - For **Jazzy/Humble**: Common interface packages + `rosidl_rust`

**Note**: Repository cloning is done through a ROS 2 terminal to handle any sudo requirements or permissions issues.

### 2. Create a Rust ROS 2 Package

Create a new Rust package in your workspace:

```bash
cd ~/ros2_ws/src
cargo new --bin my_rust_node
```

Add a `package.xml` file with ROS 2 metadata:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_rust_node</name>
  <version>0.1.0</version>
  <description>My Rust ROS 2 node</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>cargo</buildtool_depend>

  <depend>rclrs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

Add `rclrs` dependency to `Cargo.toml`:

```toml
[dependencies]
rclrs = "0.7"
```

### 3. Build Your Rust Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_rust_node
```

### 4. Debug Your Rust Node

The extension supports debugging Rust nodes in two ways:

#### Debug with Launch File

1. Create a launch file that starts your Rust node
2. Set breakpoints in your Rust source code
3. Press `F5` or use the Debug view to start debugging
4. The extension will automatically detect that the node is written in Rust and use CodeLLDB

Example `.vscode/launch.json`:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "ROS2: Launch Rust Node",
      "type": "ros2",
      "request": "launch",
      "target": "${workspaceFolder}/src/my_rust_pkg/launch/my_launch.py"
    }
  ]
}
```

#### Attach to Running Process

1. Start your Rust node normally: `ros2 run my_rust_node my_node`
2. Open the Command Palette and run: `Debug: Attach to Process`
3. Select your Rust node from the process list
4. Set breakpoints and debug

## How It Works

The extension detects Rust ROS 2 packages by:

1. Checking if a `Cargo.toml` file exists in the package directory
2. When launching a debug session, the extension analyzes the executable path
3. If the executable belongs to a package with `Cargo.toml`, it's identified as Rust
4. The extension creates an LLDB debug configuration specifically for Rust
5. CodeLLDB is used to provide full debugging capabilities

## Troubleshooting

### "Rust is not installed" Error

If you see this error when initializing the workspace:

1. Install Rust using rustup: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
2. Restart VS Code
3. Try the initialization command again

### "Rust debugging requires the CodeLLDB extension" Error

1. Install CodeLLDB from the VS Code marketplace
2. Reload VS Code
3. Try debugging again

### Build Errors

If you encounter build errors:

1. Ensure colcon plugins are installed: `pip install colcon-cargo colcon-ros-cargo`
2. Source your ROS 2 installation: `source /opt/ros/<distro>/setup.bash`
3. Clean and rebuild: `colcon build --cmake-clean-cache`

### Breakpoints Not Working

1. Ensure CodeLLDB extension is installed and enabled
2. Check that debug symbols are included in your build (default for Rust debug builds)
3. Verify that the source file paths match between your workspace and the installed package

## Example Package

A complete example Rust ROS 2 package is included in the extension samples:

- Location: `samples/rust/src/rde_rust/`
- Includes: Publisher and subscriber nodes
- Launch file: `minimal_pub_sub.launch.py`

You can use this as a reference for creating your own Rust ROS 2 packages.

## Additional Resources

- [ROS 2 Rust Documentation](https://github.com/ros2-rust/ros2_rust)
- [rclrs API Documentation](https://docs.rs/rclrs/)
- [Rust Book](https://doc.rust-lang.org/book/)
- [CodeLLDB Documentation](https://github.com/vadimcn/vscode-lldb)

## Known Limitations

- Rust ROS 2 support is currently in preview
- Not all ROS 2 features are available in rclrs yet (see [ros2_rust issues](https://github.com/ros2-rust/ros2_rust/issues))
- Debugging may require CodeLLDB to be installed separately

## Feedback

If you encounter issues or have suggestions for improving Rust support, please file an issue on our [GitHub repository](https://github.com/Ranch-Hand-Robotics/rde-ros-2/issues).
