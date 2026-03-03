# Commands and Features

Once you have ROS 2 installed and a workspace open, you can access these powerful features:

## Command Palette

Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS) to open the command palette and type "ROS2" to see all available commands:

* **ROS2: Create Terminal** - Create a terminal with the ROS environment sourced
* **ROS2: Update C++ Properties** - Configure IntelliSense for ROS C++ development
* **ROS2: Update Python Path** - Configure IntelliSense for ROS Python development
* **ROS2: Show Status** - View the ROS core runtime status
* **ROS2: Install ROS Dependencies** - Run `rosdep` to install dependencies

## Debugging

The extension provides comprehensive debugging support:

* **Launch Debugging** - Debug entire launch files with multiple nodes
* **Attach Debugging** - Attach to running ROS nodes
* **Python & C++ Support** - Debug both Python (rclpy) and C++ (rclcpp) nodes
* **Breakpoints** - Set breakpoints in your source code

To debug a launch file:
1. Open a `.launch.py` file
2. Press `F5` or go to Run → Start Debugging
3. The extension will automatically configure and launch your nodes

## Test Explorer

The extension integrates with VS Code's Test Explorer:

* Automatically discovers ROS 2 tests
* Run individual tests or entire test suites
* Debug tests with breakpoints
* View test results inline

## IntelliSense

Get smart code completion and documentation:

* **Message Files** - Hover over message types to see field definitions
* **Go to Definition** - Jump to message/service/action definitions
* **Auto-completion** - Get suggestions for ROS message fields

## Build Integration

The extension automatically creates build tasks:

* Press `Ctrl+Shift+B` to build your workspace
* Uses `colcon` for building packages
* Integrated problem matcher for compiler errors

## Learn More

* [Debug Support Documentation](command:ROS2.openDocumentation?debug-support)
* [Test Explorer Guide](command:ROS2.openDocumentation?test-explorer)
* [IntelliSense Features](command:ROS2.openDocumentation?intellisense)
