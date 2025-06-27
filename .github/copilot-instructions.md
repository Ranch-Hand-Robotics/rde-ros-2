# General
Create test cases in the test directory for all new features and bug fixes.
Do not create summary documents.
If you need to execute ROS commands, source the ROS 2 setup script in the terminal before running the command. We're testing on Kilted, so use /opt/ros/kilted/setup.bash.
This project compiles typescript using `npm run build`


# Library specifications
* This is a Visual Studio Code extension which provides tools to help develop ROS 2 code, which includes rclpy, rclcpp, rclrust and rcldonet.
* The extension is written in TypeScript and uses the Visual Studio Code API.
* The extension includes python code which is called from the TypeScript code when directly interfacing with ROS 2.
* to support modern ubuntu, the extension attempts to manage a virtual environment for python code which lives in the extension directory.
* The extension works with the language debuggers, by attempting to identify the language of the file being debugged and then using the appropriate debugger. Currently this is only implemented for python and C++.
* This extension used to support ROS 1, so some ROS 1 code may be present in the extension. This code is not used by the extension, and may be removed if encoundered. Do not get confused by this code.
* The extension is designed to be used with ROS 2, and does not support ROS 1.
* The extension is designed to be used with the latest version of ROS 2, and does not support older versions of ROS 2.
* The extension is designed to be used with the latest version of Visual Studio Code, and does not support older versions of Visual Studio Code.
* The extension is designed to be used with the latest version of TypeScript, and does not support older versions of TypeScript.
* The extension is designed to be used with the latest version of Python, and does not support older versions of Python.
* The extension is designed to be used with the latest version of C++, and does not support older versions of C++.
* The extension is designed to be used with the latest version of Rust, and does not support older versions of Rust.
* The extension is designed to be used with the latest version of .NET, and does not support older versions of .NET.
* The extension is designed to be used with the latest version of Node.js, and does not support older versions of Node.js.

# ROS 2 Launch File Dumper
This extension provides a tool to dump the contents of a ROS 2 python based launch file to enable the extension to launch each ROS node under
a debugger. The file ros2_launch_dumper.py is included in the extension and run using the system python interpreter. 
Since launch files can be python files, the dumper iterates over the objects in the launch file and dumps important contents to the stdout which is parsed
by the extension.


# ROS 2 Lifecycle Node support
This extension supports ROS 2 Lifecycle Nodes, which are a type of node that can transition between different states. 
The extension provides tools to help develop and debug Lifecycle Nodes, including:
* The ability to set the initial state of a Lifecycle Node.
* The ability to transition a Lifecycle Node between states.
* The ability to view the current state of a Lifecycle Node.
* The ability to view the available transitions for a Lifecycle Node.
* During debugging, the extension dumps the contents of the launch file. If the launch file contains Lifecycle Nodes, the extension will automatically set the initial state of the Lifecycle Node to "unconfigured" and will allow you to transition the node to "inactive", "active", and "finalized" states.
* If the launch file contains launch file event emitters, these are exported from the dumper, and after starting the debuggers.

These abilities are exposed through the command palette, and through the ROS 2 Status Webview.
