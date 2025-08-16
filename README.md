# Robotics Development Extensions for ROS 2
This is a Visual Studio Code Extension that  provides debugging support for [Robot Operating System 2 (ROS 2)](http://ros.org) development ROS 2 on Windows, Linux and MacOS. The Robot Operating System is a trademark of Open Robotics.

> NOTE: This extension is rebranded and re-released by Ranch Hand Robotics, a company owned by the maintainer of the [ms-iot VSCode ROS Extension](https://github.com/ms-iot/vscode-ros) with permission from Microsoft. The source extension was split into 3 parts - [ROS 1](https://ranchhandrobotics.github.io/rde-ros-1/), [ROS 2](https://ranchhandrobotics.github.io/rde-ros-2/) and a [URDF editor](https://ranchhandrobotics.github.io/rde-urdf/).

## Features

* Automatic ROS environment configuration.
* Allows starting, stopping and viewing the ROS core status.
* Automatically create `colcon` build and test tasks.
* Run and Debug ROS Launch Files
* Resolve dependencies with `rosdep` shortcut
* Syntax highlighting for `.msg`, `.urdf` and other ROS files.
* Automatically add the ROS C++ include and Python import paths.
* Format C++ using the ROS `clang-format` style.
* Debug a single ROS node (C++ or Python) by [attaching to the process](https://ranchhandrobotics.com/rde-ros-2/debug-support.html#attach).
* Debug ROS nodes (C++ or Python) [launched from a `.launch.py` file](https://ranchhandrobotics.com/rde-ros-2/debug-support.html#launch).
* Configure Intellisense
* Support for alternative shells (e.g. `fish`, `zsh`, etc.) for ROS 2.
* (Preview) [Model Context Protocol (MCP) Server](https://ranchhandrobotics.com/rde-ros-2/ModelContextProtocol.html), allowing LLMs to introspect a running ROS 2 system.
* (Preview) Lifecycle Node Support, allowing you to manage the lifecycle of ROS 2 nodes through the Dashboard.
* Support for Visual Studio Code and Cursor

## Getting Started

The VS Code ROS extension will attempt to detect and automatically configure the workspace for the appropriate ROS Distro.

The extension will automatically start when you open a `ROS 2` workspace.

## Migrating from ms-iot/vscode-ros
If you are migrating from the `ms-iot/vscode-ros` extension, consider the following:

* Launch.json - To support splitting the ROS 1 and ROS 2 extensions, the launch task name in launch.json has been changed to `ros2`.

* URDF Editor - The URDF editor has been split into a separate extension. If you were using the URDF editor, you will need to install the [Robot Developer Extension for URDF/Xacro](https://ranchhandrobotics.com/rde-urdf/).

* ROS 1 - If you were using the ROS 1 extension, you will need to install the [Robot Developer Extension for ROS 1](https://ranchhandrobotics.com/rde-ros-1/).


## Documentation, Tutorials, Troubleshooting

Please see the documentation site at [Robotics Developer Extensions for ROS 2 Documentation](https://ranchhandrobotics.com/rde-ros-2/)

## Support
If you encounter any issues with this extension, the following resources are provided:

### Github Issues
Bugs and feature requests are handled through [Github Issues in the Repository](https://github.com/Ranch-Hand-Robotics/rde-ros-2/issues). 
If you find that you are hitting the same issue as someone else, please give a :+1: or comment on an existing issue.
Please provide as much details as possible, including an isolated reproduction of the issue or a pointer to an online repository.

### Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or support.

## Sponsor
If you find this extension useful, please consider [sponsoring Ranch Hand Robotics](https://github.com/sponsors/Ranch-Hand-Robotics) to help support the development of this extension and other open source projects.

## Contribution
Contributions are always welcome! Please see our [contributing guide][CONTRIBUTING.md] for more details!

A big ***Thank you!*** to everyone that have helped make this extension better!

* Andrew Short ([@ajshort](https://github.com/ajshort)), **original author**
* James Giller ([@JamesGiller](https://github.com/JamesGiller))
* PickNikRobotics ([@PickNikRobotics](https://github.com/PickNikRobotics)) for code formatting
* Microsoft ([@ms-iot](https://microsoft.com)) for curating the ms-iot extension