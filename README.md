# Robotics Development Extensions for ROS 2 within Visual Studio Code
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
* Debug a single ROS node (C++ or Python) by [attaching to the process][debug_support-attach].
* Debug ROS nodes (C++ or Python) [launched from a `.launch` file][debug_support-launch].
* Configure Intellisense

## Getting Started

The VS Code ROS extension will attempt to detect and automatically configure the workspace for the appropriate ROS Distro.

The extension will automatically start when you open a `ROS 2` workspace.

## Documentation, Tutorials, Troubleshooting

Please see the documentation site at [Robotics Developer Extensions for ROS 2 Documentation](https://ranchhandrobotics.github.io/rde-ros-2/)


## Contribution
Contributions are always welcome! Please see our [contributing guide][contributing] for more details!

A big ***Thank you!*** to everyone that have helped make this extension better!

* Andrew Short ([@ajshort](https://github.com/ajshort)), **original author**
* James Giller ([@JamesGiller](https://github.com/JamesGiller))
* PickNikRobotics ([@PickNikRobotics](https://github.com/PickNikRobotics)) for code formatting
