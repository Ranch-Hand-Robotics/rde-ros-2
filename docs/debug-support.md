# Debug ROS Nodes

One of the key goals of `vscode-ros` is to provide a streamlined debugging experience for ROS nodes.
To achieve this, this extension aims to help developers utilize the debugging capabilities provided by Visual Studio Code.
This document covers instructions of how to use such functionalities.

## Attach

`vscode-ros` enables a bootstrapped debugging experience for debugging a ROS (Python or C++) node by attaching to the process.

To get started, create a `ros`-type debug configuration with an `attach` request: (use <kbd>Ctrl</kbd>-<kbd>Space</kbd> to bring up the autocomplete dropdown)

![create attach debug configuration][create_attach_debug_configuration]

### Attaching to a Python node

![attach to a python node][attach_to_python]

### Attaching to a C++ node

![attach to a cpp node][attach_to_cpp]

## Launch

`vscode-ros` enables a streamlined debugging experience for debugging a ROS (Python or C++) node in a ROS launch file similar to a native debug flow.

To get started, create a `ros`-type debug configuration with a `launch` request:

![create launch debug configuration][create_launch_debug_configuration]

### Launch and debug Python and C++ nodes

![launch and debug Python and C++ nodes][launch_and_debug_nodes]


## Note

1. Debugging functionality provided by `vscode-ros` has dependencies on VS Code’s [C++][ms-vscode.cpptools] and [Python][ms-python.python] extensions, and those have dependencies on the version of VS Code. To ensure everything works as expected, please make sure to have everything up-to-date.
2. To debug a C++ executable, please make sure the binary is [built with debug symbols][ros_answers_debug_symbol] (e.g. `-DCMAKE_BUILD_TYPE=RelWithDebInfo`, read more about [CMAKE_BUILD_TYPE here][stackoverflow-cmake_build_type]).
3. To use VS Code's C++ extension with MSVC on Windows, please make sure the VS Code instance is launched from a Visual Studio command prompt.

<!-- link to files -->
[create_attach_debug_configuration]: assets/debug-support/create-attach-debug-config.gif
[attach_to_cpp]: assets/debug-support/attach-to-cpp.gif
[attach_to_python]: assets/debug-support/attach-to-python.gif
[create_launch_debug_configuration]: assets/debug-support/create-launch-debug-config.gif
[check_roscore_status]: assets/debug-support/check-roscore-status.gif
[launch_and_debug_nodes]: assets/debug-support/launch-and-debug-nodes.gif

<!-- external links -->
[ros_answers_debug_symbol]: https://answers.ros.org/question/200155/how-to-debug-executable-built-with-catkin_make-without-roslaunch/

[ms-python.python]: https://marketplace.visualstudio.com/items?itemName=ms-python.python
[ms-vscode.cpptools]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
[ms-vscode.background_bug]: https://github.com/microsoft/vscode/issues/70283
[stackoverflow-cmake_build_type]: https://stackoverflow.com/a/59314670/888545
