### ROS Launch Configuration options
The ROS Launch configuration block supports the following configuration:

| Option | Description |
|---|:---|
| name | The name which will be displayed in the VS Code UI launch configuration |
| request | `launch` or `attach` for launching a ROS launch file, or attaching using the attach UI for Pyton or C++ |
| target | the launch file path |
| type | must be `ros` to indicate to VS Code that this is a ROS launch configuration |
| arguments | Arguments passed to roslaunch such as `map:=/foo.yaml'`|
| symbolSearchPath | A semicolon delimited search path for Windows symbols, including ROS for Windows symbols downloaded from https://ros-win.visualstudio.com/ros-win/_build |
| additionalSOLibSearchPath | A semicolon delimited search path for Linux symbols |
| sourceFileMap | A mapping of Source files from where Symbols expect and the location you have on disk. |
| launch | If specified, a list of executables to just launch, attaching to everything else. e.g. `"launch": ["rviz", "gz"]` which prevents attaching a debugger to rviz and gazebo. NOTE: the debugger will ignore file extension: x.py is the same as x.exe. |
| attachDebugger | If specified, a list of executables to debug. `"attachDebugger": ["my_ros_node"]` will only attach to my_ros_node.exe, my_ros_node.py or my_ros_node. |

### Workspace and Global Settings
The ROS extension supports the following global settings, which can be overridden in the workspace.

| Json Option | Setting Name | Description |
|---|:---|---|
| ros.distro | ROS installation distro to be sourced | The Distribution to be sourced. On linux, this cause the extension to look for the ROS setup script in `/opt/ros/{distro}/setup.bash`. On Windows, `c:\opt\ros\{distro}\setup.bat` | 
| ros.rosSetupScript | ROS workspace setup script. Overrides ros.distro. | If specified, this will cause the extension to source this script before generating the launch debugging or ROS terminal environment. This overrides the ros.distro, and can be used to specify user scripts or ROS installs in a different location. |
| ros.isolateEnvironment | Specify if the extension should not capture the environment VS Code is running in to pass to child processes. | Off by default, This setting will prevent the ROS extension from capturing it's hosting environment in case this would conflict with the ROS environment. |

Workspace example:

``` bash
└── .vscode
    ├── launch.json
    ├── settings.json
    └── tasks.json
```

`settings.json`
```json
{
    "ros.distro": "foxy",
    "ros.rosSetupScript": "/opt/ros/foxy/install/setup.bash",
    "ros.isolateEnvironment": "false"
}
```

