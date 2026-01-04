## Commands

You can access the following commands from the [Visual Studio Code command pallet](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette), typically accessed by pressing `ctrl` + `shift` + `p` and typing the command name you'd like to use from the table below.

| Name | Description |
|---|:---|
| ROS2: Create Terminal | Create a terminal with the ROS environment. |
| ROS2: Show Status | Open a detail view showing ROS core runtime status. |
| ROS2: Start | Start ROS1 core or ROS2 Daemon. |
| ROS2: Stop  | Terminate ROS core or ROS2 Daemon. |
| ROS2: Update C++ Properties | Update the C++ IntelliSense configuration to include ROS and your ROS components. See [IntelliSense](intellisense.md) for more details. |
| ROS2: Update Python Path | Update the Python IntelliSense configuration to include ROS. See [IntelliSense](intellisense.md) for more details. |
| ROS2: Preview URDF | Preview URDF and Xacro files. The display will update after the root URDF changes are saved. |
| ROS2: Install ROS Dependencies for this workspace using rosdep | Shortcut for `rosdep install --from-paths src --ignore-src -r -y`. |
| ROS2: Toggle Colcon Ignore | Toggle whether a package should be ignored during colcon build. Available in the right-click context menu on folders. |
| ROS2: Colcon Build This Package | Build only the selected package using `colcon build --packages-select`. Available in the right-click context menu on folders. |

## Colcon Build Features

### Ignoring Packages During Build

You can exclude specific packages from colcon builds using the Colcon Ignore feature:

1. Right-click on a folder containing a ROS 2 package in the Explorer view
2. Select "Toggle Colcon Ignore" from the context menu
3. The package will be added to the `ROS2.colconIgnore` configuration in your workspace settings
4. When you run colcon build tasks, ignored packages will be automatically excluded using `--packages-select`

To include a previously ignored package, simply right-click and select "Toggle Colcon Ignore" again.

### Building Individual Packages

To quickly build a single package without building the entire workspace:

1. Right-click on a folder containing a ROS 2 package in the Explorer view
2. Select "Colcon Build This Package" from the context menu
3. A build task will be created and executed for just that package using `--packages-select`

## IntelliSense Features

The extension provides rich IntelliSense support for ROS message files (`.msg`, `.srv`, `.action`). 

For detailed information about hover tooltips, go-to-definition, and other IntelliSense features, see the [IntelliSense documentation](intellisense.md).
