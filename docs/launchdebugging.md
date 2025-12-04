## Launch Debugging

## Launch Debugging

The Visual Studio Code extension for ROS supports launch debugging for ROS 2 nodes, written in Python and C++. The ROS node or nodes to be debugged must be placed in a ROS launch file with the extension `.xml` or `.py`. 

### Automatic creation of a launch.json with ROS Launch support
`.vscode/launch.json` is a file which defines a debug launch configuration within VS Code. 

To create a `.vscode/launch.json` with ROS debugging support 

  1. C++ or Python file is selected, vscode uses the selected file to seed the launch creation UI. 
  1. Click the `Run and Debug` tab on the left sidebar
  1. Select the link to create a `.vscode/launch.json` file. 
  1. VS Code will drop down from the command pallet with a list of options, which includes 'ROS'. Select this option. 
  1. In the next dialog, type the name of the ROS package containing a launch file you'd like to debug. 
  1. Then find the launch file.

Once this is created, you can use the play button in the title bar, or the "start debugging" accelerator key, or from the command palle (CTRL-SHIFT-P), select `Debug: Start Debugging`.

> NOTE: Other VS Code extensions may interfere with the selection list. If you do not see ROS in the first drop down list, you'll need to create a new file called `.vscode/launch.json`, then use the manual option described below.

Other Notes:
  * Create a new ROS launch file with just the nodes you'd like to debug, and a separate ROS launch file with all other ROS nodes.
  * Debugging a launch file with Gazebo or rviz is not supported as this time. Please split these out into separate launch files.
  * `ros2 run` is not supported.
  * Traditional XML launch files are supported for ROS1, and both python and XML based launch files are supported for ROS2.

### Manually adding a launch file to an existing launch.json
If you have an existing `launch.json` file (or if there is an extension conflict as mentioned above), you can manually add a launch configuration by adding a new block like this. 
```json
{
    "version": "0.2.0",
    "configurations": [
      {
          "name": "ROS2: Launch my file",
          "request": "launch",
          "target": "<full path to your launch.py or launch file>",
          "launch": ["rviz", "gz"],
          "type": "ros2"
      }
    ]
}  
```
Be sure to include the full path to your launch file, including file extension.

### Environment Variables

You can pass environment variables to your ROS 2 launch configuration in two ways:

#### Using the `env` property

You can define environment variables directly in your launch.json using the `env` property:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2: Launch with env",
            "request": "launch",
            "target": "${workspaceFolder}/launch/my_launch.py",
            "type": "ros2",
            "env": {
                "ROS_DOMAIN_ID": "42",
                "ROS_LOCALHOST_ONLY": "1",
                "MY_CUSTOM_VAR": "value"
            }
        }
    ]
}
```

#### Using the `envFile` property

You can also load environment variables from a file using the `envFile` property. This is useful when you have many environment variables or want to share them across multiple configurations.

Create a `.env` file in your workspace (e.g., `.env` or `.env.ros2`):

```bash
# .env file
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=1
MY_CUSTOM_VAR=value
PATH_TO_CONFIG=/path/to/config

# Comments and empty lines are ignored
# Quotes are optional but can be used for values with spaces
MESSAGE="Hello ROS 2"
```

Then reference it in your launch.json:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2: Launch with envFile",
            "request": "launch",
            "target": "${workspaceFolder}/launch/my_launch.py",
            "type": "ros2",
            "envFile": "${workspaceFolder}/.env"
        }
    ]
}
```

#### Combining `env` and `envFile`

You can use both `env` and `envFile` together. Variables defined in `env` take precedence over those in `envFile`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2: Launch with env and envFile",
            "request": "launch",
            "target": "${workspaceFolder}/launch/my_launch.py",
            "type": "ros2",
            "envFile": "${workspaceFolder}/.env",
            "env": {
                "ROS_DOMAIN_ID": "99"  // This overrides the value from .env
            }
        }
    ]
}
```

This allows you to have a common set of environment variables in a file and override specific ones as needed.
