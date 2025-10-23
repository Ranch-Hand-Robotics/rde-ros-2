# Pixi for ROS 2
[Pixi by prefix.dev](https://pixi.sh/latest/) is a next generation package manager, which includes support for ROS 2 development environments. It provides a cross-platform way to manage ROS 2 workspaces, dependencies, and tools.

Open Robotics has standardized on Pixi for Windows development.

## RoboStack & Open Robotics
Open Robotics has partnered with RoboStack to provide a Pixi-based ROS 2 development environment. This environment is designed to work seamlessly with the Robot Developer Extensions for ROS 2, providing a consistent and easy-to-use development experience across platforms.

Open Robotics provides an official distribution of core ROS Components through Open Robotics build system. This distribution is recommended for production use cases.

RoboStack provides a community-driven distribution of ROS 2 packages, which includes additional packages not available in the Open Robotics distribution. This distribution is recommended for development and testing purposes.

## Getting Started with Pixi, ROS 2, and the Robot Developer Extensions
1. **Install Pixi**: Follow the instructions on the [Pixi website](https://pixi.sh/latest/) to install Pixi on your system.
2. **Install Visual Studio**: Download and install [Visual Studio](https://visualstudio.com/). This is needed for building ROS 2 packages on Windows.
3. **Install Visual Studio Code**: Download and install [Visual Studio Code](https://code.visualstudio.com/).
4. **Install ROS 2 through RoboStack or Open Robotics** depending on your use case:
   - For Development and Testing, follow the instructions on the [RoboStack website](https://robostack.github.io/).
   - For Production Environments, follow the instructions on the [ros.org](https://docs.ros.org/en/kilted/Installation/Windows-Install-Binary.html).
5. **Install the Robot Developer Extensions (RDE) for ROS 2**: Install the [Robot Developer Extensions for ROS 2](https://ranchhandrobotics.github.io/rde-ros-2/) from the [Visual Studio Code Marketplace](https://marketplace.visualstudio.com/items?itemName=Ranch-Hand-Robotics.rde-ros-2) or [Open-Vsx.org](https://open-vsx.org/extension/Ranch-Hand-Robotics/rde-ros-2).
6. **Configure Pixi in RDE**
    - Open the Workspace settings.
    - Set the `ROS2.usePixiOnAllPlatforms` setting to `true` if you want to use Pixi on all platforms. This will allow the extension to automatically configure the ROS 2 environment using Pixi.
    - Set the `ROS2.pixiRoot` setting to specify the root directory where Pixi environments are installed. This defaults to `c:\pixi_ws\ros` on Windows and not inferred on Mac or Linux.
    - The extension will automatically detect and use the appropriate setup script based on your platform and Pixi configuration.
7. **Open a ROS 2 Workspace**: Open a folder containing a ROS 2 workspace. The Robot Developer Extensions will automatically detect the ROS 2 environment and configure the workspace accordingly.

## Pixi Environment Detection
The extension automatically detects Pixi environments by:
- Checking for the presence of `pixi.toml` or `pixi.lock` files in the workspace
- Looking for Pixi environment directories in the configured `pixiRoot`
- Detecting active Pixi environments in the current shell session

When Pixi is enabled, the extension will:
- Source the appropriate ROS 2 setup script from the Pixi environment
- Configure the Python environment to use Pixi-managed packages
- Set up the build environment for ROS 2 packages using Pixi

## Platform-Specific Behavior
- **Windows**: Uses `local_setup.bat` from the Pixi ROS 2 environment
- **Linux**: Uses `local_setup.bash` from the Pixi ROS 2 environment
- **macOS**: Uses `local_setup.bash` from the Pixi ROS 2 environment

## Troubleshooting
If you encounter issues with Pixi or the Robot Developer Extensions, consider the following:
- Ensure that Pixi is installed correctly and the `pixi` command is available in your terminal.
- Check the [Visual Studio Code output panel](./troubleshooting.md) for any error messages related to the Robot Developer Extensions.
- If you are using RoboStack, ensure that the ROS 2 packages are installed correctly and the environment is set up properly outside of the extension.
- Verify that the `ROS2.pixiRoot` setting points to the correct Pixi installation directory.
- For issues related to Pixi, refer to the [Pixi documentation](https://pixi.sh/latest/docs/) or the [Pixi Discord server](https://discord.gg/kKV8ZxyzY4) for community support.



