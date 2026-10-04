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
    - Each Windows/macOS installation asks you to choose a Pixi root folder; the distro is created in a named child folder. Cancel to stop before installation.
    - The extension caches its Pixi install root by VS Code machine ID in `ROS2.pixiInstallLocationsByMachine`; Settings Sync can safely sync this map between computers.
    - Set the machine-scoped `ROS2.pixiRoot` in User settings to an absolute directory to override the cache on this computer. An empty setting uses `C:\pixi_ws` on Windows or `~/pixi_ws` on macOS.
    - The extension automatically uses the setup script for this computer's installation.
7. **Open a ROS 2 Workspace**: Open a folder containing a ROS 2 workspace. The Robot Developer Extensions will automatically detect the ROS 2 environment and configure the workspace accordingly.

## Pixi Environment Detection
The distribution view scans known setup-script locations under the configured and
default Pixi roots. On macOS it discovers the per-distribution setup wrappers
published by successful installations. **Find ROS** can also locate these wrappers.
For a manually created environment, set `ROS2.rosSetupScript` to a script that
activates it; a `pixi.toml` alone does not automatically select an environment.

## Managing Distributions

In the **Distributions** view, select **+** beside Refresh to install another
ROS 2 distribution. This action remains available when distributions are already
installed.

Hover over a distribution and select its **trash** icon to remove it. For a
verified Pixi installation under the configured or default Pixi root, confirmation
shows the exact directory that will be moved to the Trash, including its manifest,
environments, and any other files in that directory. Stop ROS processes and close
terminals using the installation first. Other distributions and Pixi itself are
not removed.

Matching ROS setup and distribution settings in the current window's user,
workspace, and folder scopes are cleared. If the removed distribution was active,
select **Reload Window** to discard its loaded environment. Other workspaces using
that installation must select another distribution separately.

System-managed installations must be removed with their package manager or original
uninstall procedure, then refreshed in the view. Automatic removal refuses unsafe
paths, symlinked installation directories, and directories containing an open
workspace. If moving to Trash fails, the extension does not fall back to permanent
deletion.

## Platform-Specific Behavior
- **Windows**: Uses `local_setup.bat` from the Pixi ROS 2 environment
- **Linux**: Uses `local_setup.bash` from the Pixi ROS 2 environment
- **macOS**: The installer generates `<pixiRoot>/<distro>/setup.bash`, which activates the complete Pixi environment, including Python and native libraries.

## One-Touch Installation on macOS

Run **ROS 2: Install ROS 2** from the Command Palette and select a distribution.
The installer checks for a working Pixi executable, offers to install it using
the official installer when missing, and uses it immediately without a VS Code
restart. It installs RoboStack packages for the selected distribution and host
architecture only. Available packages and minimum macOS versions depend on the
RoboStack channel; solver errors are captured in the installation log.

Installations run in dedicated task terminals that remain open after completion
or failure. A failed install leaves its error output available for inspection;
starting another installation opens a new terminal without clearing the previous
one. Close these terminals manually when you no longer need them.

Temporary installer scripts are removed when the task succeeds, fails, is
cancelled, or cannot start. Captured logs and installed setup scripts are retained.
If the editor crashes before handling task completion, a temporary script may
remain in the operating system's temporary directory.

Use native Apple Silicon VS Code/Cursor on Apple Silicon, not Rosetta. Intel
Macs use `osx-64`; Apple Silicon uses `osx-arm64`. Apple Command Line Tools and
a working macOS SDK are required for the included development tools. If missing,
the extension offers to open Apple's installer. Finish that installer and rerun
the ROS command. An existing Xcode installation may require accepting its license
or correcting its selected developer directory.

The extension sources the generated setup, checks `ROS_DISTRO`, runs `ros2 --help`,
and creates/destroys an `rclpy` node before publishing the setup script. It then
saves `ROS2.rosSetupScript` and `ROS2.distro` in the current workspace (or user
settings when no workspace is open). Existing, different manifests require
confirmation and are backed up before replacement. Failed installs can be retried.

Neither Pixi nor this ROS installation requires sudo, Developer Mode,
`DevToolsSecurity`, disabling SIP, or disabling Gatekeeper. The extension does
not change these settings. Apple may ask for authorization during its own tools
installation. Debugger attachment permissions are separate from installation.
On recent macOS versions, ROS communication may need Local Network permission
for VS Code/Cursor or the terminal in System Settings. The smoke test does not
verify communication with another machine, RViz rendering, or debugger attachment.

## Resetting macOS Installation Scenarios

From this repository:

```sh
npm run reset:ros:macos -- --dry-run
npm run reset:ros:macos
```

The script displays its targets and requires typing **RESET** interactively.
It force-removes the entire ROS Pixi root, Pixi itself, all Pixi global tools,
configuration (including stored credentials), and Pixi/Rattler caches. It removes
the installer PATH entries and completion setup from common Bash, Zsh, and Fish
profiles, backing up edited files. Homebrew-installed Pixi is uninstalled through
Homebrew. Unmanaged system binaries or unsafe paths cause the reset to stop.

It removes `ROS2.rosSetupScript`, `ROS2.distro`, `ROS2.pixiRoot`,
`ROS2.pixiInstallLocationsByMachine`, and
`ROS2.neverInstallRos` from the current folder settings and VS Code, VS Code
Insiders, and Cursor user/profile settings, preserving unrelated JSONC content.
For custom roots or other workspace settings, pass explicit paths:

```sh
npm run reset:ros:macos -- --pixi-root "$HOME/custom-ros" --settings /path/to/project.code-workspace
```

`PIXI_HOME`, `PIXI_BIN_DIR`, `PIXI_CACHE_DIR`, `ZDOTDIR`, and XDG configuration/cache
locations are honored when present in the invoking terminal. Deletion is limited
to paths below your home directory, excluding common container directories, the
current project and its ancestors, and symlinked paths. Custom standalone binaries
outside your home require manual removal. Other Pixi project environments and
custom shell initialization are not scanned; clean those separately if needed.

Stop ROS processes, close other editor windows, and pause Settings Sync first.
After reset, fully quit and reopen your editors and terminals to discard inherited
environment variables. Apple developer tools and macOS permission grants are not
removed. The reset has no noninteractive confirmation bypass.

## Installer Tests

```sh
npm run test:installer
RDE_TEST_PIXI_LIVE=1 npm run test:installer
```

The opt-in live test downloads Pixi and installs Jazzy into a temporary directory,
tests setup and ROS node creation, then removes the temporary installation and
cache. It does not modify your shell profile or normal Pixi installation. It
requires macOS, Apple Command Line Tools, network access, and several GB of free
disk space. The default tests use temporary fixtures without installing ROS.

## Troubleshooting
If you encounter issues with Pixi or the Robot Developer Extensions, consider the following:

- Ensure that Pixi is installed correctly and the `pixi` command is available in your terminal.
- Check the [Visual Studio Code output panel](./troubleshooting.md) for any error messages related to the Robot Developer Extensions.
- If you are using RoboStack, ensure that the ROS 2 packages are installed correctly and the environment is set up properly outside of the extension.
- Verify that the `ROS2.pixiRoot` setting points to the correct Pixi installation directory.
- For issues related to Pixi, refer to the [Pixi documentation](https://pixi.sh/latest/docs/) or the [Pixi Discord server](https://discord.gg/kKV8ZxyzY4) for community support.



