# Pixi for ROS 2
[Pixi by prefix.dev](https://pixi.sh/latest/) is a next generation package manager, which includes support for ROS 2 development environments. It provides a cross-platform way to manage ROS 2 workspaces, dependencies, and tools.

Open Robotics has standardized on Pixi for Windows development.

## RoboStack & Open Robotics
Open Robotics has partnered with RoboStack to provide a Pixi-based ROS 2 development environment. This environment is designed to work seamlessly with the Robot Developer Extensions for ROS 2, providing a consistent and easy-to-use development experience across platforms.

Open Robotics provides an official distribution of core ROS Components through Open Robotics build system. This distribution is recommended for production use cases.

RoboStack provides a community-driven distribution of ROS 2 packages, which includes additional packages not available in the Open Robotics distribution. This distribution is recommended for development and testing purposes.

## Getting Started with Pixi, ROS 2, and the Robot Developer Extensions
1. **Install Pixi**: Follow the instructions on the [Pixi website](https://pixi.sh/latest/) to install Pixi on your system.
2. **On Windows only, install C++ tools**: Visual Studio 2022 or its standalone Build Tools must include MSVC x64/x86 and a Windows SDK. The full IDE is not required; see below. Linux and macOS do not use this compiler preflight.
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
default Pixi roots. Environment activation uses the same discovery: for a selected
distro, this computer's cached Pixi installation takes precedence over a standard
installation. An explicit `ROS2.rosSetupScript` takes priority, and `ROS2.distro`
takes priority over an inherited `ROS_DISTRO` value.
On macOS it discovers the per-distribution setup wrappers
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
- **Windows**: Activates the Visual Studio C++ environment, then the selected Pixi environment and its ROS `local_setup.bat`.
- **Linux**: For a manually managed Pixi environment, configure `ROS2.rosSetupScript` to activate that environment and source its ROS setup. This change does not add automatic discovery of arbitrary Linux Pixi manifests.
- **macOS**: The installer generates `<pixiRoot>/<distro>/setup.bash`, which activates the complete Pixi environment, including Python and native libraries.

On Linux and macOS, extension-provided `ROS2` and `colcon` tasks wait for the
resolved ROS environment when executed, not when listed. Each rerun obtains the
current environment. Use `taskOptions.cwd`, `taskOptions.env` (null removes a
variable), and `taskOptions.shell` for overrides; variables are resolved by VS Code.
The default shell is `/bin/sh`; explicit shells must use POSIX quoting. Commands
and arguments are literal words. For pipelines, invoke `sh` with `-c` explicitly.
These task terminals use piped input/output, not a full interactive shell or TTY.

### Windows compiler prerequisites

WinGet package `Microsoft.VisualStudio.2022.BuildTools` supplies the standalone
compiler tools. The base package alone is insufficient: select the C++ workload,
`Microsoft.VisualStudio.Component.VC.Tools.x86.x64`,
`Microsoft.VisualStudio.Component.VC.ATL` (C++ ATL for x86/x64), and
`Microsoft.VisualStudio.Component.Windows11SDK.26100`.

When these prerequisites are missing, **ROS2: Install ROS 2** stops before creating
the ROS environment and offers **Copy Install Command**. Review and run that
command in Administrator PowerShell, accepting the applicable agreements. This
is a large, machine-wide installation; the extension does not elevate itself or
run it automatically. Restart Windows if the installer requests it, then retry.
For an existing incomplete installation, use **Visual Studio Installer > Modify >
Desktop development with C++** to add MSVC v143 and a Windows SDK. Under
**Individual components**, also select **C++ ATL for latest v143 build tools (x86 & x64)**;
repeating
`winget install` does not add workloads to an already-installed package.

The extension discovers C++-capable Visual Studio 2022 installations using
`vswhere`, including standalone Build Tools, and runs `vcvarsall.bat x64` before
Pixi activation, dependency solving, package installation, and runtime checks.
It preserves that environment for colcon tasks and ROS terminals. A complete
inherited developer environment is reused for workspace overlays.

Before each Windows colcon build task (including package builds and Test Explorer
builds), the extension rechecks the compiler and SDK files and activates available
tools if needed. Windows colcon tasks remain available in every open workspace,
even when startup activation failed; listing them does not require ROS or colcon.
After the compiler check, a build freshly sources the selected ROS installation
and external underlays, then checks `ros2 --help` and `colcon --help` before building.
If the compiler tools were removed or the setup is incomplete, the build stops
before launching colcon and offers **Copy Install Command** or **Cancel Build**.
ROS setup failures stop the build with an error in the task terminal and
**Output > ROS 2**, including the script path and underlying error. The output
panel opens on setup failures unless `ROS2.autoShowOutputChannel` is disabled.
Listing tasks does not prompt. This check applies to extension-provided `colcon`
tasks, not commands typed directly into a terminal or custom `shell` tasks.
Colcon build tasks are registered at extension startup on all platforms and stay
available during ROS environment reloads. **Tasks: Run Build Task** offers Debug
and Release builds in any open trusted workspace folder, even without
`package.xml`, ROS, or colcon installed. Discovery does not wait for ROS setup or
installation prompts; execution still requires a working build environment.
For Windows `colcon` build tasks in `tasks.json`, use `buildOptions.cwd` and
`buildOptions.env` for working-directory and environment overrides. VS Code
resolves variables in these properties before the preflight runs.

### Recovering from an incomplete workspace install

Windows builds do **not** source the current workspace's install overlay, even
when it exists. A failed compilation can leave `package.bat` referencing a missing
package `local_setup.bat`; this must not prevent the next build from repairing it.
The task terminal and **Output > ROS 2** explain that the overlay is skipped.

Builds start from the host environment, not the extension's runtime environment.
Current install paths (including a task's `--install-base`) are removed from
inherited path lists and package-directory values before compiler/ROS activation.
External underlays and SDK paths are preserved. Recorded external parents in a
standard colcon `setup.bat` are loaded separately; self parents and case/slash
duplicates are excluded. Broken external parents, selected ROS setup, compiler,
SDK, and CLI checks remain fatal. Select the actual ROS/Pixi underlay, not this
workspace's install script, in `ROS2.rosSetupScript`.

Generated Windows package builds and Test Explorer builds use `--packages-up-to`
to build the target and its workspace dependencies. Custom `--packages-select`
arguments are unchanged: colcon loads installed dependencies per package. If those
hooks are incomplete, use `--packages-up-to <package>` or rebuild the dependencies;
explicit skip/ignore filters still apply. No dependencies are silently ignored.
Runtime/debug activation still sources the workspace overlay and reports errors;
Windows C++ tests prepare fresh underlays and load the local overlay on every
run/debug request, including when the executable already exists. Existing binaries
are not rebuilt merely to refresh their environment.

Building with the workspace already in `COLCON_PREFIX_PATH` can record it as its
own parent. Windows case and trailing-separator differences can evade colcon's
string comparisons, producing repeated warnings. The extension excludes these
paths and deduplicates external prefix lists before launching colcon. Only colcon
regenerates setup files; the extension never patches them or deletes build outputs.

Limits: nonstandard batch parent chains are rejected rather than guessed or
silently discarded. Use a terminal with explicit underlays for such custom chains.
Path cleanup handles case and separator variants, not arbitrary junction/short-name
aliases or non-path variables set by a shell before VS Code started. If VS Code was
launched from a self-overlay-sourced shell, restart it from a clean shell when such
custom hooks are involved. Existing CMake cache entries are not rewritten. Recovery
preflight does not fix C++ compilation errors; colcon must report those normally.

Do not manually set `VisualStudioVersion` to suppress an error: the compiler,
linker, SDK tools, headers, and libraries must actually be available. After
installing the tools, restart the extension debugger or reload the window and
create a new ROS terminal; already-open terminals retain their old environment.

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



