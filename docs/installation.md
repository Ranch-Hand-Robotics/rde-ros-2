# ROS 2 Installation Guide

The Robot Developer Extensions for ROS 2 includes an integrated ROS 2 installer that simplifies the process of setting up ROS 2 on your development machine.

## Features

- **Automatic Detection**: The extension automatically detects when you're working in a ROS workspace (by finding `package.xml` files) and prompts you to install ROS 2 if it's not already installed.
- **Multiple Platforms**: Supports installation on Linux, Windows, and macOS using the appropriate package manager for each platform.
- **Distro Selection**: Choose from current ROS 2 distributions with clear marking of LTS (Long-Term Support) releases.
- **Pixi Integration**: On Windows and macOS, the extension can automatically install and configure Pixi for ROS 2 development.
- **AI-Powered Troubleshooting**: If installation fails, get help from GitHub Copilot to diagnose and fix issues.

## Installation Methods

### Linux (APT)

On Linux systems, ROS 2 is installed using the APT package manager. This requires:

- Ubuntu (recommended versions depend on the ROS 2 distro)
- sudo privileges (you'll be prompted for your password)
- Internet connection

The extension will:
1. Set up the ROS 2 repository and GPG keys
2. Install the `ros-<distro>-desktop` package
3. Display all commands in a terminal so you can monitor progress

### Windows (Pixi)

On Windows, ROS 2 is installed using Pixi, a modern cross-platform package manager from prefix.dev.

The extension will:
1. Check if Pixi is installed
2. If not, offer to install Pixi using PowerShell
3. Create a Pixi workspace for ROS 2
4. Install the ROS 2 distribution via the robostack-staging channel

**Prerequisites**:
- Windows 10 or later
- Visual Studio 2019 or later (for building C++ packages)
- PowerShell execution policy that allows script execution

### macOS (Pixi)

On macOS, ROS 2 is installed using Pixi.

The extension will:
1. Check if Pixi is installed
2. If not, offer to install Pixi using curl
3. Create a Pixi workspace for ROS 2
4. Install the ROS 2 distribution via the robostack-staging channel

## How to Use

### Automatic Prompt

When you open a ROS workspace (containing `package.xml` files) and ROS 2 is not detected, you'll see a prompt:

> "ROS 2 is not detected on this system, but this appears to be a ROS workspace. Would you like to install ROS 2?"

You have three options:
- **Yes**: Starts the installation process
- **No**: Dismisses the prompt (you'll be asked again next time)
- **Never for this workspace**: Remembers your choice and won't prompt again for this workspace

### Manual Installation

You can also manually trigger the installation:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Search for "ROS2: Install ROS 2"
3. Select it and follow the prompts

## Selecting a ROS 2 Distribution

When installing, you'll be prompted to select a ROS 2 distribution:

| Distribution | Release Date | Type | Supported Platforms |
|--------------|--------------|------|---------------------|
| Kilted Kaiju | May 2024 | Latest | Ubuntu 24.04, Windows, macOS |
| Jazzy Jalisco | May 2024 | **LTS** | Ubuntu 24.04, Windows, macOS |
| Iron Irwini | November 2023 | Standard | Ubuntu 22.04, Windows, macOS |
| Humble Hawksbill | May 2022 | **LTS** | Ubuntu 22.04, Windows, macOS |

**LTS (Long-Term Support)** releases receive 5 years of support and are recommended for production use.

## Configuration

### Pixi Root Directory

By default, Pixi workspaces are created in `c:\pixi_ws` on Windows. You can change this:

1. Open Settings (`Ctrl+,` or `Cmd+,`)
2. Search for "ROS2 Pixi Root"
3. Change the `ROS2.pixiRoot` setting to your preferred location

### Never Install Prompt

If you selected "Never for this workspace", you can re-enable the prompt:

1. Open workspace settings (`.vscode/settings.json`)
2. Remove or set `"ROS2.neverInstallRos": false`

## Troubleshooting with Copilot

If the installation encounters errors, the extension can help you diagnose the problem using GitHub Copilot:

1. When an error occurs, you'll see a prompt: "ROS 2 installation may have encountered errors. Would you like help diagnosing the issue?"
2. Click "Get Copilot Help"
3. Copilot Chat will open with:
   - Your system information (OS, platform, architecture)
   - The ROS 2 distribution you're installing
   - Environment variables
   - Context about common installation issues

The AI will analyze your specific situation and provide:
- Root cause analysis
- Step-by-step solutions
- Verification steps
- Preventive measures

## Common Issues

### Linux

**Problem**: "GPG key error"
- **Solution**: The extension handles this automatically, but if you see this error, ensure you have internet connectivity and can reach ROS package servers.

**Problem**: "Permission denied"
- **Solution**: The installation requires sudo privileges. Make sure you enter your password when prompted.

**Problem**: "Unsupported Ubuntu version"
- **Solution**: Check the ROS 2 distro requirements. You may need to select a different distro or upgrade your Ubuntu version.

### Windows

**Problem**: "Pixi installation failed - Execution Policy"
- **Solution**: Run PowerShell as Administrator and execute: `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

**Problem**: "Visual Studio not found"
- **Solution**: Install Visual Studio 2019 or later with C++ development tools.

### macOS

**Problem**: "Pixi installation fails"
- **Solution**: Ensure you have `curl` installed and can access pixi.sh. Check your firewall settings.

## Manual Installation Alternative

If automated installation doesn't work for your setup, you can install ROS 2 manually:

- **Linux**: Follow the [official ROS 2 documentation](https://docs.ros.org/en/rolling/Installation.html)
- **Windows/macOS**: Follow the [Pixi setup guide](https://pixi.sh/latest/) and [RoboStack instructions](https://robostack.github.io/)

After manual installation:
1. Reload the VS Code window (`Ctrl+Shift+P` → "Developer: Reload Window")
2. The extension should automatically detect your ROS 2 installation

## Post-Installation

After successful installation:

1. **Reload the window**: The extension will prompt you to reload VS Code
2. **Verify installation**: Open a terminal in VS Code and check:
   - Linux: `ros2 --version`
   - Windows/macOS with Pixi: Navigate to your Pixi workspace (by default `<ROS2.pixiRoot>/ros2-workspace`, which is `c:\pixi_ws\ros2-workspace` on Windows or `~/pixi_ws/ros2-workspace` on macOS/Linux unless you've changed the `ROS2.pixiRoot` setting) and activate the environment
3. **Configure the extension**: The extension should automatically detect your ROS 2 installation and configure the environment

## Related Settings

| Setting | Description | Default |
|---------|-------------|---------|
| `ROS2.distro` | ROS distribution to source | (empty) |
| `ROS2.rosSetupScript` | Path to ROS setup script | (auto-detected) |
| `ROS2.pixiRoot` | Pixi workspace root directory | `c:\pixi_ws` |
| `ROS2.neverInstallRos` | Never prompt to install ROS for this workspace | `false` |

## Getting Help

If you encounter issues:

1. Check the **ROS 2 output channel** in VS Code (View → Output → ROS 2)
2. Use the **ROS2: Doctor** command to diagnose ROS installation issues
3. Use **Copilot Help** during installation failures (if available)
4. Consult the [troubleshooting guide](./troubleshooting.md)
5. File an issue on [GitHub](https://github.com/ranchhandrobotics/rde-ros-2/issues)

## Next Steps

After installing ROS 2:

- Learn about [ROS 2 debugging features](./debug-support.md)
- Set up [IntelliSense for ROS 2](./intellisense.md)
- Explore [launch file debugging](./launchdebugging.md)
- Try the [tutorials](./tutorials.md)
