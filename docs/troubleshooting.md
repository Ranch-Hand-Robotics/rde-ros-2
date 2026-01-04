# Troubleshooting the Robot Developer Extensions
ROS 2 is a complex system, and issues can arise from various sources. AI and the VSCode ecosystem are fast moving targets. This document provides troubleshooting tips for self-diagnosing and common issues encountered when using the Robot Developer Extensions for ROS 2.

## Output Panel
The VSCode output panel provides detailed logs from the Robot Developer Extensions. You can access it by going to `Output: Focus on Output View` or using the shortcut `Ctrl+Shift+U`. In the dropdwon next to the Filter edit box, select `ROS 2`.

Look for any error messages or warnings that might indicate what is going wrong.
IF it is not clear, copy the output and paste it into a [GitHub issue](https://github.com/Ranch-Hand-Robotics/rde-ros-2/issues) or [Github Discussion](https://github.com/orgs/Ranch-Hand-Robotics/discussions) for further assistance. Make sure you remove any sensitive information before sharing.

## ROS 2 Installation Issues

If you're having trouble installing ROS 2 using the extension's built-in installer, see the [Installation Guide](./installation.md) for detailed troubleshooting steps.

### Quick Install Troubleshooting

1. **Check the Output Panel**: The ROS 2 output channel shows detailed installation logs
2. **Use Copilot Help**: When installation fails, click "Get Copilot Help" for AI-powered diagnosis
3. **Verify Prerequisites**:
   - Linux: sudo privileges, internet connection
   - Windows: PowerShell execution policy, Visual Studio installed
   - macOS: curl available, firewall not blocking downloads
4. **Manual Installation**: If automated installation fails, follow the [manual installation steps](./installation.md#manual-installation-alternative)

### ROS 2 Not Detected After Installation

If ROS 2 was installed but the extension doesn't detect it:

1. **Reload the window**: `Ctrl+Shift+P` → "Developer: Reload Window"
2. **Check ROS environment variables**:
   - Linux: Open a terminal and run `source /opt/ros/<distro>/setup.bash && printenv | grep ROS`
   - Windows/macOS with Pixi: Activate your Pixi environment and check ROS variables
3. **Manually configure setup script**: Set `ROS2.rosSetupScript` in settings to point to your setup file
4. **Check the distro setting**: Ensure `ROS2.distro` matches your installed distribution

## Github Issues
Bugs and feature requests are handled through [Github Issues in the Repository](https://github.com/Ranch-Hand-Robotics/rde-ros-2/issues). 
If you find that you are hitting the same issue as someone else, please give a :+1: or comment on an existing issue.
Please provide as much details as possible, including an isolated reproduction of the issue or a pointer to an online repository. Please remove any sensitive information before sharing.

## Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or asynchronous support.


