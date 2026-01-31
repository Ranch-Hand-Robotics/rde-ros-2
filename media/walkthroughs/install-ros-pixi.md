# Installing ROS 2 with Pixi

[Pixi by prefix.dev](https://pixi.sh/latest/) is a next-generation package manager that provides a cross-platform way to manage ROS 2 workspaces, dependencies, and tools. This is the recommended approach for Windows and macOS.

## Why Pixi?

* **Cross-platform** - Works on Windows, macOS, and Linux
* **Easy setup** - No complex environment configuration
* **Reproducible** - Consistent environments across teams
* **Open Robotics endorsed** - Standardized for Windows development

## Installation Steps

### Step 1: Install Pixi

Visit [pixi.sh](https://pixi.sh/latest/) and follow the installation instructions for your platform:

**Windows (PowerShell):**
```powershell
iwr -useb https://pixi.sh/install.ps1 | iex
```

**macOS/Linux:**
```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### Step 2: Install Visual Studio (Windows only)

On Windows, you need Visual Studio for building ROS 2 packages:

1. Download and install [Visual Studio](https://visualstudio.microsoft.com/)
2. During installation, select "Desktop development with C++"

### Step 3: Install ROS 2 via Pixi

You can install ROS 2 using either:

#### Option A: RoboStack (Development & Testing)

RoboStack provides a community-driven distribution with additional packages:

```bash
pixi init my-ros-workspace
cd my-ros-workspace
pixi add ros-humble-desktop
```

Visit [robostack.github.io](https://robostack.github.io/) for more information.

#### Option B: Open Robotics (Production)

Open Robotics provides the official core ROS components:

Follow the instructions at [docs.ros.org](https://docs.ros.org/en/kilted/Installation/Windows-Install-Binary.html).

### Step 4: Configure the Extension

1. Open VS Code settings (File → Preferences → Settings)
2. Search for "ROS2: Pixi Root"
3. Set the `ROS2.pixiRoot` setting to your Pixi workspace directory (default: `c:\pixi_ws` on Windows)

The extension will automatically detect and use your Pixi environment!

## Additional Resources

* [Pixi Documentation](https://pixi.sh/latest/docs/)
* [RoboStack Website](https://robostack.github.io/)
* [Extension Pixi Documentation](https://ranchhandrobotics.github.io/rde-ros-2/pixi/)

Once Pixi and ROS 2 are set up, the extension will automatically configure your environment when you open a ROS 2 workspace!
