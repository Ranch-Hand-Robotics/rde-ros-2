# AI Features for Robotics Development

The Robot Developer Extensions for ROS 2 integrate powerful AI capabilities to enhance your development experience. This guide covers the AI-powered features available in VS Code and Cursor. The Model Context Protocol (MCP) server allows large language models (LLMs) and AI assistants like *Github Copilot* to introspect your running ROS 2 system - to understand ***and modify*** its current state. It provides a structured way for your AI assistant to query system information and interact with ROS 2 components. And it even works remotely over SSH or dev tunnels!

Ask Copilot:
**Debugging**
> "Why isn't my node connecting to this topic? What nodes are currently running?"

**System Exploration**
> "List all services available in my ROS 2 system and their types"

**Development**
> "Show me the definition of this message type and help me create a publisher for it"

**Monitoring**
> "What's the current state of my lifecycle nodes? Can I safely transition them?"

**Testing**
> "Execute this launch file and show me what nodes are running"

**Analysis**
> "Record the /cmd_vel topic to a bag file for later analysis"

### What MCP Can Do

With the MCP server running, your AI assistant can:

**Node & System Management**
- List and inspect running nodes and their details
- Manage node lifecycle states
- Monitor system health and diagnostics

**Communication Channels**
- Query available topics and their message types
- Inspect services and their request/response types
- Discover and interact with actions
- Publish messages to topics and call services

**Parameter Management**
- List all parameters for specific nodes
- Get and set parameter values
- Understand parameter configurations

**Package & File Operations**
- List available ROS 2 packages and executables
- Explore launch files and their parameters
- Access package manifests and metadata

**Data Recording & Playback**
- Record ROS 2 topics to bag files
- Play back bag files for analysis
- Retrieve bag file information

**Development Support**
- Inspect ROS 2 message, service, and action definitions
- Get package manifest information
- Run diagnostics with `ros2 doctor`, and have ***copilot fix your code***!
- Execute ROS 2 package executables

### Starting the MCP Server

1. Open the command palette (`Ctrl+Shift+P` / `Cmd+Shift+P`)
2. Find or type **"ROS2: Start MCP Server"** and press Enter
3. The server will start and register itself with Copilot

**First Time Setup**: On the first run, the extension will create a Python virtual environment inside the extension directory. You may be prompted for your super user password to install dependencies.

## ROS 2 Agents and Skills

In VS Code 1.110 or newer with Copilot Chat, select **ROS 2 Expert** from the
Chat agent picker. It can delegate to specialists for Core, Networking (DDS/RMW),
Packaging, Testing, MoveIt 2, Navigation (Nav2), Hardware, Drones (MAVROS/PX4/ArduPilot),
Simulation, and operating systems (Windows, macOS, Ubuntu, and NVIDIA Jetson). All ten
specialists are hidden from the agent picker and invoked through the top-level
**ROS 2 Expert** orchestration agent: eleven agents in total.

Type `/ros2-` in Chat to discover sixteen workflows, including shared development
guidance (`/ros2-development`) and installation diagnostics
(`/ros2-install-troubleshooting`), meaningful testing (`/ros2-test`), plus build, packaging,
actions/services/lifecycle, networking, debugging, perception, manipulation,
navigation, launch, performance, Gazebo, Omniverse Isaac Sim, and MuJoCo.
Relevant skills can also load automatically. No workspace copying
or MCP server is required for static development help; runtime inspection needs
available ROS tooling. These VS Code contributions may not be supported by Cursor.

**ROS 2 Core** owns general launch orchestration with `/ros2-launch` and uses
`/ros2-performance` for measurement-driven optimization. **ROS 2 Simulation**
handles simulator-specific integration with those shared skills and `/ros2-test`:

- `/ros2-gazebo` distinguishes modern Gazebo (`gz`, `ros_gz`) from legacy Gazebo
  Classic (`gazebo`, `gazebo_ros`); plugins and APIs are not interchangeable.
- `/ros2-omniverse` targets NVIDIA Omniverse Isaac Sim and its compatible ROS 2 bridge.
- `/ros2-mujoco` requires an explicit ROS adapter; MuJoCo alone does not expose
  ROS topics, services, TF, or a simulation clock.

Testing focuses on intended behavior and realistic defects, not mirroring code.
Discovered bugs are captured as permanent regression tests; blocked coverage is
reported explicitly.

Try: "Inspect this workspace and help me implement a cancellable action server
with lifecycle-managed resources. Validate using mocks, not the physical robot."

The agents default to simulation/static checks and require explicit approval for
live hardware state changes. Review proposed commands and keep tool confirmations
enabled; instructions are not a physical safety interlock.

## AI Completions

Smart code completions leverage AI to provide contextual suggestions for:

- **ROS 2 API calls** - Common rclpy and rclcpp patterns
- **Launch file syntax** - Python-based and XML launch file configurations
- **Message definitions** - Auto-complete for custom message types
- **Build configurations** - CMakeLists.txt and package.xml patterns

Completions work seamlessly in:
- Python files (`.py`)
- C++ files (`.cpp`, `.h`)
- Launch files (`.launch.py`, `.launch`)
- Configuration files (`.yaml`, `.json`)

### Triggering Completions

- Press `Ctrl+Space` (or `Cmd+Space` on Mac) to manually trigger completions
- Completions appear automatically as you type
- Completions adapt based on your ROS 2 environment context

## Tips for Better AI Assistance

- **MCP Server Context**: Start the MCP server before asking ROS 2-specific questions for the most accurate information
- **Open Files**: Open relevant source files before asking AI questions for better context
- **Specific Queries**: Provide exact error messages and code snippets for more targeted help
- **Workspace Structure**: Keep your workspace properly structured with package.xml files for better discovery
- **ROS 2 Configuration**: Ensure your ROS 2 environment is properly configured in settings

## Troubleshooting

### MCP Server Won't Start

- Check that you have a valid ROS 2 environment configured
- View the "ROS 2 Output" channel for detailed logs and error messages
- Check that no other process is using the port (starts at 3002)
- Try stopping the server and restarting it

**First Run**: The first time you start the server, it will create a Python virtual environment. This may require elevated privileges to install dependencies.

### Missing Completions

- Verify the file has the correct language type (`.py`, `.cpp`, etc.)
- Check that you're in a ROS 2 workspace with proper package structure
- Ensure `ROS2.distro` or `ROS2.rosSetupScript` is configured
- Try triggering completions manually with `Ctrl+Space`

### AI Assistant Can't Find Information

- Start the MCP server before asking ROS 2-specific questions
- Open your workspace folder (not just individual files)
- Include workspace path context in your questions
- Check that your ROS 2 environment is properly sourced

### Virtual Environment Issues

- The MCP server maintains its own Python virtual environment in `.venv` directory
- Do not manually modify this directory
- If you encounter persistent issues, you can delete `.venv` and restart the server
