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
