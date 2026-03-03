# AI Features for Robotics Development

The Robot Developer Extensions for ROS 2 integrate powerful AI capabilities to enhance your development experience. This guide covers the AI-powered features available in VS Code and Cursor. The Model Context Protocol (MCP) server allows large language models and AI assistants to introspect your running ROS 2 system and understand its current state. It provides a structured way for your AI assistant to query system information and interact with ROS 2 components. And it even works remotely over SSH or dev tunnels!

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
- Run diagnostics with `ros2 doctor`
- Execute ROS 2 package executables

### Starting the MCP Server

1. Open the command palette (`Ctrl+Shift+P` / `Cmd+Shift+P`)
2. Type **"ROS2: Start MCP Server"** and press Enter
3. The server will start on an available port (starting from 3002)

**First Time Setup**: On the first run, the extension will create a Python virtual environment inside the extension directory. You may be prompted for your super user password to install dependencies.

### Monitoring the Server

The MCP server runs in a dedicated terminal called "ROS 2 MCP Server". You can:
- View the terminal to see server startup logs
- Monitor for any connection issues
- Check that your ROS 2 environment was properly sourced

The extension will also display information about the MCP server connection details so you can connect your AI assistant.

### Connecting Your AI Assistant

**In Cursor**: The MCP server is automatically discovered and registered. Your AI assistant can directly query ROS 2 information without additional configuration.

**In VS Code**: If your AI extension supports MCP, you can manually register the server. When the MCP server starts (without native VS Code MCP support), it will show you:
- The connection URL (typically `http://localhost:PORT/sse`)
- Instructions to add it to your `.cursor/mcp.json` or extension config

Example configuration:

```json
{
  "mcpServers": {
    "ros2": {
      "url": "http://localhost:3002/sse"
    }
  }
}
```

Replace `3002` with the actual port displayed by the extension.

### Example Use Cases

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

## Setting Up Your Environment

To get the most out of AI features:

1. **Configure ROS 2 Environment**
   - Ensure `ROS2.distro` or `ROS2.rosSetupScript` is configured
   - The extension will auto-detect installed ROS 2 distributions

2. **Start the MCP Server** (optional but recommended)
   - Command: `ROS2: Start MCP Server`
   - Enables rich AI queries about your ROS 2 system

3. **Enable AI Features in Your Editor**
   - Use AI completions with `Ctrl+Space`
   - Ask your AI assistant questions about your ROS 2 system
   - Leverage the MCP server for system introspection

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

## Advanced Configuration

### Custom MCP Server Port

The extension automatically discovers an available port starting from 3002. If you need to specify a different starting port, check the extension settings for `ROS2.mcpServerPort` (future enhancement).

### Workspace Integration

The MCP server will use the same ROS 2 environment as the extension, including:
- Your configured distro or setup script
- Environment variables and sourced files
- Installed packages and overlays

## Learn More

- [Configuration Guide](../configuration.md) - Detailed settings reference
- [Debug Support](../debug-support.md) - ROS 2 debugging features
- [Test Explorer](../test-explorer.md) - Test discovery and execution
- [Model Context Protocol Documentation](../ModelContextProtocol.md) - Detailed MCP implementation and available tools
