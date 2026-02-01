# Feature: Debug Session Context MCP Tools

## Related Issue
Part of DebugMCP integration investigation

## Summary
Create a TypeScript MCP server to expose debug session information to AI assistants, enabling them to understand the current debugging state and provide contextual assistance.

## Motivation
AI assistants need visibility into active ROS 2 debugging sessions. Requirements:
- Which ROS nodes are currently being debugged
- What debug configurations are active
- Which breakpoints are set and where
- Current debug session state (running, paused, etc.)

This enables AI assistants to:
- Suggest relevant breakpoints based on ROS message flow
- Identify which nodes to debug when investigating issues
- Coordinate debugging across multiple nodes in a composition

## Proposed Solution

**Create `src/ros2-debug-mcp.ts`** following the [RDE-URDF mcp.ts pattern](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts) with three MCP tools:

### 1. `get_active_debug_sessions()`
Returns information about all currently active debug sessions managed by RDE.

### 2. `get_node_debug_info(node_name: str)`
Returns debugging-specific information for a given node.

### 3. `get_launch_file_debug_info(launch_file: str)`
Analyzes a launch file and returns debugging-relevant information.

## Implementation Details

**Files to Create/Modify:**
- `src/ros2-debug-mcp.ts` (NEW): TypeScript MCP server with debug tools
- `src/extension.ts`: Start MCP server on activation
- `src/debugger/manager.ts`: Add debug state tracking
- `package.json`: Add MCP SDK dependencies

**Pattern to Follow:**
https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts

## Dependencies
- `@modelcontextprotocol/sdk`: ^0.5.0
- `express`: ^4.18.2

## Estimated Effort
1-2 days for a developer familiar with the codebase

## Implementation Priority
High - Foundational feature for AI-assisted debugging

**IMPORTANT**: TypeScript implementation only. Do NOT modify `assets/scripts/server.py` (Python MCP server).
