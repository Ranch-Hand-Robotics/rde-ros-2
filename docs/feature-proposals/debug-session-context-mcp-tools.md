# Feature: Debug Session Context MCP Tools

## Related Issue
Part of DebugMCP integration investigation

## Summary
Extend the RDE MCP server to expose debug session information to AI assistants, enabling them to understand the current debugging state and provide contextual assistance.

## Motivation
Currently, the RDE MCP server provides excellent ROS 2 system introspection (nodes, topics, services, etc.), but lacks visibility into active debugging sessions. AI assistants working with DebugMCP need to understand:
- Which ROS nodes are currently being debugged
- What debug configurations are active
- Which breakpoints are set and where
- Current debug session state (running, paused, etc.)

This information enables AI assistants to provide better debugging assistance, such as:
- Suggesting relevant breakpoints based on ROS message flow
- Identifying which nodes to debug when investigating issues
- Coordinating debugging across multiple nodes in a composition

## Proposed Solution

Add the following MCP tools to `assets/scripts/server.py`:

### 1. `get_active_debug_sessions()`
Returns information about all currently active debug sessions managed by RDE.

### 2. `get_node_debug_info(node_name: str)`
Returns debugging-specific information for a given node.

### 3. `get_launch_file_debug_info(launch_file: str)`
Analyzes a launch file and returns debugging-relevant information.

## Implementation Details

**Files to Modify:**
- `assets/scripts/server.py`: Add new MCP tool functions
- `src/debugger/manager.ts`: Add debug state export functionality
- `assets/scripts/ros2_launch_dumper.py`: Enhance with additional metadata

## Dependencies
None (can be implemented independently)

## Estimated Effort
1-2 days for a developer familiar with the codebase

## Implementation Priority
High - This is a foundational feature that other DebugMCP integration features will build upon.
