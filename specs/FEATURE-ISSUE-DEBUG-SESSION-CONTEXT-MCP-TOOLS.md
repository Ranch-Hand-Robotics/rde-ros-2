# GitHub Issue Template: Debug Session Context MCP Tools

## Issue Title
[Feature] Add Debug Session Context MCP Tools

## Labels
- `enhancement`
- `mcp`
- `debugging`
- `good first issue` (if appropriate)

## Assignee
@copilot (or specific copilot agent)

## Issue Description

### Summary
Add MCP (Model Context Protocol) tools to the RDE ROS 2 extension that expose debug session state and launch file composition information to AI assistants. This enables AI-powered debugging assistance for multi-node ROS 2 systems.

### Background
As part of the DebugMCP integration investigation (see #[issue_number]), we identified that exposing debug session context via MCP would provide immediate value for AI-assisted debugging without requiring external dependencies.

Currently, the RDE MCP server provides excellent ROS 2 system introspection (34 tools for nodes, topics, services, etc.) but lacks visibility into active debugging sessions. This feature adds three new MCP tools to bridge that gap.

### Motivation
- Enable AI assistants to understand which ROS nodes are currently being debugged
- Provide launch file composition analysis for debugging planning
- Allow AI to suggest relevant breakpoints based on message flow
- Foundation for future automated debugging workflows

### Proposed Implementation

Add three new MCP tools to `assets/scripts/server.py`:

#### 1. `get_active_debug_sessions()`
Returns information about all currently active debug sessions managed by RDE.

**Returns:**
```json
[
  {
    "session_id": "debug-12345",
    "type": "ros2",
    "request": "launch",
    "target": "/path/to/launch/file.py",
    "nodes": [
      {
        "node_name": "talker",
        "executable": "/workspace/install/demo_nodes_cpp/lib/demo_nodes_cpp/talker",
        "process_id": 12345,
        "runtime": "C++",
        "debug_status": "paused"
      }
    ],
    "status": "active",
    "started_at": "2024-01-31T10:30:00Z"
  }
]
```

#### 2. `get_node_debug_info(node_name: str)`
Returns debugging-specific information for a given node.

**Parameters:**
- `node_name` (str): Name of the ROS node

**Returns:**
```json
{
  "node_name": "talker",
  "executable": "/workspace/install/demo_nodes_cpp/lib/demo_nodes_cpp/talker",
  "source_file": "/workspace/src/demos/demo_nodes_cpp/src/talker.cpp",
  "process_id": 12345,
  "runtime": "C++",
  "debug_session_id": "debug-12345",
  "debug_status": "paused"
}
```

#### 3. `get_launch_file_debug_info(launch_file: str)`
Analyzes a launch file and returns debugging-relevant information.

**Parameters:**
- `launch_file` (str): Path to the launch file

**Returns:**
```json
{
  "launch_file": "/workspace/src/demos/demo_launch.py",
  "nodes": [
    {
      "node_name": "talker",
      "package": "demo_nodes_cpp",
      "executable": "talker",
      "runtime": "C++",
      "debuggable": true,
      "source_package_path": "/workspace/src/demos/demo_nodes_cpp"
    }
  ],
  "topics": [
    {
      "name": "/chatter",
      "type": "std_msgs/String",
      "publishers": ["talker"],
      "subscribers": ["listener"]
    }
  ],
  "lifecycle_nodes": []
}
```

### Implementation Guidance

#### Files to Modify
1. **`assets/scripts/server.py`**: Add the three new MCP tool functions
2. **`src/debugger/manager.ts`**: Add debug state export functionality
3. **`assets/scripts/ros2_launch_dumper.py`**: Enhance with additional metadata (source paths, runtime detection)

#### Technical Approach

**Integration with Debug Manager:**
The MCP tools need to access debug session state. Recommended approach:
- Export debug session state to a JSON file in a known location (e.g., `<extension_dir>/.debug-state.json`)
- Have `src/debugger/manager.ts` write session state when sessions start/stop
- MCP tools read from this file

**Launch File Analysis:**
- Leverage existing `ros2_launch_dumper.py` functionality
- Enhance dumper to include:
  - Source package paths for each node
  - Runtime type detection (C++/Python/Rust/.NET)
  - Topic/service relationship mapping

**State Management:**
- Monitor debug session lifecycle events
- Maintain session state file with current sessions
- Clean up state when sessions end

#### Error Handling
- Return empty array `[]` if no active debug sessions
- Return error if node not found or not being debugged
- Return error if launch file cannot be parsed
- Handle cases where debug manager state is unavailable gracefully

### Testing Requirements

#### Unit Tests
- Test each MCP tool in isolation
- Mock debug session state
- Verify correct JSON response format
- Test error conditions

#### Integration Tests
- Start a debug session via RDE
- Call `get_active_debug_sessions()` and verify response
- Verify node debug info for debugged nodes
- Test launch file analysis on Python and XML launch files

#### Manual Testing
1. Start MCP server: `ROS2: Start MCP Server` command
2. Launch a ROS composition with debugger
3. Query debug state via Copilot/Cursor using MCP
4. Verify information accuracy

### Success Criteria
- [ ] All three MCP tools implemented and functional
- [ ] Tools return accurate information about debug sessions
- [ ] Launch file analysis works for Python and XML launch files
- [ ] No performance degradation when tools are called frequently
- [ ] Documentation updated in `docs/ModelContextProtocol.md` with new tool descriptions
- [ ] Unit tests pass for all scenarios
- [ ] Manual testing completed successfully

### Dependencies
- None (can be implemented independently)

### Estimated Effort
**1-2 days** for a developer familiar with the codebase

### Priority
**High** - This is a foundational feature that provides immediate value and enables future DebugMCP integration work.

### Related Documentation
- [DebugMCP Integration Investigation](specs/DebugMCP-Integration-Investigation.md)
- [Feature Proposal](specs/feature-proposals/debug-session-context-mcp-tools.md)
- [Current MCP Server Documentation](docs/ModelContextProtocol.md)

### Notes
- This feature does not require DebugMCP to be installed
- Can be tested immediately without external dependencies
- Provides value even without full DebugMCP integration
- Foundation for Feature #2 (DebugMCP Integration Layer) if implemented later

### Example Use Cases

**Use Case 1: Identifying nodes to debug**
User asks AI: "Which nodes should I debug for the WebRTC pipeline?"
AI uses `get_launch_file_debug_info()` to analyze launch file and suggests debugging the `webrtc_bridge` node based on topic relationships.

**Use Case 2: Debug session overview**
User asks AI: "What am I currently debugging?"
AI uses `get_active_debug_sessions()` to show all active debug sessions and their states.

**Use Case 3: Node-specific debugging info**
User asks AI: "What's the status of the talker node?"
AI uses `get_node_debug_info("talker")` to show process ID, source file, and debug status.

---

## Instructions for Implementation

### Step 1: Set Up Debug State Management
1. Create `assets/scripts/debug_state_manager.py` helper module
2. Implement state file management (read/write debug session state)
3. Define JSON schema for debug state

### Step 2: Modify Debug Manager
1. Edit `src/debugger/manager.ts`
2. Add hooks to export debug state when sessions start
3. Add hooks to clean up state when sessions end
4. Export state to `<extension_dir>/.debug-state.json`

### Step 3: Enhance Launch Dumper
1. Edit `assets/scripts/ros2_launch_dumper.py`
2. Add source package path detection
3. Add runtime type detection logic
4. Add topic/service relationship mapping

### Step 4: Implement MCP Tools
1. Edit `assets/scripts/server.py`
2. Add `@mcp.tool()` decorated function for `get_active_debug_sessions()`
3. Add `@mcp.tool()` decorated function for `get_node_debug_info(node_name: str)`
4. Add `@mcp.tool()` decorated function for `get_launch_file_debug_info(launch_file: str)`
5. Implement error handling for each tool

### Step 5: Update Documentation
1. Edit `docs/ModelContextProtocol.md`
2. Add tool descriptions to the Available Tools table
3. Add example usage for new tools

### Step 6: Testing
1. Create unit tests in appropriate test directory
2. Test each tool independently
3. Test error conditions
4. Perform manual integration testing

### Step 7: Verification
1. Run `npm run build` to ensure no build errors
2. Run `npm run lint` to check code style
3. Run tests to verify functionality
4. Manual test with actual ROS 2 workspace

---

## Acceptance Checklist
- [ ] Code implemented and tested
- [ ] Unit tests added and passing
- [ ] Documentation updated
- [ ] Build passes without errors
- [ ] Manual testing completed
- [ ] No new security vulnerabilities introduced
- [ ] Changes reviewed and approved
