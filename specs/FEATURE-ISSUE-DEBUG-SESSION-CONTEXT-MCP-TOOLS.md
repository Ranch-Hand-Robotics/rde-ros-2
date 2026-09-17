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

**IMPORTANT**: All MCP tools must be implemented in TypeScript following the [RDE-URDF mcp.ts pattern](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts). Create a new file `src/ros2-debug-mcp.ts` for these tools.

Add three new MCP tools to `src/ros2-debug-mcp.ts`:

#### 1. `get_active_debug_sessions()`
Returns information about all currently active debug sessions managed by RDE.

**TypeScript Implementation:**
```typescript
this.server.registerTool('get_active_debug_sessions', {
  title: 'Get Active Debug Sessions',
  description: 'Returns information about all active ROS debug sessions',
  inputSchema: {}
}, async (args) => {
  // Access vscode.debug.activeDebugSession
  // Return debug session information
  return { content: [{ type: 'text', text: JSON.stringify(sessions) }] };
});
```

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

**TypeScript Implementation:**
```typescript
this.server.registerTool('get_node_debug_info', {
  title: 'Get Node Debug Info',
  description: 'Returns debugging information for a specific ROS node',
  inputSchema: {
    type: 'object',
    properties: {
      node_name: { type: 'string', description: 'Name of the ROS node' }
    },
    required: ['node_name']
  }
}, async (args) => {
  const nodeName = (args as any).node_name;
  // Query debug manager for node info
  return { content: [{ type: 'text', text: JSON.stringify(nodeInfo) }] };
});
```

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

**TypeScript Implementation:**
```typescript
this.server.registerTool('get_launch_file_debug_info', {
  title: 'Get Launch File Debug Info',
  description: 'Analyzes a launch file for debugging information',
  inputSchema: {
    type: 'object',
    properties: {
      launch_file: { type: 'string', description: 'Path to launch file' }
    },
    required: ['launch_file']
  }
}, async (args) => {
  const launchFile = (args as any).launch_file;
  // Use launch dumper to analyze
  return { content: [{ type: 'text', text: JSON.stringify(launchInfo) }] };
});
```

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

#### Files to Create/Modify
1. **`src/ros2-debug-mcp.ts`** (NEW): TypeScript MCP server following [RDE-URDF pattern](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts)
2. **`src/extension.ts`**: Import and start the MCP server
3. **`src/debugger/manager.ts`**: Add debug state tracking
4. **`assets/scripts/ros2_launch_dumper.py`**: Enhance with additional metadata (if needed)

#### Technical Approach

**TypeScript MCP Server Implementation:**
Following the [RDE-URDF mcp.ts pattern](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts):
- Create `src/ros2-debug-mcp.ts` with `Ros2DebugMcpServer` class
- Use `@modelcontextprotocol/sdk` for MCP server infrastructure
- Register tools using `this.server.registerTool()` method
- Access VS Code debugging APIs directly via `vscode.debug.*`
- Integrate into extension lifecycle in `src/extension.ts`

**Debug State Access:**
- Query `vscode.debug.activeDebugSession` for current sessions
- Use `vscode.debug.breakpoints` for breakpoint information
- Access debug manager state from `src/debugger/manager.ts`
- Track session lifecycle via `vscode.debug.onDidStartDebugSession` and `vscode.debug.onDidTerminateDebugSession`

**Launch File Analysis:**
- Use existing `ros2_launch_dumper.py` via child_process
- Parse dumper output for node composition
- Enhance dumper if needed for:
  - Source package paths
  - Runtime type detection
  - Topic/service relationships

**Error Handling:**
- Return empty array `[]` if no active debug sessions
- Return error messages in MCP format for invalid inputs
- Handle cases where debug state is unavailable
- Graceful degradation when tools cannot access data

### Testing Requirements

#### Unit Tests
- Test each MCP tool registration and response format
- Mock VS Code debug APIs
- Verify JSON schema compliance
- Test error conditions

#### Integration Tests
- Start TypeScript MCP server (port 3003)
- Launch ROS debug session via RDE
- Call tools via MCP and verify responses
- Test with Python and XML launch files

#### Manual Testing
1. Start extension with MCP server enabled
2. Launch a ROS composition with debugger
3. Use Copilot/Cursor to query debug state
4. Verify accuracy of returned information

### Success Criteria
- [ ] TypeScript MCP server (`src/ros2-debug-mcp.ts`) created following RDE-URDF pattern
- [ ] All three MCP tools implemented and functional
- [ ] Tools return accurate debug session information
- [ ] Launch file analysis works for Python and XML files
- [ ] No performance degradation
- [ ] Documentation updated
- [ ] Tests pass

### Dependencies
- `@modelcontextprotocol/sdk`: ^0.5.0 (same as RDE-URDF)
- `express`: ^4.18.2 (same as RDE-URDF)

### Estimated Effort
**1-2 days** for a developer familiar with the codebase

### Priority
**High** - Foundational feature for AI-assisted debugging

### Related Documentation
- [RDE-URDF MCP Implementation](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts) - **Use this as the template**
- [DebugMCP Integration Investigation](DebugMCP-Integration-Investigation.md)
- [Feature Proposal](feature-proposals/debug-session-context-mcp-tools.md)

### Notes
- **TypeScript implementation only** - do NOT modify Python MCP server
- Follow RDE-URDF pattern exactly for consistency
- Python server (`assets/scripts/server.py`) is separate and handles ROS CLI only
- This provides value independently without DebugMCP

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

### Step 3: Create TypeScript MCP Server
1. Create `src/ros2-debug-mcp.ts` following [RDE-URDF pattern](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts)
2. Set up MCP server infrastructure with express
3. Configure server on port 3003

### Step 4: Implement MCP Tools
1. In `src/ros2-debug-mcp.ts`, register tool `get_active_debug_sessions()`
2. Register tool `get_node_debug_info(node_name: str)`
3. Register tool `get_launch_file_debug_info(launch_file: str)`
4. Implement each tool using VS Code debug APIs
5. Add error handling for all tools

### Step 5: Integrate with Extension
1. Edit `src/extension.ts` to import and start MCP server
2. Add server to extension subscriptions for cleanup
3. Configure server startup in activation

### Step 6: Enhance Launch Dumper (if needed)
1. Edit `assets/scripts/ros2_launch_dumper.py` if additional metadata needed
2. Add source package path detection
3. Add runtime type detection logic
4. Add topic/service relationship mapping

### Step 7: Testing
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
