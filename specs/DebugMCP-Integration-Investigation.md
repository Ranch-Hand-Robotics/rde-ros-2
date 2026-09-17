# DebugMCP Integration Investigation

## Executive Summary

This document outlines the investigation into integrating Microsoft's DebugMCP (Model Context Protocol for debugging) with the Robot Developer Extension for ROS 2. The goal is to enable AI-assisted debugging of complex multi-node ROS 2 launch compositions.

## Background

### Current RDE ROS 2 Debugging Capabilities

The Robot Developer Extension currently provides:

1. **Launch File Interception**: Intercepts ROS 2 launch files (XML and Python) and analyzes the process composition
2. **Multi-Process Debugging**: Automatically attaches appropriate debuggers (C++/Python) to each process in a launch composition
3. **Lifecycle Node Support**: Manages lifecycle state transitions during debugging
4. **Debug Configuration**: Provides `launch`, `debug_launch`, and `attach` debug configurations
5. **Launch File Dumper**: Python script (`ros2_launch_dumper.py`) that parses launch files and extracts process information

### Microsoft DebugMCP Overview

DebugMCP is a Visual Studio Code extension that exposes debugging capabilities via the Model Context Protocol (MCP). Key features:

- **Multi-language debugging**: Supports C++, Python, Java, and other languages
- **MCP-based API**: Allows AI assistants (GitHub Copilot, Cursor, etc.) to programmatically control debuggers
- **Available operations**:
  - Start/stop debug sessions
  - Step through code (step over, into, out)
  - Manage breakpoints (set, clear, list)
  - Inspect variables and expressions
  - Evaluate expressions in debug context
  - View call stacks and threads

### Current RDE MCP Server

The extension already implements an MCP server (`assets/scripts/server.py`) that provides 34 ROS-specific tools:

- Node management (list, inspect)
- Topic operations (list, echo, publish)
- Service operations (list, call)
- Parameter management
- Action operations
- Bag recording/playback
- Launch operations
- Lifecycle node management
- Package inspection

## Problem Statement

ROS 2 systems are inherently distributed, often running 6+ nodes from a single launch file. Debugging these systems is challenging:

1. **Cognitive Overload**: Developers must track multiple simultaneous debug sessions
2. **Complex Interactions**: Bugs may span multiple nodes (e.g., publisher → subscriber → service call chain)
3. **Manual Context Switching**: Developers manually switch between debugging different nodes
4. **Lack of End-to-End Testing**: No automated way to verify multi-node interactions

### Example Scenario

A typical ROS 2 composition for WebRTC streaming:
- **synthetic_frame_generator**: Publishes to `/image/raw`
- **webrtc_bridge**: Subscribes to `/image/raw`, streams via WebRTC
- **web_server**: Hosts webpage for WebRTC viewing
- **diagnostics_node**: Monitors system health
- **lifecycle_manager**: Manages node lifecycles
- **transform_broadcaster**: Publishes TF frames

When this system fails, developers need to:
1. Identify which node is causing the issue
2. Attach debuggers to relevant nodes
3. Set appropriate breakpoints
4. Step through code across multiple processes
5. Inspect message data flowing between nodes
6. Verify end-to-end functionality

## Integration Opportunities

### 1. Debug Context Provider for AI Assistants

**Capability**: Provide ROS launch composition context to DebugMCP-enabled AI assistants

**Implementation**:
- Extend current MCP server to expose debug session information
- Provide launch file analysis results (process list, dependencies, topics, services)
- Export debug configuration metadata (which nodes are being debugged, breakpoints set)

**MCP Tools Needed**:
```typescript
@mcp.tool()
async def get_active_debug_sessions() -> List[DebugSessionInfo]
  """Returns information about currently active debug sessions"""

@mcp.tool()
async def get_launch_composition_context(launch_file: str) -> LaunchContext
  """Analyzes a launch file and returns its composition context"""

@mcp.tool()
async def get_node_debug_info(node_name: str) -> NodeDebugInfo
  """Returns debugging information for a specific node"""
```

### 2. Automated Debugging Workflows

**Capability**: AI-assisted debugging based on system state and error patterns

**Use Cases**:
- **Crash Investigation**: When a node crashes, automatically:
  - Identify the crashed node from launch composition
  - Analyze crash logs
  - Set breakpoints near last known good state
  - Restart with debugger attached
  
- **End-to-End Validation**: Automatically:
  - Launch composition with debuggers
  - Publish test messages to input topics
  - Verify output on downstream topics
  - Check for expected state transitions
  
- **Performance Debugging**: Automatically:
  - Identify slow nodes in pipeline
  - Set conditional breakpoints on performance metrics
  - Profile message flow through system

**MCP Tools Needed**:
```typescript
@mcp.tool()
async def set_breakpoint_in_node(node_name: str, file: str, line: int) -> BreakpointInfo
  """Sets a breakpoint in a specific node's code"""

@mcp.tool()
async def get_message_flow_trace(topic: str, duration_sec: int) -> MessageTrace
  """Traces message flow through a topic across nodes"""

@mcp.tool()
async def validate_node_state(node_name: str, expected_state: dict) -> ValidationResult
  """Validates a node's internal state against expectations"""
```

### 3. Launch File Debug Annotations

**Capability**: Enhance launch files with debug metadata for AI assistants

**Implementation**:
- Add optional debug hints to launch file comments
- Specify which nodes are critical for debugging
- Define expected message flow patterns
- Provide validation criteria

**Example**:
```python
# @debug: critical_path=['frame_generator', 'webrtc_bridge', 'web_server']
# @debug: expected_topics=['/image/raw', '/webrtc/stream']
# @debug: validation={'frame_rate': '>=30Hz', 'latency': '<100ms'}

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='synthetic_vision', executable='frame_generator'),
        Node(package='webrtc_ros', executable='bridge'),
        Node(package='web_interface', executable='server'),
    ])
```

### 4. Integration with DebugMCP Extension

**Capability**: Direct integration with Microsoft's DebugMCP for unified debugging

**Approach**:
- RDE MCP server acts as orchestrator
- Delegates actual debugging operations to DebugMCP
- Provides ROS-specific context and intelligence layer
- Coordinates multi-process debugging across composition

**Architecture**:
```
┌─────────────────┐
│  AI Assistant   │
│ (Copilot/Cursor)│
└────────┬────────┘
         │ MCP
         ▼
┌─────────────────┐         ┌──────────────┐
│  RDE MCP Server │◄───────►│  DebugMCP    │
│  (ROS Context)  │  MCP    │  (Debug Ops) │
└────────┬────────┘         └──────┬───────┘
         │                          │
         ▼                          ▼
┌─────────────────┐         ┌──────────────┐
│  ROS 2 System   │         │  VS Code     │
│  (Nodes/Topics) │         │  Debuggers   │
└─────────────────┘         └──────────────┘
```

## Technical Considerations

### 1. Security and Permissions

- DebugMCP requires local server access (typically localhost:3001)
- RDE MCP server runs on configurable port (default: 3002)
- Need to ensure proper authentication between MCP servers
- Consider sandboxing for automated debugging operations

### 2. Performance and Scalability

- Debugging large compositions (10+ nodes) may be resource-intensive
- Need efficient filtering to attach debuggers only to relevant nodes
- Consider lazy attachment strategies
- Implement smart breakpoint management to avoid overwhelming the system

### 3. User Experience

- Provide clear visibility into which nodes are being debugged
- Offer opt-in for AI-assisted debugging features
- Allow manual override of automated debugging decisions
- Provide explanations for debugging actions taken by AI

### 4. Compatibility

- DebugMCP is currently VSCode-specific
- Need to ensure RDE debugging features work with and without DebugMCP
- Consider fallback strategies for environments without DebugMCP
- Maintain backward compatibility with existing debug configurations

## Open Questions Requiring Clarification

### For Maintainers

1. **Priority Use Cases**: Which debugging scenarios should be prioritized?
   - Crash investigation and root cause analysis?
   - End-to-end functional validation?
   - Performance debugging and optimization?
   - Interactive debugging assistance?

2. **Integration Scope**: What level of DebugMCP integration is desired?
   - Minimal: Just provide context to DebugMCP
   - Moderate: Orchestrate some automated workflows
   - Full: Deep integration with coordinated debugging

3. **User Opt-In**: How should users enable these features?
   - Automatic when DebugMCP is installed?
   - Explicit configuration required?
   - Per-launch-file basis?

4. **Existing Debugging Workflows**: Are there specific debugging patterns that are particularly painful today that we should address first?

5. **DebugMCP Access**: Do maintainers have access to DebugMCP for testing?
   - Can we set up a test environment?
   - Are there example debugging scenarios to validate against?

### Technical Questions

1. **MCP Server Communication**: Should RDE MCP server communicate directly with DebugMCP, or should AI assistants orchestrate between them?

2. **Debug State Persistence**: Should debug session state be persisted across ROS launches?

3. **Breakpoint Management**: How should breakpoints be managed across multiple instances of the same node?

4. **Error Handling**: How should the system handle debugging failures (e.g., node crashes during debug)?

## Recommended Next Steps

### Immediate Actions (No Additional Context Required)

1. **Prototype MCP Debug Tools**: Create proof-of-concept MCP tools for:
   - Listing active debug sessions
   - Getting launch composition context
   - Reporting node debug status

2. **Document Current Debug Flow**: Create detailed documentation of how RDE currently:
   - Parses launch files
   - Attaches debuggers to processes
   - Manages debug sessions
   - Handles lifecycle nodes during debugging

3. **Research DebugMCP API**: Deep dive into DebugMCP's MCP interface:
   - Available tools/operations
   - Request/response formats
   - Authentication mechanisms
   - Error handling patterns

### Actions Requiring Maintainer Input

1. **Define Example Scenarios**: Work with maintainers to define 2-3 concrete debugging scenarios that would benefit most from AI assistance

2. **Establish Success Criteria**: Define what "successful" DebugMCP integration looks like:
   - Specific time savings?
   - Reduced debugging steps?
   - Improved bug discovery rate?

3. **Create Integration Roadmap**: Based on priorities, create phased implementation plan:
   - Phase 1: Basic context exposure
   - Phase 2: Simple automated workflows
   - Phase 3: Advanced AI-assisted debugging

## Actionable Feature Items

Based on this investigation, the following feature items can be created (pending clarification on priorities):

### Feature 1: Debug Session Context MCP Tools
**Description**: Extend RDE MCP server with tools to expose debug session information to AI assistants
**Dependencies**: None
**Estimated Effort**: Small (1-2 days)
**Blocking Issues**: None

### Feature 2: Launch Composition Analysis Tool
**Description**: Add MCP tool to analyze launch files and provide structured composition information
**Dependencies**: None
**Estimated Effort**: Medium (3-5 days)
**Blocking Issues**: None

### Feature 3: DebugMCP Integration Layer
**Description**: Create integration layer for RDE to communicate with DebugMCP
**Dependencies**: Access to DebugMCP for testing
**Estimated Effort**: Large (1-2 weeks)
**Blocking Issues**: Need clarification on integration scope and test environment

### Feature 4: Automated Crash Investigation Workflow
**Description**: Implement AI-assisted workflow for investigating node crashes
**Dependencies**: Feature 1, Feature 3
**Estimated Effort**: Medium (4-6 days)
**Blocking Issues**: Need specific use case scenarios from maintainers

### Feature 5: End-to-End Validation Framework
**Description**: Create framework for automated validation of multi-node compositions
**Dependencies**: Feature 1, Feature 2
**Estimated Effort**: Large (1-2 weeks)
**Blocking Issues**: Need validation criteria and success metrics

## Conclusion

Integrating DebugMCP with RDE ROS 2 has significant potential to improve the debugging experience for complex multi-node systems. The extension already has strong foundations with its launch file interception and MCP server.

Key success factors:
1. **Clear prioritization** of use cases
2. **Maintainer input** on debugging pain points
3. **Incremental approach** starting with simple context exposure
4. **User opt-in** and clear communication about AI-assisted features
5. **Robust error handling** for complex debugging scenarios

The investigation reveals that several features can be implemented immediately (Features 1-2), while others require additional context and testing infrastructure (Features 3-5).

## References

- [Microsoft DebugMCP GitHub](https://github.com/microsoft/DebugMCP)
- [Model Context Protocol Specification](https://www.anthropic.com/mcp)
- [RDE ROS 2 Debug Support Documentation](https://ranchhandrobotics.com/rde-ros-2/debug-support.html)
- [RDE MCP Server Documentation](https://ranchhandrobotics.com/rde-ros-2/ModelContextProtocol.html)
