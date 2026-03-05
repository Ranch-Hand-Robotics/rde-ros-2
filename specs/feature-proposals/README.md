# DebugMCP Integration Feature Proposals

This directory contains feature proposals for integrating Microsoft's DebugMCP with the Robot Developer Extension for ROS 2.

## Overview

These proposals stem from an investigation into enhancing RDE's debugging capabilities for complex multi-node ROS 2 systems through integration with DebugMCP and AI-assisted debugging workflows.

## Proposals

### 1. Debug Session Context MCP Tools
**Status**: Ready for implementation  
**Priority**: High  
**Dependencies**: None

Extends the RDE MCP server to expose debug session information to AI assistants. This is a foundational feature that can be implemented immediately without external dependencies.

[Read full proposal →](debug-session-context-mcp-tools.md)

### 2. DebugMCP Integration Layer - Scenario Implementations
**Status**: Detailed proposals ready for review  
**Priority**: Medium-High  
**Dependencies**: Proposal #1

Three detailed implementation proposals for AI-assisted debugging scenarios:
- **Scenario 1: Automated Crash Investigation** - Auto-detect crashes, analyze logs, set breakpoints, restart with debugger
- **Scenario 2: End-to-End Validation** - Validate multi-node pipelines, publish test messages, verify message flow
- **Scenario 3: Performance Debugging** - Profile pipelines, identify bottlenecks, set conditional breakpoints

Each proposal includes:
- Complete TypeScript MCP tool specifications
- Implementation using TypeScript MCP server (following RDE-URDF pattern)
- Integration points with existing RDE code
- VS Code API usage

**Pattern Reference**: https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts

[Read detailed proposals →](scenario-implementations-detailed.md)

### 3. DebugMCP Integration Layer (Original)
**Status**: Superseded by detailed scenario proposals  
**Priority**: Medium  
**Dependencies**: Proposal #1, Access to DebugMCP

Original high-level proposal for DebugMCP integration. See scenario-implementations-detailed.md for concrete implementation plans.

[Read original proposal →](debugmcp-integration-layer.md)

## Background Documentation

For complete context and investigation details, see:
- [DebugMCP Integration Investigation](../DebugMCP-Integration-Investigation.md)

## Implementation Status

- [ ] Proposal #1: Not started
- [ ] Proposal #2: Blocked (requires clarification)

## Contributing

When adding new proposals:
1. Create a new markdown file in this directory
2. Follow the template structure from existing proposals
3. Update this README with a summary
4. Link to the full investigation document for context
