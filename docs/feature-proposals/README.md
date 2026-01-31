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

### 2. DebugMCP Integration Layer
**Status**: Requires clarification  
**Priority**: Medium  
**Dependencies**: Proposal #1, Access to DebugMCP

Creates a bidirectional integration between RDE and DebugMCP for coordinated multi-node debugging. Requires access to DebugMCP for testing and clarification on integration scope.

[Read full proposal →](debugmcp-integration-layer.md)

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
