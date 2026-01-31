# Feature: DebugMCP Integration Layer

## Related Issue
Part of DebugMCP integration investigation

## Summary
Create an integration layer that allows the RDE ROS 2 extension to communicate with Microsoft's DebugMCP extension, enabling coordinated debugging of multi-node ROS compositions.

## Motivation
DebugMCP provides powerful debugging capabilities through MCP, but it lacks ROS-specific context. RDE knows about launch compositions, node relationships, and message flows, but cannot directly control debuggers via MCP. An integration layer would:
- Allow RDE to delegate debugging operations to DebugMCP
- Provide ROS context to DebugMCP for intelligent debugging
- Enable AI assistants to orchestrate debugging across multiple ROS nodes
- Support automated debugging workflows

## Proposed Solution

Create a bidirectional integration between RDE MCP server and DebugMCP:

1. **MCP Client in RDE**: Add capability for RDE MCP server to act as an MCP client to DebugMCP
2. **ROS Context Provider**: Expose ROS-specific debugging context to DebugMCP
3. **Debug Orchestration Tools**: Add MCP tools that coordinate debugging across nodes
4. **State Synchronization**: Keep RDE aware of DebugMCP's debugging state

## Implementation Details

**New Files:**
- `assets/scripts/debugmcp_client.py`: MCP client for communicating with DebugMCP
- `assets/scripts/debug_orchestrator.py`: Orchestration logic for multi-node debugging

**Files to Modify:**
- `assets/scripts/server.py`: Add integration layer and new MCP tools
- `src/extension.ts`: Add configuration for DebugMCP integration

## Dependencies
- Feature: Debug Session Context MCP Tools (provides foundation)
- External: DebugMCP extension must be installed
- Testing: Access to DebugMCP for integration testing

## Blocking Issues
1. Need access to DebugMCP for testing
2. Need clarification on integration scope (minimal vs. full integration)
3. Need to understand DebugMCP's MCP API specification

## Estimated Effort
1-2 weeks for a developer familiar with both RDE and MCP protocols

## Implementation Priority
Medium - Provides significant value but requires external dependencies and testing infrastructure

## Open Questions
1. Should RDE MCP server communicate directly with DebugMCP, or should AI assistants orchestrate?
2. What authentication/security is needed between MCP servers?
3. How to handle DebugMCP availability (graceful degradation)?
