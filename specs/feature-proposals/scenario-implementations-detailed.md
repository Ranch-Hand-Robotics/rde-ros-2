# Feature #2: Detailed Implementation Proposals for AI-Assisted Debugging Scenarios

## Overview

This document provides three detailed implementation proposals for AI-assisted debugging scenarios in RDE ROS 2. **All proposals use a TypeScript-only MCP server implementation** following the exact pattern from [RDE-URDF's mcp.ts](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts).

The TypeScript MCP server (`src/ros2-debug-mcp.ts`) will provide direct access to:
- Debug session state and management
- Launch file composition and node mapping
- Breakpoint control and crash analysis
- Performance profiling and validation tools

**Important:** This TypeScript MCP server is the ONLY way to expose extension-internal debugging features. The existing Python MCP server (`assets/scripts/server.py`) is separate and handles only ROS CLI commands - it is NOT used for any debugging features described in this document.

Each proposal outlines the scenario, required MCP tools, implementation approach, and integration strategy.

---

## Scenario 1: Automated Crash Investigation

### Description
When a ROS node crashes during debugging or execution, automatically identify the crashed node, analyze crash logs, set breakpoints near the failure point, and restart with debugger attached.

### User Story
*"As a ROS developer debugging a multi-node system, when a node crashes, I want the AI to automatically identify which node failed, suggest where to set breakpoints based on the crash log, and help me restart the session with debugging enabled on that specific node."*

### Example Workflow
1. User launches a ROS composition with 6 nodes
2. `webrtc_bridge` node crashes with a segfault
3. AI assistant detects the crash via MCP tools
4. AI analyzes crash log and identifies the crash occurred in `processFrame()` method
5. AI suggests setting breakpoints at the function entry and before the crash line
6. AI offers to restart the launch with debugger attached to `webrtc_bridge`
7. User confirms, and the node restarts with breakpoints pre-configured

### Required MCP Tools

#### Built-in TypeScript MCP Server Tools

**Tool 1: `get_active_processes`**
```typescript
@mcp.tool({
  title: 'Get Active ROS Processes',
  description: 'Returns information about all active ROS processes from the current debug session',
  inputSchema: {}
})
async get_active_processes(): Promise<ProcessInfo[]>
```

**Returns:**
```json
[
  {
    "node_name": "webrtc_bridge",
    "process_id": 12345,
    "status": "crashed",
    "exit_code": -11,
    "runtime": "C++",
    "executable": "/workspace/install/webrtc_ros/lib/webrtc_ros/bridge",
    "last_known_state": "running",
    "crash_timestamp": "2024-01-31T10:45:23Z"
  }
]
```

**Tool 2: `get_crash_analysis`**
```typescript
@mcp.tool({
  title: 'Get Crash Analysis',
  description: 'Analyzes crash logs and core dumps for a crashed node to identify failure location',
  inputSchema: {
    type: 'object',
    properties: {
      node_name: { type: 'string', description: 'Name of the crashed node' }
    },
    required: ['node_name']
  }
})
async get_crash_analysis(node_name: string): Promise<CrashAnalysis>
```

**Returns:**
```json
{
  "node_name": "webrtc_bridge",
  "crash_type": "segmentation_fault",
  "signal": "SIGSEGV",
  "stack_trace": [
    {
      "function": "processFrame",
      "file": "/workspace/src/webrtc_ros/src/bridge.cpp",
      "line": 142,
      "address": "0x7f8b4c123456"
    },
    {
      "function": "imageCallback",
      "file": "/workspace/src/webrtc_ros/src/bridge.cpp",
      "line": 98,
      "address": "0x7f8b4c123000"
    }
  ],
  "likely_cause": "null_pointer_dereference",
  "suggested_breakpoints": [
    { "file": "/workspace/src/webrtc_ros/src/bridge.cpp", "line": 140 },
    { "file": "/workspace/src/webrtc_ros/src/bridge.cpp", "line": 95 }
  ]
}
```

**Tool 3: `restart_node_with_debugger`**
```typescript
@mcp.tool({
  title: 'Restart Node with Debugger',
  description: 'Restarts a specific node from the launch composition with debugger attached and optional breakpoints',
  inputSchema: {
    type: 'object',
    properties: {
      node_name: { type: 'string', description: 'Name of the node to restart' },
      breakpoints: {
        type: 'array',
        items: {
          type: 'object',
          properties: {
            file: { type: 'string' },
            line: { type: 'number' }
          }
        },
        description: 'Optional breakpoints to set before starting'
      }
    },
    required: ['node_name']
  }
})
async restart_node_with_debugger(node_name: string, breakpoints?: Breakpoint[]): Promise<DebugSessionInfo>
```

**Tool 4: `get_launch_composition`**
```typescript
@mcp.tool({
  title: 'Get Launch Composition',
  description: 'Returns the complete launch file composition including node relationships and topics',
  inputSchema: {
    type: 'object',
    properties: {
      launch_file: { type: 'string', description: 'Path to the launch file (optional, uses active if not provided)' }
    }
  }
})
async get_launch_composition(launch_file?: string): Promise<LaunchComposition>
```

**Returns:**
```json
{
  "launch_file": "/workspace/src/webrtc_demo/launch/demo.launch.py",
  "nodes": [
    {
      "node_name": "webrtc_bridge",
      "package": "webrtc_ros",
      "executable": "bridge",
      "source_path": "/workspace/src/webrtc_ros/src/bridge.cpp"
    }
  ],
  "topic_graph": {
    "/image/raw": {
      "publishers": ["frame_generator"],
      "subscribers": ["webrtc_bridge"]
    }
  }
}
```

### Implementation Strategy

#### TypeScript Built-in MCP Server
Following the RDE-URDF pattern in `src/mcp.ts`:

1. **Create `src/ros2-ros2-debug-mcp.ts`**:
```typescript
import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import * as vscode from 'vscode';
import * as express from 'express';
import * as debugManager from './debugger/manager';
import * as rosUtils from './ros/utils';

export class Ros2Ros2DebugMcpServer {
  private server: McpServer;
  private app: express.Application;
  private port: number = 3003;
  private httpServer: any;
  private transports: { [sessionId: string]: StreamableHTTPServerTransport } = {};

  constructor(port: number = 3003) {
    this.port = port;
    this.app = express();
    this.app.use(express.json());
    
    this.server = new McpServer(
      {
        name: 'ros2-debug',
        description: 'ROS 2 Debug Context MCP Server',
        version: '1.0.0',
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupRoutes();
    this.setupCrashInvestigationTools();
  }

  private setupCrashInvestigationTools(): void {
    // Tool 1: Get active ROS processes
    this.server.registerTool('get_active_processes', {
      title: 'Get Active ROS Processes',
      description: 'Returns information about all active ROS processes from the current debug session',
      inputSchema: {}
    }, async (args) => {
      // Access debug session state from extension
      const debugSessions = vscode.debug.activeDebugSession;
      // Query ROS node process information
      // Return process status, PIDs, crash states
      return { content: [{ type: 'text', text: JSON.stringify(processInfo) }] };
    });

    // Tool 2: Analyze crash
    this.server.registerTool('get_crash_analysis', {
      title: 'Get Crash Analysis',
      description: 'Analyzes crash logs and core dumps for a crashed node to identify failure location',
      inputSchema: {
        type: 'object',
        properties: {
          node_name: { type: 'string', description: 'Name of the crashed node' }
        },
        required: ['node_name']
      }
    }, async (args) => {
      const nodeName = (args as any).node_name;
      // Parse ROS log files for crash information
      // Use VS Code debug adapter to extract stack traces
      // Analyze and suggest breakpoints
      return { content: [{ type: 'text', text: JSON.stringify(crashAnalysis) }] };
    });

    // Tool 3: Restart node with debugger
    this.server.registerTool('restart_node_with_debugger', {
      title: 'Restart Node with Debugger',
      description: 'Restarts a specific node from the launch composition with debugger attached',
      inputSchema: {
        type: 'object',
        properties: {
          node_name: { type: 'string', description: 'Name of the node to restart' },
          breakpoints: {
            type: 'array',
            items: {
              type: 'object',
              properties: {
                file: { type: 'string' },
                line: { type: 'number' }
              }
            }
          }
        },
        required: ['node_name']
      }
    }, async (args) => {
      const nodeName = (args as any).node_name;
      const breakpoints = (args as any).breakpoints || [];
      // Use vscode.debug.startDebugging() to restart with debugger
      // Pre-configure breakpoints via vscode.debug.addBreakpoints()
      return { content: [{ type: 'text', text: JSON.stringify(debugSession) }] };
    });

    // Tool 4: Get launch composition
    this.server.registerTool('get_launch_composition', {
      title: 'Get Launch Composition',
      description: 'Returns the complete launch file composition',
      inputSchema: {
        type: 'object',
        properties: {
          launch_file: { type: 'string' }
        }
      }
    }, async (args) => {
      const launchFile = (args as any).launch_file;
      // Use launch dumper to analyze composition
      // Return node relationships and topic graph
      return { content: [{ type: 'text', text: JSON.stringify(composition) }] };
    });
  }
}
```

2. **Integrate in `src/extension.ts`**:
```typescript
import { Ros2DebugMcpServer } from './ros2-debug-mcp';

let ros2DebugMcpServer: Ros2DebugMcpServer | null = null;

export async function activate(context: vscode.ExtensionContext) {
  // ... existing code ...
  
  // Start built-in ROS 2 debug MCP server
  ros2DebugMcpServer = new Ros2DebugMcpServer();
  await ros2DebugMcpServer.start();
  
  context.subscriptions.push({
    dispose: () => ros2DebugMcpServer?.stop()
  });
}
```

#### Integration Points

**Files to Modify:**
- `src/debugger/manager.ts` - Add crash detection and state export
- `src/debugger/debug-session.ts` - Add crash log capture
- `src/ros/utils.ts` - Add log file parsing utilities

**New Files:**
- `src/ros2-debug-mcp.ts` - Built-in MCP server implementation
- `src/debugger/crash-analyzer.ts` - Crash analysis logic
- `src/debugger/debug-state-tracker.ts` - Track debug session state

### Context Requirements

**Information Needed:**
1. Active debug session state (from VS Code debug API)
2. ROS log file locations (from ROS environment)
3. Launch file composition (from launch dumper)
4. Process monitoring (from debug manager)
5. Stack trace parsing (from debug adapters)

**VS Code APIs Used:**
- `vscode.debug.activeDebugSession`
- `vscode.debug.breakpoints`
- `vscode.debug.startDebugging()`
- `vscode.debug.onDidTerminateDebugSession`

---

## Scenario 2: End-to-End Validation

### Description
Automatically validate that a multi-node ROS composition is functioning correctly by publishing test messages, monitoring topic outputs, and verifying expected behaviors across the system.

### User Story
*"As a ROS developer, I want to verify that my WebRTC streaming pipeline (frame generator → bridge → web server) is working end-to-end by having the AI automatically publish test images, verify they appear on the web stream, and report any issues in the pipeline."*

### Example Workflow
1. User opens a launch file with frame generator, WebRTC bridge, and web server nodes
2. User asks AI: "Verify this pipeline works end-to-end"
3. AI uses MCP tools to:
   - Launch the composition with selective debugging
   - Publish test image to `/image/raw`
   - Monitor `/webrtc/stream` for output
   - Check web server endpoints for stream availability
   - Measure latency through the pipeline
4. AI reports: "Pipeline is functional. Average latency: 45ms. Detected minor frame drops at bridge node."
5. AI suggests: "Consider setting a breakpoint in bridge.cpp:L142 to investigate frame drops"

### Required MCP Tools

#### Built-in TypeScript MCP Server Tools

**Tool 1: `launch_with_debug_profile`**
```typescript
@mcp.tool({
  title: 'Launch with Debug Profile',
  description: 'Launches a ROS composition with selective debugging based on validation requirements',
  inputSchema: {
    type: 'object',
    properties: {
      launch_file: { type: 'string', description: 'Path to launch file' },
      debug_nodes: {
        type: 'array',
        items: { type: 'string' },
        description: 'Nodes to attach debugger to (empty = none)'
      },
      validation_mode: {
        type: 'boolean',
        description: 'Enable validation instrumentation'
      }
    },
    required: ['launch_file']
  }
})
async launch_with_debug_profile(
  launch_file: string, 
  debug_nodes?: string[], 
  validation_mode?: boolean
): Promise<LaunchSessionInfo>
```

**Tool 2: `validate_message_flow`**
```typescript
@mcp.tool({
  title: 'Validate Message Flow',
  description: 'Validates that messages flow correctly through a topic pipeline from publishers to subscribers',
  inputSchema: {
    type: 'object',
    properties: {
      topic_chain: {
        type: 'array',
        items: { type: 'string' },
        description: 'Chain of topics to validate (e.g., ["/image/raw", "/webrtc/stream"])'
      },
      test_duration_sec: {
        type: 'number',
        description: 'How long to monitor the flow'
      },
      expected_rate_hz: {
        type: 'number',
        description: 'Expected message rate'
      }
    },
    required: ['topic_chain', 'test_duration_sec']
  }
})
async validate_message_flow(
  topic_chain: string[], 
  test_duration_sec: number, 
  expected_rate_hz?: number
): Promise<ValidationResult>
```

**Returns:**
```json
{
  "status": "partial_success",
  "topic_results": [
    {
      "topic": "/image/raw",
      "publishers": ["frame_generator"],
      "subscribers": ["webrtc_bridge"],
      "message_count": 300,
      "actual_rate_hz": 30.2,
      "expected_rate_hz": 30.0,
      "status": "ok"
    },
    {
      "topic": "/webrtc/stream",
      "publishers": ["webrtc_bridge"],
      "subscribers": ["web_server"],
      "message_count": 285,
      "actual_rate_hz": 28.5,
      "expected_rate_hz": 30.0,
      "status": "degraded",
      "issues": ["frame_drops_detected"]
    }
  ],
  "overall_latency_ms": 45,
  "recommendations": [
    "Investigate frame drops in webrtc_bridge node",
    "Consider increasing bridge processing queue size"
  ]
}
```

**Tool 3: `publish_test_message`**
```typescript
@mcp.tool({
  title: 'Publish Test Message',
  description: 'Publishes a test message to a topic for validation purposes',
  inputSchema: {
    type: 'object',
    properties: {
      topic: { type: 'string', description: 'Topic to publish to' },
      message_type: { type: 'string', description: 'ROS message type' },
      message_data: { type: 'object', description: 'Message content' },
      count: { type: 'number', description: 'Number of messages to publish' }
    },
    required: ['topic', 'message_type', 'message_data']
  }
})
async publish_test_message(
  topic: string, 
  message_type: string, 
  message_data: any, 
  count?: number
): Promise<PublishResult>
```

**Tool 4: `check_endpoint_availability`**
```typescript
@mcp.tool({
  title: 'Check Endpoint Availability',
  description: 'Checks if a web endpoint (e.g., WebRTC stream URL) is available and responding',
  inputSchema: {
    type: 'object',
    properties: {
      url: { type: 'string', description: 'URL to check' },
      expected_status: { type: 'number', description: 'Expected HTTP status code' },
      timeout_ms: { type: 'number', description: 'Request timeout' }
    },
    required: ['url']
  }
})
async check_endpoint_availability(
  url: string, 
  expected_status?: number, 
  timeout_ms?: number
): Promise<EndpointCheckResult>
```

### Implementation Strategy

#### TypeScript Built-in MCP Server
Extend `src/ros2-ros2-debug-mcp.ts`:

```typescript
private setupValidationTools(): void {
  this.server.registerTool('launch_with_debug_profile', {
    title: 'Launch with Debug Profile',
    description: 'Launches a ROS composition with selective debugging',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Use existing ROS launch mechanisms from src/ros/cli.ts
    // Selectively attach debuggers based on debug_nodes parameter
    // Track launch session for validation
  });

  this.server.registerTool('validate_message_flow', {
    title: 'Validate Message Flow',
    description: 'Validates message flow through topic chains',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Execute ros2 topic commands to monitor topics
    // Measure message rates and latencies
    // Detect anomalies (drops, delays, etc.)
    // Return comprehensive validation report
  });

  this.server.registerTool('publish_test_message', {
    title: 'Publish Test Message',
    description: 'Publishes test messages to topics',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Use ros2 topic pub command via vscode.tasks
    // Support standard ROS message types
    // Handle message serialization
  });

  this.server.registerTool('check_endpoint_availability', {
    title: 'Check Endpoint Availability',
    description: 'Checks web endpoint availability',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Make HTTP request using Node.js http/https
    // Check response status and content
    // Handle timeouts gracefully
  });
}
```

#### Integration Points

**Files to Modify:**
- `src/ros2-ros2-debug-mcp.ts` - Add validation tools
- `src/ros/cli.ts` - Add selective debug launch capability
- `src/debugger/configuration/resolvers/ros2/launch.ts` - Support partial debugging

**New Files:**
- `src/validation/message-flow-validator.ts` - Message flow validation logic
- `src/validation/endpoint-checker.ts` - Web endpoint checking

### Context Requirements

**Information Needed:**
1. Launch file composition and node relationships (from launch dumper)
2. Topic publishers and subscribers mapping (from ROS graph)
3. Message type schemas (from ROS introspection)
4. Expected performance metrics (from configuration or user input)
5. Web endpoint URLs (from launch parameters or configuration)

**VS Code APIs Used:**
- `vscode.debug.startDebugging()` for selective debug launch
- `vscode.tasks.executeTask()` for ROS commands
- `vscode.workspace.fs` for file operations
- Node.js `http`/`https` for endpoint checking

---

## Scenario 3: Performance Debugging and Optimization

### Description
Automatically identify performance bottlenecks in a ROS pipeline, set conditional breakpoints based on performance metrics, and profile message flow through the system.

### User Story
*"As a ROS developer optimizing a real-time vision pipeline, I want the AI to identify which nodes are causing latency, set breakpoints when processing time exceeds thresholds, and suggest optimizations based on profiling data."*

### Example Workflow
1. User launches a vision pipeline with 5 processing nodes
2. User asks AI: "Profile this pipeline and find bottlenecks"
3. AI uses MCP tools to:
   - Attach performance profilers to each node
   - Monitor processing times for each node
   - Identify `feature_extraction` node is slow (120ms per frame)
   - Set conditional breakpoint when processing time > 100ms
   - Capture profiling data at the breakpoint
4. AI reports: "Bottleneck found in feature_extraction node. Processing time: 120ms (expected: 33ms for 30fps). Main overhead in SIFT feature computation."
5. AI suggests: "Consider using ORB features instead of SIFT for better performance, or parallelize feature extraction"

### Required MCP Tools

#### Built-in TypeScript MCP Server Tools

**Tool 1: `start_performance_profiling`**
```typescript
@mcp.tool({
  title: 'Start Performance Profiling',
  description: 'Starts performance profiling for specified nodes in the ROS composition',
  inputSchema: {
    type: 'object',
    properties: {
      node_names: {
        type: 'array',
        items: { type: 'string' },
        description: 'Nodes to profile (empty = all nodes)'
      },
      metrics: {
        type: 'array',
        items: {
          type: 'string',
          enum: ['cpu', 'memory', 'message_latency', 'callback_duration']
        },
        description: 'Metrics to collect'
      },
      duration_sec: {
        type: 'number',
        description: 'How long to profile'
      }
    },
    required: ['duration_sec']
  }
})
async start_performance_profiling(
  node_names?: string[], 
  metrics?: string[], 
  duration_sec: number
): Promise<ProfilingSessionInfo>
```

**Tool 2: `get_performance_report`**
```typescript
@mcp.tool({
  title: 'Get Performance Report',
  description: 'Retrieves performance profiling results and identifies bottlenecks',
  inputSchema: {
    type: 'object',
    properties: {
      session_id: { type: 'string', description: 'Profiling session ID' }
    },
    required: ['session_id']
  }
})
async get_performance_report(session_id: string): Promise<PerformanceReport>
```

**Returns:**
```json
{
  "session_id": "prof-12345",
  "duration_sec": 10,
  "nodes": [
    {
      "node_name": "feature_extraction",
      "avg_callback_duration_ms": 120,
      "max_callback_duration_ms": 185,
      "min_callback_duration_ms": 95,
      "callback_count": 300,
      "avg_cpu_percent": 85,
      "avg_memory_mb": 512,
      "message_latency_ms": 15,
      "status": "bottleneck",
      "bottleneck_severity": "high"
    },
    {
      "node_name": "object_detection",
      "avg_callback_duration_ms": 28,
      "callback_count": 300,
      "status": "ok"
    }
  ],
  "overall_pipeline_latency_ms": 165,
  "target_latency_ms": 33,
  "bottlenecks": [
    {
      "node": "feature_extraction",
      "metric": "callback_duration",
      "value": 120,
      "threshold": 33,
      "impact": "Pipeline cannot achieve 30fps target"
    }
  ],
  "recommendations": [
    "feature_extraction is the primary bottleneck (120ms vs 33ms target)",
    "Consider algorithm optimization or parallelization",
    "Review SIFT feature computation efficiency"
  ]
}
```

**Tool 3: `set_conditional_breakpoint`**
```typescript
@mcp.tool({
  title: 'Set Conditional Breakpoint',
  description: 'Sets a breakpoint that triggers when a specific condition is met (e.g., processing time exceeds threshold)',
  inputSchema: {
    type: 'object',
    properties: {
      node_name: { type: 'string', description: 'Node to set breakpoint in' },
      file: { type: 'string', description: 'Source file path' },
      line: { type: 'number', description: 'Line number' },
      condition: {
        type: 'string',
        description: 'Condition expression (e.g., "processing_time > 100")'
      },
      hit_condition: {
        type: 'string',
        description: 'Hit count condition (e.g., ">5" to break after 5 hits)'
      }
    },
    required: ['node_name', 'file', 'line', 'condition']
  }
})
async set_conditional_breakpoint(
  node_name: string,
  file: string,
  line: number,
  condition: string,
  hit_condition?: string
): Promise<BreakpointInfo>
```

**Tool 4: `get_message_flow_profiling`**
```typescript
@mcp.tool({
  title: 'Get Message Flow Profiling',
  description: 'Profiles message flow through a topic chain to identify latency sources',
  inputSchema: {
    type: 'object',
    properties: {
      topic_chain: {
        type: 'array',
        items: { type: 'string' },
        description: 'Topics to profile in order'
      },
      sample_count: {
        type: 'number',
        description: 'Number of messages to sample'
      }
    },
    required: ['topic_chain', 'sample_count']
  }
})
async get_message_flow_profiling(
  topic_chain: string[], 
  sample_count: number
): Promise<MessageFlowProfile>
```

**Returns:**
```json
{
  "topic_chain": ["/camera/image_raw", "/features", "/objects"],
  "total_latency_ms": 165,
  "stages": [
    {
      "from_topic": "/camera/image_raw",
      "to_topic": "/features",
      "processing_node": "feature_extraction",
      "avg_latency_ms": 120,
      "percentage_of_total": 72.7
    },
    {
      "from_topic": "/features",
      "to_topic": "/objects",
      "processing_node": "object_detection",
      "avg_latency_ms": 28,
      "percentage_of_total": 17.0
    }
  ],
  "slowest_stage": {
    "stage_index": 0,
    "node": "feature_extraction",
    "latency_ms": 120
  }
}
```

### Implementation Strategy

#### TypeScript Built-in MCP Server
Extend `src/ros2-ros2-debug-mcp.ts`:

```typescript
private setupPerformanceTools(): void {
  this.server.registerTool('start_performance_profiling', {
    title: 'Start Performance Profiling',
    description: 'Starts performance profiling for ROS nodes',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Execute ros2 node info commands for CPU/memory metrics
    // Monitor callback durations via debug instrumentation
    // Store profiling session data in extension state
    // Return session ID for later retrieval
  });

  this.server.registerTool('get_performance_report', {
    title: 'Get Performance Report',
    description: 'Retrieves performance profiling results',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Retrieve profiling session data from extension state
    // Analyze collected metrics for bottlenecks
    // Generate recommendations based on thresholds
    // Return comprehensive performance report
  });

  this.server.registerTool('set_conditional_breakpoint', {
    title: 'Set Conditional Breakpoint',
    description: 'Sets performance-based conditional breakpoints',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Use vscode.debug API to create conditional breakpoint
    // Support expressions like "processing_time > 100"
    // Handle hit count conditions
    // Return breakpoint info
  });

  this.server.registerTool('get_message_flow_profiling', {
    title: 'Get Message Flow Profiling',
    description: 'Profiles message latency through topic chains',
    inputSchema: { /* as shown above */ }
  }, async (args) => {
    // Execute ros2 topic echo commands to monitor topics
    // Measure inter-topic latencies using timestamps
    // Identify slowest processing stages
    // Return detailed flow profile
  });
}
```

#### Integration Points

**Files to Modify:**
- `src/ros2-ros2-debug-mcp.ts` - Add performance profiling tools
- `src/debugger/configuration/resolvers/ros2/launch.ts` - Support profiling mode

**New Files:**
- `src/performance/profiler.ts` - Performance profiling engine
- `src/performance/metrics-collector.ts` - Collect ROS node metrics
- `src/performance/bottleneck-analyzer.ts` - Analyze bottlenecks and generate recommendations

**Dependencies:**
- `perf` or similar profiling tools for CPU profiling
- ROS 2 performance analysis tools (if available)
- Message timestamp analysis for latency measurement

### Context Requirements

**Information Needed:**
1. Real-time CPU and memory metrics from ROS nodes
2. Message timestamps for latency calculation
3. Callback execution times (from instrumentation)
4. Source code locations for breakpoint setting
5. Performance targets/thresholds from configuration

**VS Code APIs Used:**
- `vscode.debug.addBreakpoints()` for conditional breakpoints
- `vscode.debug.activeDebugSession?.customRequest()` for debugger integration
- Custom debug adapter protocol messages for performance data

---

## Implementation Recommendations

### Recommended Approach: TypeScript MCP Server Only

**All features must be implemented using a TypeScript MCP server** following the exact pattern from [RDE-URDF's mcp.ts](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts):

**TypeScript MCP Server** (`src/ros2-debug-mcp.ts`, Port 3003):
- Debug session management and state tracking
- Breakpoint control and crash analysis
- Direct VS Code API integration
- Performance profiling coordination
- Launch composition analysis
- **Implementation Pattern**: https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts
- **Advantages**: 
  - Direct access to extension internals and VS Code debugging APIs
  - Type-safe TypeScript implementation
  - Integrated with extension lifecycle
  - No external process dependencies for debug features
  - Real-time access to debug session state

**Critical Note**: The existing Python MCP server (`assets/scripts/server.py`, Port 3002) is for ROS CLI commands ONLY. **DO NOT modify or reference the Python server** for debugging features. All debugging tools described in this document go in `src/ros2-debug-mcp.ts`.

### Priority Implementation Order

1. **Start with Scenario 1 (Crash Investigation)** - Most immediate value, foundational for other scenarios
2. **Then Scenario 2 (End-to-End Validation)** - Builds on crash investigation tools
3. **Finally Scenario 3 (Performance Profiling)** - Most complex, requires additional infrastructure

### Package Dependencies

**TypeScript MCP Server** (same as RDE-URDF):
```json
{
  "dependencies": {
    "@modelcontextprotocol/sdk": "^0.5.0",
    "express": "^4.18.2"
  }
}
```

---

## Implementation Steps

1. **Review and Approve** these proposals
2. **Clarify priorities** - which scenario to implement first?
3. **Install MCP SDK dependencies** - Add to `package.json`
4. **Create `src/ros2-debug-mcp.ts`** - Copy structure from [RDE-URDF mcp.ts](https://github.com/Ranch-Hand-Robotics/rde-urdf/blob/main/src/mcp.ts)
5. **Integrate with `src/extension.ts`** - Start MCP server on activation
6. **Implement tools incrementally** - Begin with Scenario 1
7. **Test with AI assistants** - Validate with Copilot/Cursor

---

## Questions for Maintainers

1. **Scenario Priority**: Which scenario provides the most value for your users?
   - [ ] Scenario 1: Crash Investigation
   - [ ] Scenario 2: End-to-End Validation
   - [ ] Scenario 3: Performance Profiling

2. **Port Configuration**: Is port 3003 acceptable for the TypeScript MCP server?
   - RDE-URDF uses 3005
   - Suggested: 3003 for ROS 2 Debug MCP

3. **Performance Profiling**: Do you have existing performance analysis tools or preferences?

4. **Testing**: What test environments are available?
   - Sample launch files with 6+ nodes?
   - Example crash scenarios?
   - Performance benchmarks?

5. **Timeline**: What's the desired rollout timeline for these features?
   - All at once (experimental feature flag)?
   - Incremental release (one scenario at a time)?
