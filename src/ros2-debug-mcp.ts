// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import { SSEServerTransport } from '@modelcontextprotocol/sdk/server/sse.js';
import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { 
  ErrorCode,
  McpError,
  ListToolsRequestSchema,
  CallToolRequestSchema,
} from '@modelcontextprotocol/sdk/types.js';
import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as child_process from 'child_process';
import * as util from 'util';
import * as express from 'express';
import { randomUUID } from 'node:crypto';
import { debugStateTracker } from './debugger/debug-state-tracker';
import * as extension from './extension';

const promisifiedExec = util.promisify(child_process.exec);

/**
 * ROS 2 Debug MCP Server Implementation
 * 
 * This MCP server provides debug session context and launch file analysis for ROS 2.
 * It exposes information about active debug sessions, node debugging status, and launch file composition.
 */
export class Ros2DebugMcpServer {
  private server: Server;
  private app: express.Application;
  private transports: { [sessionId: string]: SSEServerTransport } = {};
  private isRunning = false;
  private port: number;
  private httpServer: any;

  constructor(port: number = 3003) {
    this.port = port;
    this.app = express();
    this.app.use(express.json());
    
    this.server = new Server(
      {
        name: 'ros2-debug',
        version: '1.0.0',
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupRoutes();
    this.setupTools();
  }

  private setupTools(): void {
    // Register tools/list handler
    this.server.setRequestHandler(ListToolsRequestSchema, async () => {
      return {
        tools: [
          {
            name: 'get_active_debug_sessions',
            description: 'Returns information about all active ROS 2 debug sessions managed by RDE',
            inputSchema: {
              type: 'object',
              properties: {}
            }
          },
          {
            name: 'get_node_debug_info',
            description: 'Returns debugging information for a specific ROS node by name',
            inputSchema: {
              type: 'object',
              properties: {
                node_name: {
                  type: 'string',
                  description: 'Name of the ROS node to get debug information for'
                }
              },
              required: ['node_name']
            }
          },
          {
            name: 'get_launch_file_debug_info',
            description: 'Analyzes a launch file and returns debugging-relevant information including nodes, topics, and lifecycle nodes',
            inputSchema: {
              type: 'object',
              properties: {
                launch_file: {
                  type: 'string',
                  description: 'Path to the launch file (.py or .xml)'
                }
              },
              required: ['launch_file']
            }
          }
        ]
      };
    });

    // Register tools/call handler
    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const toolName = request.params.name;
      const args = request.params.arguments || {};

      switch (toolName) {
        case 'get_active_debug_sessions':
          return await this.handleGetActiveDebugSessions(args);
        case 'get_node_debug_info':
          return await this.handleGetNodeDebugInfo(args);
        case 'get_launch_file_debug_info':
          return await this.handleGetLaunchFileDebugInfo(args);
        default:
          throw new McpError(ErrorCode.MethodNotFound, `Unknown tool: ${toolName}`);
      }
    });
  }

  private async handleGetActiveDebugSessions(args: any): Promise<any> {
    try {
      const sessions = debugStateTracker.getActiveSessions();
      
      extension.outputChannel?.appendLine(`MCP Debug Server: Returning ${sessions.length} active debug sessions`);
      
      return {
        content: [{
          type: 'text',
          text: JSON.stringify(sessions, null, 2)
        }]
      };
    } catch (error) {
      extension.outputChannel?.appendLine(`MCP Debug Server error: ${error instanceof Error ? error.message : String(error)}`);
      
      if (error instanceof McpError) {
        throw error;
      }
      
      throw new McpError(
        ErrorCode.InternalError,
        `Failed to get active debug sessions: ${error instanceof Error ? error.message : String(error)}`
      );
    }
  }

  private async handleGetNodeDebugInfo(args: any): Promise<any> {
    try {
      const nodeName = (args as any).node_name;
      
      extension.outputChannel?.appendLine(`MCP Debug Server: Getting debug info for node: ${nodeName}`);
      
      const nodeInfo = debugStateTracker.getNodeDebugInfo(nodeName);
      const session = debugStateTracker.getSessionForNode(nodeName);
      
      if (!nodeInfo) {
        return {
          content: [{
            type: 'text',
            text: JSON.stringify({
              error: `Node '${nodeName}' not found in any active debug session`,
              available_nodes: debugStateTracker.getActiveSessions()
                .flatMap(s => s.nodes.map(n => n.node_name))
            }, null, 2)
          }]
        };
      }
      
      const response = {
        node_name: nodeInfo.node_name,
        executable: nodeInfo.executable,
        source_file: nodeInfo.source_file,
        process_id: nodeInfo.process_id,
        runtime: nodeInfo.runtime,
        debug_session_id: session?.session_id,
        debug_status: nodeInfo.debug_status
      };
      
      return {
        content: [{
          type: 'text',
          text: JSON.stringify(response, null, 2)
        }]
      };
    } catch (error) {
      extension.outputChannel?.appendLine(`MCP Debug Server error: ${error instanceof Error ? error.message : String(error)}`);
      
      if (error instanceof McpError) {
        throw error;
      }
      
      throw new McpError(
        ErrorCode.InternalError,
        `Failed to get node debug info: ${error instanceof Error ? error.message : String(error)}`
      );
    }
  }

  private async handleGetLaunchFileDebugInfo(args: any): Promise<any> {
    try {
      const launchFile = (args as any).launch_file;
      
      extension.outputChannel?.appendLine(`MCP Debug Server: Analyzing launch file: ${launchFile}`);
      
      // Resolve the launch file path
      let resolvedPath = launchFile;
      if (!path.isAbsolute(launchFile)) {
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
        if (workspaceRoot) {
          resolvedPath = path.join(workspaceRoot, launchFile);
        }
      }
      
      // Check if file exists
      if (!fs.existsSync(resolvedPath)) {
        return {
          content: [{
            type: 'text',
            text: JSON.stringify({
              error: `Launch file not found: ${resolvedPath}`
            }, null, 2)
          }]
        };
      }
      
      // Use the ros2_launch_dumper.py script to analyze the launch file
      const dumperPath = path.join(extension.extPath, 'assets', 'scripts', 'ros2_launch_dumper.py');
      
      if (!fs.existsSync(dumperPath)) {
        throw new Error(`Launch dumper script not found at ${dumperPath}`);
      }
      
      // Get ROS environment
      const rosEnv = await extension.resolvedEnv();
      
      // Determine Python executable from virtual environment
      const venvPath = path.join(extension.extPath, ".venv");
      const pythonExecutable = process.platform === "win32" 
          ? path.join(venvPath, "Scripts", "python3.exe")
          : path.join(venvPath, "bin", "python3");
      
      // Execute the dumper
      const command = `"${pythonExecutable}" "${dumperPath}" "${resolvedPath}"`;
      
      extension.outputChannel?.appendLine(`MCP Debug Server: Executing launch dumper: ${command}`);
      
      const { stdout, stderr } = await promisifiedExec(command, {
        env: { ...process.env, ...rosEnv },
        maxBuffer: 10 * 1024 * 1024 // 10MB buffer
      });
      
      if (stderr) {
        extension.outputChannel?.appendLine(`Launch dumper stderr: ${stderr}`);
      }
      
      // Parse the JSON output from the dumper
      let launchData;
      try {
        launchData = JSON.parse(stdout);
      } catch (parseError) {
        throw new Error(`Failed to parse launch dumper output: ${parseError}`);
      }
      
      // Transform the dumper output to the expected format
      const nodes = launchData.processes?.map((proc: any) => ({
        node_name: proc.node_name || path.basename(proc.executable),
        package: this.extractPackageName(proc.executable),
        executable: proc.executable,
        runtime: this.detectRuntime(proc.executable),
        debuggable: this.isDebuggable(proc.executable),
        source_package_path: this.findSourcePackagePath(proc.executable)
      })) || [];
      
      // Add lifecycle nodes
      const lifecycleNodes = launchData.lifecycle_nodes?.map((node: any) => ({
        node_name: node.node_name,
        namespace: node.namespace,
        package: node.package,
        executable: node.executable,
        runtime: this.detectRuntime(node.executable),
        debuggable: true,
        parameters: node.parameters
      })) || [];
      
      const response = {
        launch_file: resolvedPath,
        nodes: nodes,
        topics: [], // Topics require runtime analysis, not available from static launch file
        lifecycle_nodes: lifecycleNodes,
        warnings: launchData.warnings || [],
        errors: launchData.errors || []
      };
      
      extension.outputChannel?.appendLine(`MCP Debug Server: Found ${nodes.length} nodes and ${lifecycleNodes.length} lifecycle nodes`);
      
      return {
        content: [{
          type: 'text',
          text: JSON.stringify(response, null, 2)
        }]
      };
    } catch (error) {
      extension.outputChannel?.appendLine(`MCP Debug Server error analyzing launch file: ${error instanceof Error ? error.message : String(error)}`);
      
      if (error instanceof McpError) {
        throw error;
      }
      
      throw new McpError(
        ErrorCode.InternalError,
        `Failed to analyze launch file: ${error instanceof Error ? error.message : String(error)}`
      );
    }
  }

  /**
   * Extracts package name from executable path
   */
  private extractPackageName(executablePath: string): string {
    const parts = executablePath.split(path.sep);
    const installIndex = parts.lastIndexOf('install');
    if (installIndex !== -1 && installIndex + 1 < parts.length) {
      return parts[installIndex + 1];
    }
    return 'unknown';
  }

  /**
   * Detects the runtime type of an executable
   */
  private detectRuntime(executablePath: string): string {
    const ext = path.extname(executablePath);
    
    if (ext === '.py') {
      return 'Python';
    }
    
    // Check shebang for scripts without extension
    if (fs.existsSync(executablePath)) {
      try {
        const content = fs.readFileSync(executablePath, 'utf8');
        const firstLine = content.split('\n')[0];
        
        if (firstLine.startsWith('#!')) {
          if (firstLine.includes('python')) {
            return 'Python';
          }
        }
      } catch (error) {
        // Ignore read errors
      }
    }
    
    // Assume C++ for compiled executables
    return 'C++';
  }

  /**
   * Determines if an executable is debuggable
   */
  private isDebuggable(executablePath: string): boolean {
    const ext = path.extname(executablePath);
    
    // Shell scripts are not debuggable
    if (['.sh', '.bash', '.bat', '.cmd'].includes(ext)) {
      return false;
    }
    
    return true;
  }

  /**
   * Attempts to find the source package path for an executable
   */
  private findSourcePackagePath(executablePath: string): string | null {
    const parts = executablePath.split(path.sep);
    const installIndex = parts.lastIndexOf('install');
    
    if (installIndex === -1) {
      return null;
    }
    
    const workspaceRoot = parts.slice(0, installIndex).join(path.sep);
    const packageName = parts[installIndex + 1];
    
    if (!workspaceRoot || !packageName) {
      return null;
    }
    
    // Try to find the package in the source directory
    const srcPath = path.join(workspaceRoot, 'src');
    if (!fs.existsSync(srcPath)) {
      return null;
    }
    
    // Search for package.xml files
    const searchPackage = (dir: string): string | null => {
      try {
        const entries = fs.readdirSync(dir, { withFileTypes: true });
        
        for (const entry of entries) {
          if (entry.isDirectory()) {
            const packageXmlPath = path.join(dir, entry.name, 'package.xml');
            if (fs.existsSync(packageXmlPath)) {
              const content = fs.readFileSync(packageXmlPath, 'utf8');
              if (content.includes(`<name>${packageName}</name>`)) {
                return path.join(dir, entry.name);
              }
            }
            
            // Recursively search subdirectories
            const result = searchPackage(path.join(dir, entry.name));
            if (result) {
              return result;
            }
          }
        }
      } catch (error) {
        // Ignore read errors
      }
      
      return null;
    };
    
    return searchPackage(srcPath);
  }

  private setupRoutes(): void {
    // Health check endpoint
    this.app.get('/health', (req, res) => {
      res.json({ status: 'ok', server: 'ros2-debug-mcp' });
    });

    // SSE endpoint for MCP communication
    this.app.get('/sse', async (req, res) => {
      const sessionId = randomUUID();
      extension.outputChannel?.appendLine(`MCP Debug Server: New SSE connection (session ${sessionId})`);

      const transport = new SSEServerTransport('/message', res);
      this.transports[sessionId] = transport;

      await this.server.connect(transport);
      await transport.start();

      req.on('close', async () => {
        extension.outputChannel?.appendLine(`MCP Debug Server: SSE connection closed (session ${sessionId})`);
        await transport.close();
        delete this.transports[sessionId];
      });
    });

    // Message endpoint for MCP communication
    this.app.post('/message', async (req, res) => {
      const sessionId = Object.keys(this.transports).find(id => 
        this.transports[id] !== undefined
      );

      if (!sessionId || !this.transports[sessionId]) {
        res.status(400).json({ error: 'No active transport' });
        return;
      }

      await this.transports[sessionId].handlePostMessage(req, res);
    });
  }

  /**
   * Starts the MCP server
   */
  public async start(): Promise<void> {
    if (this.isRunning) {
      extension.outputChannel?.appendLine('MCP Debug Server is already running');
      return;
    }

    return new Promise((resolve, reject) => {
      this.httpServer = this.app.listen(this.port, () => {
        this.isRunning = true;
        extension.outputChannel?.appendLine(`MCP Debug Server started on port ${this.port}`);
        extension.outputChannel?.appendLine(`SSE endpoint: http://localhost:${this.port}/sse`);
        resolve();
      });

      this.httpServer.on('error', (error: any) => {
        extension.outputChannel?.appendLine(`MCP Debug Server error: ${error.message}`);
        reject(error);
      });
    });
  }

  /**
   * Stops the MCP server
   */
  public async stop(): Promise<void> {
    if (!this.isRunning) {
      return;
    }

    // Close all transports
    for (const sessionId in this.transports) {
      delete this.transports[sessionId];
    }

    // Close HTTP server
    if (this.httpServer) {
      await new Promise<void>((resolve) => {
        this.httpServer.close(() => {
          extension.outputChannel?.appendLine('MCP Debug Server stopped');
          this.isRunning = false;
          resolve();
        });
      });
    }
  }

  /**
   * Returns whether the server is running
   */
  public get running(): boolean {
    return this.isRunning;
  }
}
