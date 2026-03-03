# Launch File Tree Viewer - Technical Implementation Guide

This document provides detailed technical specifications for implementing the Launch File Tree Viewer in the VS Code ROS 2 extension.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      VS Code Extension                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │         LaunchTreeDataProvider                      │    │
│  │  (implements vscode.TreeDataProvider)              │    │
│  │                                                     │    │
│  │  - getTreeItem(element)                            │    │
│  │  - getChildren(element?)                           │    │
│  │  - refresh()                                       │    │
│  │  - onDidChangeTreeData: Event                      │    │
│  └──────────────────┬──────────────────────────────────┘    │
│                     │                                        │
│                     ▼                                        │
│  ┌────────────────────────────────────────────────────┐    │
│  │           LaunchFileParser                          │    │
│  │                                                     │    │
│  │  - parseLaunchFile(filePath)                       │    │
│  │  - findLaunchFiles(workspace)                      │    │
│  │  - extractNodes(launchData)                        │    │
│  └──────────────────┬──────────────────────────────────┘    │
│                     │                                        │
│                     ▼                                        │
│  ┌────────────────────────────────────────────────────┐    │
│  │        ros2_launch_dumper.py (Python)              │    │
│  │                                                     │    │
│  │  Parses .launch.py files and returns JSON:         │    │
│  │  - nodes (IJsonProcess)                            │    │
│  │  - lifecycle_nodes (IJsonLifecycleNode)            │    │
│  │  - includes (IncludeLaunchDescription)             │    │
│  │  - arguments (DeclareLaunchArgument)               │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

## File Structure

### New Files to Create

```
src/
├── ros/
│   ├── launch-tree/
│   │   ├── launch-tree-provider.ts      # Main TreeDataProvider
│   │   ├── launch-tree-item.ts          # Tree item classes
│   │   ├── launch-parser.ts             # Launch file parsing logic
│   │   └── types.ts                     # Type definitions
│   └── commands/
│       └── launch-tree-commands.ts      # Command handlers
│
test/
└── suite/
    └── launch-tree/
        ├── launch-tree-provider.test.ts # Provider tests
        ├── launch-parser.test.ts        # Parser tests
        └── fixtures/
            ├── simple.launch.py         # Test launch file
            └── complex.launch.py        # Complex test case
```

### Files to Modify

```
src/
├── extension.ts                         # Register tree view & commands
└── debugger/
    └── configuration/
        └── resolvers/
            └── ros2/
                └── launch.ts            # Export interfaces for reuse

package.json                             # Add views & commands
```

## Type Definitions

### types.ts

```typescript
import * as vscode from 'vscode';

/**
 * Types of items that can appear in the launch tree
 */
export enum LaunchTreeItemType {
  Workspace = 'workspace',
  Package = 'package',
  LaunchFile = 'launchFile',
  Node = 'node',
  LifecycleNode = 'lifecycleNode',
  Include = 'include',
  ParameterGroup = 'parameterGroup',
  Parameter = 'parameter',
  ArgumentGroup = 'argumentGroup',
  Argument = 'argument',
  RemapGroup = 'remapGroup',
  Remap = 'remap',
  ExecuteProcess = 'executeProcess',
  Error = 'error',
  Loading = 'loading'
}

/**
 * Base interface for all tree items
 */
export interface ILaunchTreeItem {
  type: LaunchTreeItemType;
  label: string;
  description?: string;
  tooltip?: string;
  contextValue?: string;
  resourceUri?: vscode.Uri;
  children?: ILaunchTreeItem[];
  collapsibleState?: vscode.TreeItemCollapsibleState;
}

/**
 * Package-level tree item
 */
export interface IPackageTreeItem extends ILaunchTreeItem {
  type: LaunchTreeItemType.Package;
  packageName: string;
  packagePath: string;
}

/**
 * Launch file tree item
 */
export interface ILaunchFileTreeItem extends ILaunchTreeItem {
  type: LaunchTreeItemType.LaunchFile;
  packageName: string;
  fileName: string;
  filePath: string;
  parsed?: boolean;
  parseError?: string;
}

/**
 * Node tree item
 */
export interface INodeTreeItem extends ILaunchTreeItem {
  type: LaunchTreeItemType.Node | LaunchTreeItemType.LifecycleNode;
  nodeName: string;
  packageName: string;
  executable: string;
  namespace?: string;
  launchFilePath: string;
  lineNumber?: number;
  parameters?: Record<string, any>;
  remappings?: Array<[string, string]>;
  arguments?: string[];
}

/**
 * Include tree item
 */
export interface IIncludeTreeItem extends ILaunchTreeItem {
  type: LaunchTreeItemType.Include;
  targetPath: string;
  exists: boolean;
}

/**
 * Parameter tree item
 */
export interface IParameterTreeItem extends ILaunchTreeItem {
  type: LaunchTreeItemType.Parameter;
  parameterName: string;
  parameterValue: any;
  valueType: string;
}

/**
 * Launch file parse result (from ros2_launch_dumper.py)
 */
export interface ILaunchFileData {
  processes: IJsonProcess[];
  lifecycle_nodes: IJsonLifecycleNode[];
  includes: string[];
  arguments: ILaunchArgument[];
  warnings: string[];
  errors: string[];
}

/**
 * Process definition (from launch.ts)
 */
export interface IJsonProcess {
  type: string;
  command: string;
  node_name?: string;
  executable?: string;
  arguments?: string[];
  package?: string;
  namespace?: string;
  parameters?: Record<string, any>;
  remappings?: Array<[string, string]>;
}

/**
 * Lifecycle node definition (from launch.ts)
 */
export interface IJsonLifecycleNode {
  type: 'lifecycle';
  node_name: string;
  namespace?: string;
  package: string;
  executable: string;
  parameters?: Record<string, any>;
  remappings?: Array<[string, string]>;
}

/**
 * Launch argument definition
 */
export interface ILaunchArgument {
  name: string;
  default?: any;
  description?: string;
  choices?: any[];
}

/**
 * Workspace structure
 */
export interface IWorkspacePackage {
  name: string;
  path: string;
  launchFiles: string[];
}
```

## Implementation Details

### 1. LaunchTreeDataProvider

```typescript
// src/ros/launch-tree/launch-tree-provider.ts

import * as vscode from 'vscode';
import * as path from 'path';
import { LaunchTreeItem } from './launch-tree-item';
import { LaunchFileParser } from './launch-parser';
import {
  LaunchTreeItemType,
  ILaunchTreeItem,
  IWorkspacePackage
} from './types';

export class LaunchTreeDataProvider implements vscode.TreeDataProvider<LaunchTreeItem> {
  private _onDidChangeTreeData = new vscode.EventEmitter<LaunchTreeItem | undefined | void>();
  readonly onDidChangeTreeData = this._onDidChangeTreeData.event;

  private parser: LaunchFileParser;
  private workspacePackages: IWorkspacePackage[] = [];
  private fileWatcher?: vscode.FileSystemWatcher;

  constructor(
    private context: vscode.ExtensionContext,
    private outputChannel: vscode.OutputChannel
  ) {
    this.parser = new LaunchFileParser(outputChannel);
    this.setupFileWatcher();
  }

  /**
   * Refresh the tree view
   */
  refresh(): void {
    this._onDidChangeTreeData.fire();
  }

  /**
   * Get tree item for display
   */
  getTreeItem(element: LaunchTreeItem): vscode.TreeItem {
    return element;
  }

  /**
   * Get children of a tree item
   */
  async getChildren(element?: LaunchTreeItem): Promise<LaunchTreeItem[]> {
    if (!element) {
      // Root level - return workspace packages
      return this.getRootItems();
    }

    // Return children based on item type
    switch (element.type) {
      case LaunchTreeItemType.Package:
        return this.getPackageLaunchFiles(element);
      
      case LaunchTreeItemType.LaunchFile:
        return this.getLaunchFileContents(element);
      
      case LaunchTreeItemType.Node:
      case LaunchTreeItemType.LifecycleNode:
        return this.getNodeDetails(element);
      
      case LaunchTreeItemType.ParameterGroup:
        return this.getParameters(element);
      
      case LaunchTreeItemType.RemapGroup:
        return this.getRemappings(element);
      
      case LaunchTreeItemType.ArgumentGroup:
        return this.getArguments(element);
      
      default:
        return [];
    }
  }

  /**
   * Get root items (workspace packages with launch files)
   */
  private async getRootItems(): Promise<LaunchTreeItem[]> {
    const workspaceFolders = vscode.workspace.workspaceFolders;
    
    if (!workspaceFolders || workspaceFolders.length === 0) {
      return [this.createEmptyStateItem('No workspace opened')];
    }

    try {
      // Find all ROS packages with launch files
      this.workspacePackages = await this.parser.findWorkspacePackages(
        workspaceFolders.map(f => f.uri.fsPath)
      );

      if (this.workspacePackages.length === 0) {
        return [this.createEmptyStateItem('No launch files found in workspace')];
      }

      // Create package tree items
      return this.workspacePackages.map(pkg => 
        LaunchTreeItem.createPackageItem(pkg)
      );
    } catch (error) {
      this.outputChannel.appendLine(`Error scanning workspace: ${error}`);
      return [this.createErrorItem('Failed to scan workspace')];
    }
  }

  /**
   * Get launch files for a package
   */
  private async getPackageLaunchFiles(
    packageItem: LaunchTreeItem
  ): Promise<LaunchTreeItem[]> {
    const pkg = this.workspacePackages.find(p => p.name === packageItem.packageName);
    
    if (!pkg) {
      return [];
    }

    return pkg.launchFiles.map(filePath => 
      LaunchTreeItem.createLaunchFileItem(pkg.name, filePath)
    );
  }

  /**
   * Get contents of a launch file (nodes, includes, etc.)
   */
  private async getLaunchFileContents(
    launchFileItem: LaunchTreeItem
  ): Promise<LaunchTreeItem[]> {
    const filePath = launchFileItem.launchFilePath;
    
    if (!filePath) {
      return [];
    }

    try {
      // Parse launch file using ros2_launch_dumper.py
      const launchData = await this.parser.parseLaunchFile(filePath);

      const items: LaunchTreeItem[] = [];

      // Add nodes
      if (launchData.processes.length > 0) {
        items.push(...launchData.processes.map(proc => 
          LaunchTreeItem.createNodeItem(proc, filePath)
        ));
      }

      // Add lifecycle nodes
      if (launchData.lifecycle_nodes.length > 0) {
        items.push(...launchData.lifecycle_nodes.map(node => 
          LaunchTreeItem.createLifecycleNodeItem(node, filePath)
        ));
      }

      // Add includes
      if (launchData.includes.length > 0) {
        items.push(...launchData.includes.map(includePath => 
          LaunchTreeItem.createIncludeItem(includePath)
        ));
      }

      // Add arguments
      if (launchData.arguments.length > 0) {
        items.push(
          LaunchTreeItem.createArgumentGroupItem(launchData.arguments)
        );
      }

      // Show warnings if any
      if (launchData.warnings.length > 0) {
        items.push(
          LaunchTreeItem.createWarningItem(launchData.warnings.join(', '))
        );
      }

      // Show errors if any
      if (launchData.errors.length > 0) {
        items.push(
          LaunchTreeItem.createErrorItem(launchData.errors.join(', '))
        );
      }

      return items;
    } catch (error) {
      this.outputChannel.appendLine(`Error parsing ${filePath}: ${error}`);
      return [LaunchTreeItem.createErrorItem(`Parse error: ${error}`)];
    }
  }

  /**
   * Get details for a node (parameters, remappings, etc.)
   */
  private getNodeDetails(nodeItem: LaunchTreeItem): LaunchTreeItem[] {
    const items: LaunchTreeItem[] = [];

    // Add node metadata
    if (nodeItem.packageName) {
      items.push(LaunchTreeItem.createInfoItem('Package', nodeItem.packageName));
    }
    if (nodeItem.executable) {
      items.push(LaunchTreeItem.createInfoItem('Executable', nodeItem.executable));
    }
    if (nodeItem.namespace) {
      items.push(LaunchTreeItem.createInfoItem('Namespace', nodeItem.namespace));
    }

    // Add parameter group if parameters exist
    if (nodeItem.parameters && Object.keys(nodeItem.parameters).length > 0) {
      items.push(LaunchTreeItem.createParameterGroupItem(nodeItem.parameters));
    }

    // Add remapping group if remappings exist
    if (nodeItem.remappings && nodeItem.remappings.length > 0) {
      items.push(LaunchTreeItem.createRemapGroupItem(nodeItem.remappings));
    }

    // Add argument group if arguments exist
    if (nodeItem.arguments && nodeItem.arguments.length > 0) {
      items.push(LaunchTreeItem.createArgumentGroupItem(nodeItem.arguments));
    }

    return items;
  }

  /**
   * Get parameter items
   */
  private getParameters(paramGroupItem: LaunchTreeItem): LaunchTreeItem[] {
    if (!paramGroupItem.parameters) {
      return [];
    }

    return Object.entries(paramGroupItem.parameters).map(([key, value]) =>
      LaunchTreeItem.createParameterItem(key, value)
    );
  }

  /**
   * Get remapping items
   */
  private getRemappings(remapGroupItem: LaunchTreeItem): LaunchTreeItem[] {
    if (!remapGroupItem.remappings) {
      return [];
    }

    return remapGroupItem.remappings.map(([from, to]) =>
      LaunchTreeItem.createRemapItem(from, to)
    );
  }

  /**
   * Get argument items
   */
  private getArguments(argGroupItem: LaunchTreeItem): LaunchTreeItem[] {
    if (!argGroupItem.arguments) {
      return [];
    }

    return argGroupItem.arguments.map(arg =>
      LaunchTreeItem.createArgumentItem(arg)
    );
  }

  /**
   * Setup file watcher for auto-refresh
   */
  private setupFileWatcher(): void {
    // Watch for changes to .launch.py files
    this.fileWatcher = vscode.workspace.createFileSystemWatcher(
      '**/*.launch.py'
    );

    // Debounce refresh to avoid excessive updates
    let refreshTimeout: NodeJS.Timeout | undefined;
    const debouncedRefresh = () => {
      if (refreshTimeout) {
        clearTimeout(refreshTimeout);
      }
      refreshTimeout = setTimeout(() => this.refresh(), 500);
    };

    this.fileWatcher.onDidChange(debouncedRefresh);
    this.fileWatcher.onDidCreate(debouncedRefresh);
    this.fileWatcher.onDidDelete(debouncedRefresh);
  }

  /**
   * Create empty state item
   */
  private createEmptyStateItem(message: string): LaunchTreeItem {
    return LaunchTreeItem.createEmptyStateItem(message);
  }

  /**
   * Create error item
   */
  private createErrorItem(message: string): LaunchTreeItem {
    return LaunchTreeItem.createErrorItem(message);
  }

  /**
   * Dispose resources
   */
  dispose(): void {
    this.fileWatcher?.dispose();
    this._onDidChangeTreeData.dispose();
  }
}
```

### 2. LaunchTreeItem

```typescript
// src/ros/launch-tree/launch-tree-item.ts

import * as vscode from 'vscode';
import * as path from 'path';
import {
  LaunchTreeItemType,
  ILaunchTreeItem,
  IWorkspacePackage,
  IJsonProcess,
  IJsonLifecycleNode
} from './types';

export class LaunchTreeItem extends vscode.TreeItem implements ILaunchTreeItem {
  type: LaunchTreeItemType;
  children?: ILaunchTreeItem[];
  
  // Package properties
  packageName?: string;
  packagePath?: string;
  
  // Launch file properties
  launchFilePath?: string;
  
  // Node properties
  nodeName?: string;
  executable?: string;
  namespace?: string;
  lineNumber?: number;
  parameters?: Record<string, any>;
  remappings?: Array<[string, string]>;
  arguments?: string[] | any[];

  constructor(
    label: string,
    type: LaunchTreeItemType,
    collapsibleState?: vscode.TreeItemCollapsibleState
  ) {
    super(label, collapsibleState);
    this.type = type;
    this.contextValue = type;
  }

  /**
   * Create a package tree item
   */
  static createPackageItem(pkg: IWorkspacePackage): LaunchTreeItem {
    const item = new LaunchTreeItem(
      pkg.name,
      LaunchTreeItemType.Package,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.packageName = pkg.name;
    item.packagePath = pkg.path;
    item.resourceUri = vscode.Uri.file(pkg.path);
    item.iconPath = new vscode.ThemeIcon('package');
    item.description = `${pkg.launchFiles.length} launch file(s)`;
    item.tooltip = pkg.path;
    
    return item;
  }

  /**
   * Create a launch file tree item
   */
  static createLaunchFileItem(packageName: string, filePath: string): LaunchTreeItem {
    const fileName = path.basename(filePath);
    const item = new LaunchTreeItem(
      fileName,
      LaunchTreeItemType.LaunchFile,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.packageName = packageName;
    item.launchFilePath = filePath;
    item.resourceUri = vscode.Uri.file(filePath);
    item.iconPath = new vscode.ThemeIcon('rocket');
    item.tooltip = filePath;
    
    // Make it clickable to open file
    item.command = {
      command: 'vscode.open',
      title: 'Open Launch File',
      arguments: [vscode.Uri.file(filePath)]
    };
    
    return item;
  }

  /**
   * Create a node tree item
   */
  static createNodeItem(process: IJsonProcess, launchFilePath: string): LaunchTreeItem {
    const nodeName = process.node_name || process.executable || 'unknown';
    const item = new LaunchTreeItem(
      nodeName,
      LaunchTreeItemType.Node,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.nodeName = nodeName;
    item.packageName = process.package;
    item.executable = process.executable;
    item.namespace = process.namespace;
    item.parameters = process.parameters;
    item.remappings = process.remappings;
    item.arguments = process.arguments;
    item.launchFilePath = launchFilePath;
    item.iconPath = new vscode.ThemeIcon('symbol-method');
    item.description = process.package || '';
    item.tooltip = `Node: ${nodeName}\nPackage: ${process.package}\nExecutable: ${process.executable}`;
    
    return item;
  }

  /**
   * Create a lifecycle node tree item
   */
  static createLifecycleNodeItem(
    node: IJsonLifecycleNode,
    launchFilePath: string
  ): LaunchTreeItem {
    const item = new LaunchTreeItem(
      node.node_name,
      LaunchTreeItemType.LifecycleNode,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.nodeName = node.node_name;
    item.packageName = node.package;
    item.executable = node.executable;
    item.namespace = node.namespace;
    item.parameters = node.parameters;
    item.remappings = node.remappings;
    item.launchFilePath = launchFilePath;
    item.iconPath = new vscode.ThemeIcon('symbol-method', new vscode.ThemeColor('charts.blue'));
    item.description = `${node.package} (lifecycle)`;
    item.tooltip = `Lifecycle Node: ${node.node_name}\nPackage: ${node.package}\nExecutable: ${node.executable}`;
    
    return item;
  }

  /**
   * Create an include tree item
   */
  static createIncludeItem(includePath: string): LaunchTreeItem {
    const fileName = path.basename(includePath);
    const item = new LaunchTreeItem(
      fileName,
      LaunchTreeItemType.Include,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.resourceUri = vscode.Uri.file(includePath);
    item.iconPath = new vscode.ThemeIcon('link');
    item.description = 'included';
    item.tooltip = includePath;
    
    // Make it clickable to open included file
    item.command = {
      command: 'vscode.open',
      title: 'Open Included File',
      arguments: [vscode.Uri.file(includePath)]
    };
    
    return item;
  }

  /**
   * Create parameter group item
   */
  static createParameterGroupItem(parameters: Record<string, any>): LaunchTreeItem {
    const count = Object.keys(parameters).length;
    const item = new LaunchTreeItem(
      'Parameters',
      LaunchTreeItemType.ParameterGroup,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.parameters = parameters;
    item.iconPath = new vscode.ThemeIcon('symbol-parameter');
    item.description = `${count} parameter(s)`;
    
    return item;
  }

  /**
   * Create parameter item
   */
  static createParameterItem(name: string, value: any): LaunchTreeItem {
    const valueStr = JSON.stringify(value);
    const item = new LaunchTreeItem(
      `${name}: ${valueStr}`,
      LaunchTreeItemType.Parameter,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('symbol-variable');
    item.tooltip = `Parameter: ${name}\nValue: ${valueStr}\nType: ${typeof value}`;
    
    return item;
  }

  /**
   * Create remapping group item
   */
  static createRemapGroupItem(remappings: Array<[string, string]>): LaunchTreeItem {
    const item = new LaunchTreeItem(
      'Remappings',
      LaunchTreeItemType.RemapGroup,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.remappings = remappings;
    item.iconPath = new vscode.ThemeIcon('arrow-swap');
    item.description = `${remappings.length} remap(s)`;
    
    return item;
  }

  /**
   * Create remapping item
   */
  static createRemapItem(from: string, to: string): LaunchTreeItem {
    const item = new LaunchTreeItem(
      `${from} → ${to}`,
      LaunchTreeItemType.Remap,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('arrow-right');
    item.tooltip = `Remap: ${from} → ${to}`;
    
    return item;
  }

  /**
   * Create argument group item
   */
  static createArgumentGroupItem(args: string[] | any[]): LaunchTreeItem {
    const item = new LaunchTreeItem(
      'Arguments',
      LaunchTreeItemType.ArgumentGroup,
      vscode.TreeItemCollapsibleState.Collapsed
    );
    
    item.arguments = args;
    item.iconPath = new vscode.ThemeIcon('symbol-key');
    item.description = `${args.length} argument(s)`;
    
    return item;
  }

  /**
   * Create argument item
   */
  static createArgumentItem(arg: string | any): LaunchTreeItem {
    const label = typeof arg === 'string' ? arg : arg.name;
    const item = new LaunchTreeItem(
      label,
      LaunchTreeItemType.Argument,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('symbol-constant');
    
    if (typeof arg !== 'string') {
      item.tooltip = `Name: ${arg.name}\nDefault: ${arg.default}\nDescription: ${arg.description}`;
    }
    
    return item;
  }

  /**
   * Create info item (for node metadata)
   */
  static createInfoItem(label: string, value: string): LaunchTreeItem {
    const item = new LaunchTreeItem(
      `${label}: ${value}`,
      LaunchTreeItemType.Package, // Reuse type
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('info');
    
    return item;
  }

  /**
   * Create empty state item
   */
  static createEmptyStateItem(message: string): LaunchTreeItem {
    const item = new LaunchTreeItem(
      message,
      LaunchTreeItemType.Loading,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('info');
    
    return item;
  }

  /**
   * Create error item
   */
  static createErrorItem(message: string): LaunchTreeItem {
    const item = new LaunchTreeItem(
      message,
      LaunchTreeItemType.Error,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('error');
    
    return item;
  }

  /**
   * Create warning item
   */
  static createWarningItem(message: string): LaunchTreeItem {
    const item = new LaunchTreeItem(
      `⚠️ ${message}`,
      LaunchTreeItemType.Error,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('warning');
    
    return item;
  }
}
```

### 3. LaunchFileParser

```typescript
// src/ros/launch-tree/launch-parser.ts

import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as child_process from 'child_process';
import { promisify } from 'util';
import {
  ILaunchFileData,
  IWorkspacePackage,
  IJsonProcess,
  IJsonLifecycleNode,
  ILaunchArgument
} from './types';

const exec = promisify(child_process.exec);
const readdir = promisify(fs.readdir);
const stat = promisify(fs.stat);
const readFile = promisify(fs.readFile);

export class LaunchFileParser {
  private dumperScript: string;
  private cache = new Map<string, ILaunchFileData>();

  constructor(private outputChannel: vscode.OutputChannel) {
    // Path to ros2_launch_dumper.py
    this.dumperScript = path.join(
      __dirname,
      '../../../assets/scripts/ros2_launch_dumper.py'
    );
  }

  /**
   * Find all ROS packages with launch files in workspace
   */
  async findWorkspacePackages(workspacePaths: string[]): Promise<IWorkspacePackage[]> {
    const packages: IWorkspacePackage[] = [];

    for (const workspacePath of workspacePaths) {
      const pkgs = await this.findPackagesInPath(workspacePath);
      packages.push(...pkgs);
    }

    return packages;
  }

  /**
   * Find packages in a specific path
   */
  private async findPackagesInPath(searchPath: string): Promise<IWorkspacePackage[]> {
    const packages: IWorkspacePackage[] = [];

    try {
      // Find all package.xml files
      const packageXmlFiles = await this.findFiles(searchPath, 'package.xml');

      for (const packageXmlPath of packageXmlFiles) {
        const packageDir = path.dirname(packageXmlPath);
        const packageName = await this.getPackageNameFromXml(packageXmlPath);

        // Find launch files in package
        const launchFiles = await this.findLaunchFilesInPackage(packageDir);

        if (launchFiles.length > 0) {
          packages.push({
            name: packageName,
            path: packageDir,
            launchFiles
          });
        }
      }
    } catch (error) {
      this.outputChannel.appendLine(`Error finding packages: ${error}`);
    }

    return packages;
  }

  /**
   * Find all files matching pattern
   */
  private async findFiles(dir: string, pattern: string): Promise<string[]> {
    const results: string[] = [];
    
    const searchRecursive = async (currentDir: string) => {
      try {
        const entries = await readdir(currentDir);
        
        for (const entry of entries) {
          const fullPath = path.join(currentDir, entry);
          const stats = await stat(fullPath);
          
          if (stats.isDirectory()) {
            // Skip common non-package directories
            if (!['build', 'install', 'log', '.git', 'node_modules'].includes(entry)) {
              await searchRecursive(fullPath);
            }
          } else if (entry === pattern) {
            results.push(fullPath);
          }
        }
      } catch (error) {
        // Ignore permission errors
      }
    };
    
    await searchRecursive(dir);
    return results;
  }

  /**
   * Get package name from package.xml
   */
  private async getPackageNameFromXml(packageXmlPath: string): Promise<string> {
    try {
      const content = await readFile(packageXmlPath, 'utf8');
      const match = content.match(/<name>([^<]+)<\/name>/);
      return match ? match[1] : path.basename(path.dirname(packageXmlPath));
    } catch {
      return path.basename(path.dirname(packageXmlPath));
    }
  }

  /**
   * Find launch files in package directory
   */
  private async findLaunchFilesInPackage(packageDir: string): Promise<string[]> {
    const launchDir = path.join(packageDir, 'launch');
    const launchFiles: string[] = [];

    if (!fs.existsSync(launchDir)) {
      return launchFiles;
    }

    try {
      const files = await readdir(launchDir);
      
      for (const file of files) {
        if (file.endsWith('.launch.py')) {
          launchFiles.push(path.join(launchDir, file));
        }
      }
    } catch {
      // Directory doesn't exist or can't be read
    }

    return launchFiles;
  }

  /**
   * Parse a launch file using ros2_launch_dumper.py
   */
  async parseLaunchFile(filePath: string): Promise<ILaunchFileData> {
    // Check cache first
    const cached = this.cache.get(filePath);
    if (cached) {
      return cached;
    }

    try {
      // Execute ros2_launch_dumper.py
      const { stdout, stderr } = await exec(
        `python3 "${this.dumperScript}" "${filePath}" --json`,
        { timeout: 30000 } // 30 second timeout
      );

      if (stderr) {
        this.outputChannel.appendLine(`Warning from dumper: ${stderr}`);
      }

      // Parse JSON output
      const data: ILaunchFileData = JSON.parse(stdout);

      // Cache the result
      this.cache.set(filePath, data);

      return data;
    } catch (error) {
      this.outputChannel.appendLine(`Error parsing ${filePath}: ${error}`);
      
      // Return empty data with error
      return {
        processes: [],
        lifecycle_nodes: [],
        includes: [],
        arguments: [],
        warnings: [],
        errors: [String(error)]
      };
    }
  }

  /**
   * Clear cache for a file (called when file changes)
   */
  invalidateCache(filePath: string): void {
    this.cache.delete(filePath);
  }

  /**
   * Clear all cache
   */
  clearCache(): void {
    this.cache.clear();
  }
}
```

### 4. Command Handlers

```typescript
// src/ros/commands/launch-tree-commands.ts

import * as vscode from 'vscode';
import { LaunchTreeDataProvider } from '../launch-tree/launch-tree-provider';
import { LaunchTreeItem } from '../launch-tree/launch-tree-item';
import { LaunchTreeItemType } from '../launch-tree/types';

export function registerLaunchTreeCommands(
  context: vscode.ExtensionContext,
  treeDataProvider: LaunchTreeDataProvider
): void {
  // Refresh command
  context.subscriptions.push(
    vscode.commands.registerCommand('ROS2.launchTree.refresh', () => {
      treeDataProvider.refresh();
    })
  );

  // Reveal in tree command
  context.subscriptions.push(
    vscode.commands.registerCommand('ROS2.launchTree.reveal', async (uri: vscode.Uri) => {
      // TODO: Implement reveal logic
      vscode.window.showInformationMessage(`Reveal ${uri.fsPath} in tree`);
    })
  );

  // Find usages command
  context.subscriptions.push(
    vscode.commands.registerCommand('ROS2.launchTree.findUsages', async (item: LaunchTreeItem) => {
      if (item.type !== LaunchTreeItemType.LaunchFile || !item.launchFilePath) {
        return;
      }

      const fileName = path.basename(item.launchFilePath);
      
      // Search for IncludeLaunchDescription references
      const results = await vscode.workspace.findFiles('**/*.launch.py');
      // TODO: Parse files and find includes
      
      vscode.window.showInformationMessage(`Finding usages of ${fileName}...`);
    })
  );

  // Run launch file command
  context.subscriptions.push(
    vscode.commands.registerCommand('ROS2.launchTree.run', async (item: LaunchTreeItem) => {
      if (item.type !== LaunchTreeItemType.LaunchFile || !item.launchFilePath) {
        return;
      }

      // Delegate to existing roslaunch command
      await vscode.commands.executeCommand('ROS2.roslaunch', item.launchFilePath);
    })
  );

  // Debug launch file command
  context.subscriptions.push(
    vscode.commands.registerCommand('ROS2.launchTree.debug', async (item: LaunchTreeItem) => {
      if (item.type !== LaunchTreeItemType.LaunchFile || !item.launchFilePath) {
        return;
      }

      // Create debug configuration
      const config: vscode.DebugConfiguration = {
        type: 'ros2',
        name: `Debug ${path.basename(item.launchFilePath)}`,
        request: 'launch',
        target: item.launchFilePath
      };

      // Start debugging
      await vscode.debug.startDebugging(undefined, config);
    })
  );
}
```

### 5. Extension Registration

```typescript
// Modification to src/extension.ts

import { LaunchTreeDataProvider } from './ros/launch-tree/launch-tree-provider';
import { registerLaunchTreeCommands } from './ros/commands/launch-tree-commands';

export function activate(context: vscode.ExtensionContext) {
  // ... existing code ...

  // Register launch tree view
  const launchTreeProvider = new LaunchTreeDataProvider(context, outputChannel);
  const launchTreeView = vscode.window.createTreeView('ros2LaunchTree', {
    treeDataProvider: launchTreeProvider,
    showCollapseAll: true
  });

  context.subscriptions.push(launchTreeView);
  context.subscriptions.push(launchTreeProvider);

  // Register launch tree commands
  registerLaunchTreeCommands(context, launchTreeProvider);

  // ... existing code ...
}
```

## package.json Configuration

```json
{
  "contributes": {
    "views": {
      "explorer": [
        {
          "id": "ros2LaunchTree",
          "name": "ROS 2 Launch Files",
          "contextualTitle": "ROS 2 Launch Tree",
          "icon": "media/ros-icon.svg",
          "visibility": "visible"
        }
      ]
    },
    "commands": [
      {
        "command": "ROS2.launchTree.refresh",
        "title": "Refresh Launch Tree",
        "category": "ROS2",
        "icon": "$(refresh)"
      },
      {
        "command": "ROS2.launchTree.reveal",
        "title": "Reveal in Launch Tree",
        "category": "ROS2"
      },
      {
        "command": "ROS2.launchTree.findUsages",
        "title": "Find Launch File Usages",
        "category": "ROS2",
        "icon": "$(search)"
      },
      {
        "command": "ROS2.launchTree.run",
        "title": "Run Launch File",
        "category": "ROS2",
        "icon": "$(play)"
      },
      {
        "command": "ROS2.launchTree.debug",
        "title": "Debug Launch File",
        "category": "ROS2",
        "icon": "$(debug-alt)"
      }
    ],
    "menus": {
      "view/title": [
        {
          "command": "ROS2.launchTree.refresh",
          "when": "view == ros2LaunchTree",
          "group": "navigation"
        }
      ],
      "view/item/context": [
        {
          "command": "ROS2.launchTree.run",
          "when": "view == ros2LaunchTree && viewItem == launchFile",
          "group": "1_run@1"
        },
        {
          "command": "ROS2.launchTree.debug",
          "when": "view == ros2LaunchTree && viewItem == launchFile",
          "group": "1_run@2"
        },
        {
          "command": "ROS2.launchTree.findUsages",
          "when": "view == ros2LaunchTree && viewItem == launchFile",
          "group": "2_navigation@1"
        }
      ]
    }
  }
}
```

## Testing Strategy

See test files in file structure section above for test implementation examples.

## Performance Optimizations

1. **Lazy Loading**: Only parse launch files when tree node is expanded
2. **Caching**: Cache parsed results, invalidate on file changes
3. **Debouncing**: Debounce file watcher events to avoid excessive refreshes
4. **Background Parsing**: Use async/await for non-blocking operations
5. **Incremental Updates**: Only refresh affected tree nodes

## Error Handling

1. **Missing ROS Environment**: Show helpful error message with setup instructions
2. **Parse Errors**: Display error node in tree with details
3. **File Not Found**: Handle gracefully, show warning icon
4. **Timeout**: Set reasonable timeout for dumper script
5. **Invalid JSON**: Catch parse errors and show error state

---

**Author:** GitHub Copilot  
**Date:** 2026-01-31  
**Version:** 1.0
