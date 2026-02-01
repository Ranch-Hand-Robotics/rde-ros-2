// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

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
  Loading = 'loading',
  Info = 'info'
}

/**
 * Base interface for all tree items
 */
export interface ILaunchTreeItem {
  type: LaunchTreeItemType;
  label?: string | vscode.TreeItemLabel;
  description?: string | boolean;
  tooltip?: string | vscode.MarkdownString;
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
  packageName?: string;
  executable?: string;
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
 * Workspace package structure
 */
export interface IWorkspacePackage {
  name: string;
  path: string;
  launchFiles: string[];
}
