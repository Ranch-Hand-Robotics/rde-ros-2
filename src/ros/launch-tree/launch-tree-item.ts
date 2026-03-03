// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

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
      LaunchTreeItemType.Info,
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
