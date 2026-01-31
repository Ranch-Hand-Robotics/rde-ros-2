// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

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
    private outputChannel: vscode.OutputChannel,
    private extPath: string
  ) {
    this.parser = new LaunchFileParser(outputChannel, extPath);
    this.setupFileWatcher();
  }

  /**
   * Refresh the tree view
   */
  refresh(): void {
    this.parser.clearCache();
    this.workspacePackages = [];
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

      // If no content was found, show a message
      if (items.length === 0) {
        items.push(this.createEmptyStateItem('No nodes or includes found'));
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

    if (items.length === 0) {
      items.push(this.createEmptyStateItem('No additional details'));
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
