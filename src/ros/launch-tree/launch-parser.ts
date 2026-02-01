// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as child_process from 'child_process';
import { promisify } from 'util';
import * as extension from '../../extension';
import {
  ILaunchFileData,
  IWorkspacePackage,
  IJsonProcess,
  IJsonLifecycleNode,
  ILaunchArgument
} from './types';

const execAsync = promisify(child_process.exec);
const readdir = promisify(fs.readdir);
const stat = promisify(fs.stat);
const readFile = promisify(fs.readFile);

export class LaunchFileParser {
  private dumperScript: string;
  private cache = new Map<string, ILaunchFileData>();

  constructor(private outputChannel: vscode.OutputChannel, extPath: string) {
    // Path to ros2_launch_dumper.py
    this.dumperScript = path.join(
      extPath,
      'assets/scripts/ros2_launch_dumper.py'
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
          
          try {
            const stats = await stat(fullPath);
            
            if (stats.isDirectory()) {
              // Skip common non-package directories
              if (!['build', 'install', 'log', '.git', 'node_modules', 'dist', 'out'].includes(entry)) {
                await searchRecursive(fullPath);
              }
            } else if (entry === pattern) {
              results.push(fullPath);
            }
          } catch (error) {
            // Ignore permission errors on individual files
          }
        }
      } catch (error) {
        // Ignore permission errors on directories
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
   * Supports both Python (.launch.py) and XML (.launch.xml) files
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
        // Support both Python and XML launch files
        if (file.endsWith('.launch.py') || file.endsWith('.launch.xml')) {
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
   * Supports both Python (.launch.py) and XML (.launch.xml) launch files
   */
  async parseLaunchFile(filePath: string): Promise<ILaunchFileData> {
    // Check cache first
    const cached = this.cache.get(filePath);
    if (cached) {
      return cached;
    }

    try {
      // Get ROS environment from extension
      const rosEnv = await extension.resolvedEnv();
      
      // Execute ros2_launch_dumper.py with proper ROS environment
      // Note: The dumper script automatically outputs JSON when USE_JSON_OUTPUT=True
      const { stdout, stderr } = await execAsync(
        `python3 "${this.dumperScript}" "${filePath}"`,
        { 
          timeout: 30000, // 30 second timeout
          maxBuffer: 1024 * 1024 * 10, // 10MB buffer
          env: rosEnv // Use ROS environment from extension
        }
      );

      if (stderr) {
        // Stderr is not necessarily an error - log for debugging
        this.outputChannel.appendLine(`Launch dumper stderr: ${stderr}`);
      }

      if (!stdout || stdout.length === 0) {
        throw new Error('Launch dumper produced no output');
      }

      // Parse JSON output
      let data: any;
      try {
        data = JSON.parse(stdout);
      } catch (parseError) {
        this.outputChannel.appendLine(`Failed to parse JSON from dumper: ${parseError}`);
        this.outputChannel.appendLine(`Output was: ${stdout.substring(0, 500)}`);
        throw new Error(`Failed to parse launch file: ${parseError}`);
      }

      // Transform to our interface (matching the structure from launch.ts)
      const result: ILaunchFileData = {
        processes: data.processes || [],
        lifecycle_nodes: data.lifecycle_nodes || [],
        includes: data.includes || [],
        arguments: data.arguments || [],
        warnings: data.warnings || [],
        errors: data.errors || []
      };

      // Cache the result
      this.cache.set(filePath, result);

      return result;
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
