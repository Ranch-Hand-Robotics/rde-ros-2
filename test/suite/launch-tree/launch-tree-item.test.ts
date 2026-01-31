// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from 'assert';
import * as vscode from 'vscode';
import { LaunchTreeItem } from '../../../src/ros/launch-tree/launch-tree-item';
import { LaunchTreeItemType } from '../../../src/ros/launch-tree/types';

describe('Launch Tree Tests', () => {
    
    it('LaunchTreeItem.createPackageItem creates package item with correct properties', () => {
        const pkg = {
            name: 'test_package',
            path: '/path/to/test_package',
            launchFiles: ['file1.launch.py', 'file2.launch.py']
        };
        
        const item = LaunchTreeItem.createPackageItem(pkg);
        
        assert.strictEqual(item.type, LaunchTreeItemType.Package);
        assert.strictEqual(item.packageName, 'test_package');
        assert.strictEqual(item.packagePath, '/path/to/test_package');
        assert.strictEqual(item.description, '2 launch file(s)');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.Collapsed);
    });
    
    it('LaunchTreeItem.createLaunchFileItem creates launch file item with correct properties', () => {
        const item = LaunchTreeItem.createLaunchFileItem('test_package', '/path/to/file.launch.py');
        
        assert.strictEqual(item.type, LaunchTreeItemType.LaunchFile);
        assert.strictEqual(item.packageName, 'test_package');
        assert.strictEqual(item.launchFilePath, '/path/to/file.launch.py');
        assert.strictEqual(item.label, 'file.launch.py');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.Collapsed);
        assert.ok(item.command); // Should have click command
        assert.strictEqual(item.command?.command, 'vscode.open');
    });
    
    it('LaunchTreeItem.createNodeItem creates node item with correct properties', () => {
        const process = {
            type: 'node',
            command: 'ros2 run test_pkg test_node',
            node_name: 'test_node',
            executable: 'test_node',
            package: 'test_pkg',
            namespace: '/test',
            parameters: { param1: 'value1' },
            remappings: [['old_topic', 'new_topic'] as [string, string]],
            arguments: ['--arg1', 'value1']
        };
        
        const item = LaunchTreeItem.createNodeItem(process, '/path/to/file.launch.py');
        
        assert.strictEqual(item.type, LaunchTreeItemType.Node);
        assert.strictEqual(item.nodeName, 'test_node');
        assert.strictEqual(item.packageName, 'test_pkg');
        assert.strictEqual(item.executable, 'test_node');
        assert.strictEqual(item.namespace, '/test');
        assert.strictEqual(item.launchFilePath, '/path/to/file.launch.py');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.Collapsed);
    });
    
    it('LaunchTreeItem.createLifecycleNodeItem creates lifecycle node with correct properties', () => {
        const node = {
            type: 'lifecycle' as const,
            node_name: 'lifecycle_test',
            namespace: '/lifecycle',
            package: 'lifecycle_pkg',
            executable: 'lifecycle_node',
            parameters: { managed: true },
            remappings: []
        };
        
        const item = LaunchTreeItem.createLifecycleNodeItem(node, '/path/to/file.launch.py');
        
        assert.strictEqual(item.type, LaunchTreeItemType.LifecycleNode);
        assert.strictEqual(item.nodeName, 'lifecycle_test');
        assert.strictEqual(item.packageName, 'lifecycle_pkg');
        assert.strictEqual(item.description, 'lifecycle_pkg (lifecycle)');
    });
    
    it('LaunchTreeItem.createParameterGroupItem creates parameter group with correct count', () => {
        const params = {
            param1: 'value1',
            param2: 42,
            param3: true
        };
        
        const item = LaunchTreeItem.createParameterGroupItem(params);
        
        assert.strictEqual(item.type, LaunchTreeItemType.ParameterGroup);
        assert.strictEqual(item.label, 'Parameters');
        assert.strictEqual(item.description, '3 parameter(s)');
        assert.strictEqual(item.parameters, params);
    });
    
    it('LaunchTreeItem.createParameterItem formats parameter value correctly', () => {
        const item = LaunchTreeItem.createParameterItem('test_param', 'test_value');
        
        assert.strictEqual(item.type, LaunchTreeItemType.Parameter);
        assert.strictEqual(item.label, 'test_param: "test_value"');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.None);
    });
    
    it('LaunchTreeItem.createRemapGroupItem creates remap group with correct count', () => {
        const remaps: Array<[string, string]> = [
            ['old_topic1', 'new_topic1'],
            ['old_topic2', 'new_topic2']
        ];
        
        const item = LaunchTreeItem.createRemapGroupItem(remaps);
        
        assert.strictEqual(item.type, LaunchTreeItemType.RemapGroup);
        assert.strictEqual(item.label, 'Remappings');
        assert.strictEqual(item.description, '2 remap(s)');
        assert.strictEqual(item.remappings, remaps);
    });
    
    it('LaunchTreeItem.createRemapItem formats remap correctly', () => {
        const item = LaunchTreeItem.createRemapItem('/old_topic', '/new_topic');
        
        assert.strictEqual(item.type, LaunchTreeItemType.Remap);
        assert.strictEqual(item.label, '/old_topic → /new_topic');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.None);
    });
    
    it('LaunchTreeItem.createIncludeItem creates include item with click command', () => {
        const item = LaunchTreeItem.createIncludeItem('/path/to/included.launch.py');
        
        assert.strictEqual(item.type, LaunchTreeItemType.Include);
        assert.strictEqual(item.label, 'included.launch.py');
        assert.strictEqual(item.description, 'included');
        assert.ok(item.command);
        assert.strictEqual(item.command?.command, 'vscode.open');
    });
    
    it('LaunchTreeItem.createEmptyStateItem creates info item', () => {
        const item = LaunchTreeItem.createEmptyStateItem('No items found');
        
        assert.strictEqual(item.type, LaunchTreeItemType.Loading);
        assert.strictEqual(item.label, 'No items found');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.None);
    });
    
    it('LaunchTreeItem.createErrorItem creates error item', () => {
        const item = LaunchTreeItem.createErrorItem('Parse error occurred');
        
        assert.strictEqual(item.type, LaunchTreeItemType.Error);
        assert.strictEqual(item.label, 'Parse error occurred');
        assert.strictEqual(item.collapsibleState, vscode.TreeItemCollapsibleState.None);
    });
});
