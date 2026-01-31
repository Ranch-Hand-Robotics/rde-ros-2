# Launch Tree Module

This module provides a tree view visualization for ROS 2 launch files in VS Code.

## Overview

The launch tree module displays a hierarchical view of ROS 2 packages, launch files, and nodes in the VS Code Explorer sidebar. Users can navigate launch files, view node configurations, and execute launch commands directly from the tree.

## Architecture

```
LaunchTreeDataProvider (implements vscode.TreeDataProvider)
  │
  ├─ getChildren() - Returns tree items for a given parent
  ├─ getTreeItem() - Converts our items to VS Code tree items
  ├─ refresh() - Clears cache and refreshes tree
  └─ File watcher - Auto-refresh on .launch.py changes
      │
      └─ LaunchFileParser
            │
            ├─ findWorkspacePackages() - Scans workspace
            ├─ parseLaunchFile() - Parses a single file
            └─ Cache - Map<filePath, ILaunchFileData>
                  │
                  └─ ros2_launch_dumper.py
                        └─ Returns JSON with nodes, includes, params
```

## File Descriptions

### types.ts
Defines all TypeScript interfaces and enums:
- `LaunchTreeItemType` - Enum for item types (Package, LaunchFile, Node, etc.)
- `ILaunchTreeItem` - Base interface for tree items
- `ILaunchFileData` - Data returned from ros2_launch_dumper.py
- `IJsonProcess` / `IJsonLifecycleNode` - Node definitions
- `IWorkspacePackage` - Workspace package structure

### launch-tree-item.ts
Contains the `LaunchTreeItem` class with static factory methods:
- `createPackageItem()` - Creates package tree item
- `createLaunchFileItem()` - Creates launch file item (with click-to-open)
- `createNodeItem()` - Creates regular node item
- `createLifecycleNodeItem()` - Creates lifecycle node item
- `createParameterGroupItem()` - Creates parameter group
- `createParameterItem()` - Creates individual parameter
- `createRemapGroupItem()` - Creates remapping group
- `createRemapItem()` - Creates individual remapping
- `createIncludeItem()` - Creates include item (with click-to-open)
- `createArgumentGroupItem()` - Creates argument group
- `createArgumentItem()` - Creates individual argument
- `createInfoItem()` - Creates metadata info item
- `createEmptyStateItem()` - Creates "no items" message
- `createErrorItem()` - Creates error message
- `createWarningItem()` - Creates warning message

### launch-parser.ts
The `LaunchFileParser` class handles file system operations and parsing:
- `findWorkspacePackages()` - Scans workspace folders for ROS packages
- `findPackagesInPath()` - Finds packages in a directory
- `findFiles()` - Recursively searches for files (e.g., package.xml)
- `findLaunchFilesInPackage()` - Finds .launch.py files in a package
- `parseLaunchFile()` - Executes ros2_launch_dumper.py and parses JSON
- `invalidateCache()` - Clears cache for a specific file
- `clearCache()` - Clears entire cache

### launch-tree-provider.ts
The `LaunchTreeDataProvider` class manages the tree view:
- `refresh()` - Refreshes the entire tree
- `getTreeItem()` - Returns VS Code TreeItem for display
- `getChildren()` - Returns child items for a given element
- `getRootItems()` - Returns workspace packages
- `getPackageLaunchFiles()` - Returns launch files for a package
- `getLaunchFileContents()` - Returns nodes/includes for a launch file
- `getNodeDetails()` - Returns parameters/remaps for a node
- `getParameters()` - Returns parameter items
- `getRemappings()` - Returns remapping items
- `getArguments()` - Returns argument items
- `setupFileWatcher()` - Watches for .launch.py file changes
- `dispose()` - Cleanup resources

## Data Flow

### Initial Load
```
1. VS Code activates extension
2. LaunchTreeDataProvider registered
3. Tree view appears in Explorer
4. User expands tree
5. getRootItems() called
   ├─ findWorkspacePackages()
   ├─ Scans for package.xml files
   ├─ Finds .launch.py files in each package
   └─ Returns package tree items
```

### Expanding a Launch File
```
1. User clicks to expand launch file
2. getChildren(launchFileItem) called
3. getLaunchFileContents()
   ├─ parseLaunchFile()
   ├─ Execute: python3 ros2_launch_dumper.py file.launch.py --json
   ├─ Parse JSON output
   ├─ Cache result
   └─ Create tree items for nodes, includes, arguments
4. Tree displays nodes and includes
```

### File Change Detection
```
1. User modifies .launch.py file
2. FileSystemWatcher detects change
3. Debounce timer (500ms)
4. refresh() called
5. Cache invalidated
6. Tree re-rendered
```

## Integration Points

### extension.ts
```typescript
// Registration in activate()
launchTreeProvider = new LaunchTreeDataProvider(context, outputChannel, extPath);
const launchTreeView = vscode.window.createTreeView('ros2LaunchTree', {
    treeDataProvider: launchTreeProvider,
    showCollapseAll: true
});
context.subscriptions.push(launchTreeView);
context.subscriptions.push(launchTreeProvider);

// Command handlers
vscode.commands.registerCommand(Commands.LaunchTreeRefresh, ...);
vscode.commands.registerCommand(Commands.LaunchTreeRun, ...);
vscode.commands.registerCommand(Commands.LaunchTreeDebug, ...);
```

### package.json
```json
{
  "views": {
    "explorer": [
      {
        "id": "ros2LaunchTree",
        "name": "ROS 2 Launch Files"
      }
    ]
  },
  "commands": [...],
  "menus": {
    "view/title": [...],
    "view/item/context": [...]
  }
}
```

### ros2_launch_dumper.py
The Python script that parses launch files:
- Located: `assets/scripts/ros2_launch_dumper.py`
- Input: Launch file path, `--json` flag
- Output: JSON with structure:
  ```json
  {
    "processes": [...],
    "lifecycle_nodes": [...],
    "includes": [...],
    "arguments": [...],
    "warnings": [...],
    "errors": [...]
  }
  ```

## Performance Considerations

### Lazy Loading
- Packages shown immediately (workspace scan)
- Launch files parsed only when expanded
- Node details shown only when expanded

### Caching
- Parse results cached in `Map<filePath, ILaunchFileData>`
- Cache invalidated on file change
- Cleared on manual refresh

### Debouncing
- File watcher events debounced at 500ms
- Prevents excessive refreshes during rapid editing

### Background Processing
- All parsing is async
- UI doesn't block during parse operations
- Timeout set at 30 seconds for dumper script

## Error Handling

### Scenarios
1. **No workspace opened** → "No workspace opened" message
2. **No launch files found** → "No launch files found" message
3. **Parse error** → Error item with message in tree
4. **Missing ROS environment** → Graceful degradation, shows errors
5. **File not found** → Handled in parser, returns empty data

### Error Display
```
▼ 🚀 problematic.launch.py
  └─ ❌ Parse error: SyntaxError at line 15
```

## Testing

### Unit Tests
- `test/suite/launch-tree/launch-tree-item.test.ts`
- Tests all static factory methods
- Tests item properties and types
- 14 test cases covering common scenarios

### Test Fixtures
- `test/suite/launch-tree/fixtures/simple.launch.py`
- Basic launch file for integration testing

### Manual Testing
1. Open ROS 2 workspace with launch files
2. Verify tree appears in Explorer
3. Expand packages, launch files, nodes
4. Click items to navigate
5. Right-click for context menu
6. Modify launch file, verify auto-refresh

## Future Enhancements

### Phase 2 (Planned)
- Complete "Reveal in Launch Tree" functionality
- Implement "Find Usages" search
- Enhanced error messaging
- Better loading states

### Phase 3 (Planned)
- Search/filter within tree
- Inline parameter editing (modify values in tree)
- Dependency graph visualization
- Launch file validation (check for errors)
- Support for XML launch files (ROS 1 legacy)

## Troubleshooting

### Tree doesn't appear
- Check workspace has `.launch.py` files
- Verify "ROS 2 LAUNCH FILES" section in Explorer
- Try manual refresh command

### Parse errors
- Ensure ROS 2 environment is sourced
- Check `ros2_launch_dumper.py` is executable
- Verify Python 3 is available
- Check extension output channel for details

### Performance issues
- Check cache is working (file changes trigger refresh)
- Verify file watcher is active
- Look for errors in output channel

## Contributing

When modifying this module:
1. Follow existing code patterns
2. Add unit tests for new features
3. Update this README if adding new files
4. Test with real ROS 2 workspaces
5. Check performance with large workspaces (100+ files)

## Related Files

- `src/extension.ts` - Extension activation and command registration
- `package.json` - View and command configuration
- `assets/scripts/ros2_launch_dumper.py` - Python parser
- `docs/IMPLEMENTATION_SCREENSHOT.md` - Visual mockup and features
- Original proposal documents in `/docs/` directory

---

**Author:** GitHub Copilot  
**Date:** 2026-01-31  
**Status:** Phase 1 MVP Complete
