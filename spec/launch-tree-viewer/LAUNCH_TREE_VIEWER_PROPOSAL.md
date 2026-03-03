# Launch File Tree Viewer - Design Proposal

## Overview

This proposal outlines the design and implementation of a **Launch File Tree Viewer** for the ROS 2 VS Code extension. This feature will provide developers with a hierarchical visualization of ROS 2 launch files, similar to `rqt_launchtree`, integrated directly into the VS Code explorer sidebar.

## Problem Statement

**Current Challenge:**
> "Launch files melt my brain" - Issue #318

ROS 2 launch files can be complex, with multiple levels of includes, numerous nodes, and intricate parameter configurations. Developers need:
- Visual overview of launch file structure
- Quick navigation to launch files and node definitions
- Understanding of launch file dependencies (includes)
- Ability to find all usages of a launch file
- Insight into variables and arguments within launch files

**Reference Implementation:**
The [rqt_launchtree](https://github.com/pschillinger/rqt_launchtree) tool provides this visualization in RQt, showing:
- Hierarchical tree of packages, launch files, and nodes
- Node properties (parameters, arguments, remaps)
- Launch file includes
- Expandable/collapsible structure

![rqt_launchtree example](https://github.com/user-attachments/assets/bd3cab49-73ab-4fd3-a537-5aebfb5e6d37)

## Proposed Solution

### 1. Tree View Integration

**VS Code TreeView API:**
Create a custom `TreeDataProvider` that integrates with VS Code's native tree view system. This will appear in the Explorer sidebar and can be toggled via View menu or Command Palette.

**Tree Structure:**
```
📦 ROS 2 Workspace
  └─ 📦 Package: map_server
      ├─ 🚀 map_server.launch.py
      │   ├─ 📍 Node: map_server_node
      │   │   ├─ 📄 Package: nav2_map_server
      │   │   ├─ 🔧 Executable: map_server
      │   │   ├─ 📊 Parameters
      │   │   │   ├─ yaml_filename: /path/to/map.yaml
      │   │   │   └─ use_sim_time: true
      │   │   └─ 🔀 Remaps
      │   │       └─ /tf → /map_tf
      │   ├─ 📍 Node: lifecycle_manager
      │   └─ 🔗 Include: common_params.launch.py
      └─ 🚀 scenario_setup.launch.py
```

**Tree Hierarchy Levels:**
1. **Workspace Root** - Shows all ROS 2 packages with launch files
2. **Package Level** - Groups launch files by package
3. **Launch File Level** - Individual `.launch.py` files
4. **Node/Action Level** - Nodes, includes, and other launch actions
5. **Details Level** - Parameters, arguments, remaps, and node properties

### 2. View Container Configuration

**package.json Contribution:**
```json
{
  "contributes": {
    "viewsContainers": {
      "activitybar": [
        {
          "id": "ros2-explorer",
          "title": "ROS 2",
          "icon": "media/ros-icon.svg"
        }
      ]
    },
    "views": {
      "ros2-explorer": [
        {
          "id": "ros2LaunchTree",
          "name": "Launch Files",
          "contextualTitle": "ROS 2 Launch Tree"
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
        "category": "ROS2"
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
          "command": "ROS2.launchTree.findUsages",
          "when": "view == ros2LaunchTree && viewItem == launchFile",
          "group": "inline"
        }
      ]
    }
  }
}
```

**Alternative: File Explorer Integration**
Instead of a separate Activity Bar icon, the tree view can be added to the existing File Explorer container:
```json
{
  "views": {
    "explorer": [
      {
        "id": "ros2LaunchTree",
        "name": "ROS 2 Launch Files"
      }
    ]
  }
}
```

### 3. Data Model

**Tree Item Types:**
```typescript
enum LaunchTreeItemType {
  Workspace = 'workspace',
  Package = 'package',
  LaunchFile = 'launchFile',
  Node = 'node',
  LifecycleNode = 'lifecycleNode',
  Include = 'include',
  Parameter = 'parameter',
  Argument = 'argument',
  Remap = 'remap',
  ExecuteProcess = 'executeProcess'
}

interface LaunchTreeItem extends vscode.TreeItem {
  type: LaunchTreeItemType;
  packageName?: string;
  launchFilePath?: string;
  nodeName?: string;
  resourceUri?: vscode.Uri;
  children?: LaunchTreeItem[];
}
```

**Data Source:**
Leverage existing `ros2_launch_dumper.py` script:
- **Input:** Launch file path
- **Output:** JSON with nodes, includes, and actions
- **Interfaces:** Reuse `IJsonProcess`, `IJsonLifecycleNode` from `src/debugger/configuration/resolvers/ros2/launch.ts`

**Workspace Discovery:**
```typescript
// Find all packages with launch files
const packages = await findRosPackagesInWorkspace();

// For each package, find launch files
const launchFiles = await glob('**/*.launch.py', { cwd: packagePath });

// Parse each launch file using ros2_launch_dumper.py
const launchData = await parseLaunchFile(launchFilePath);
```

### 4. Click-to-Navigate Behavior

**Navigation Actions by Item Type:**

| Item Type | Click Behavior | Additional Actions |
|-----------|----------------|-------------------|
| **Package** | Open package directory | Context menu: "Open package.xml" |
| **Launch File** | Open launch file in editor | Context menu: "Run", "Debug", "Find Usages" |
| **Node** | Jump to node instantiation in launch file | Context menu: "Go to Definition" (node source) |
| **Include** | Open included launch file | Context menu: "Reveal in Tree" |
| **Parameter** | Jump to parameter definition | Context menu: "Edit Value" |
| **Argument** | Jump to argument declaration | - |

**Implementation:**
```typescript
class LaunchTreeDataProvider implements vscode.TreeDataProvider<LaunchTreeItem> {
  async onTreeItemClick(item: LaunchTreeItem): Promise<void> {
    switch (item.type) {
      case LaunchTreeItemType.LaunchFile:
        const doc = await vscode.workspace.openTextDocument(item.resourceUri);
        await vscode.window.showTextDocument(doc);
        break;
      
      case LaunchTreeItemType.Node:
        // Jump to line in launch file where node is defined
        await this.revealInLaunchFile(item.launchFilePath, item.lineNumber);
        break;
      
      case LaunchTreeItemType.Package:
        // Open package directory in explorer
        await vscode.commands.executeCommand('revealInExplorer', item.resourceUri);
        break;
    }
  }
}
```

### 5. Commands and Interactions

**Primary Commands:**

1. **`ROS2.launchTree.refresh`** - Refresh tree view
   - Triggered manually via button or command palette
   - Auto-refresh on file changes (via FileSystemWatcher)
   
2. **`ROS2.launchTree.reveal`** - Reveal active file in tree
   - When a launch file is opened, highlight it in the tree
   - Similar to "Reveal in Explorer" in VS Code
   
3. **`ROS2.launchTree.findUsages`** - Find all usages of a launch file
   - Search workspace for `IncludeLaunchDescription` pointing to selected file
   - Show results in tree or search panel

4. **`ROS2.launchTree.runLaunch`** - Run selected launch file
   - Context menu on launch file items
   - Calls existing `ROS2.roslaunch` command

5. **`ROS2.launchTree.debugLaunch`** - Debug selected launch file
   - Context menu on launch file items
   - Creates debug configuration and starts debugging

**Context Menus:**
```json
{
  "menus": {
    "view/item/context": [
      {
        "command": "ROS2.roslaunch",
        "when": "view == ros2LaunchTree && viewItem == launchFile",
        "group": "1_run"
      },
      {
        "command": "ROS2.launchTree.findUsages",
        "when": "view == ros2LaunchTree && viewItem == launchFile",
        "group": "2_navigation"
      }
    ]
  }
}
```

### 6. Advanced Features (Future Enhancements)

**Phase 2 Features:**
- **Variable Tracking**: Show where launch arguments are used throughout the file
- **Dependency Graph**: Visualize include relationships between launch files
- **Search/Filter**: Filter tree by package, node name, or parameter
- **Inline Editing**: Edit parameter values directly in tree (write back to file)
- **Launch File Validation**: Show errors/warnings in tree (missing files, invalid params)
- **Multi-workspace Support**: Handle multiple ROS workspaces

**Integration with Existing Features:**
- Link to **Test Provider**: Show test launch files in separate section
- Link to **Lifecycle Manager**: Highlight lifecycle nodes with state indicators
- Link to **Debugger**: Quick-launch debugging from tree

## Implementation Plan

### Phase 1: Core Tree View (MVP)
**Files to Create:**
```
src/ros/launch-tree-provider.ts    # TreeDataProvider implementation
src/ros/launch-tree-item.ts        # Tree item classes and types
src/ros/launch-parser.ts            # Wrapper for ros2_launch_dumper.py
test/suite/launch-tree.test.ts     # Unit tests
```

**Files to Modify:**
```
src/extension.ts                    # Register tree view and commands
package.json                        # Add views, commands, menus
```

**Steps:**
1. Define tree item interfaces and types
2. Create `LaunchTreeDataProvider` skeleton
3. Implement workspace scanning for packages and launch files
4. Integrate `ros2_launch_dumper.py` for parsing
5. Build tree structure from parsed data
6. Register view in `extension.ts` and `package.json`
7. Add refresh command
8. Implement basic click-to-navigate

**Estimated Effort:** 2-3 days

### Phase 2: Enhanced Navigation
**Steps:**
1. Implement "Reveal in Launch Tree" command
2. Add context menus for launch files (Run, Debug)
3. Implement "Find Usages" command
4. Add FileSystemWatcher for auto-refresh
5. Improve error handling and loading states

**Estimated Effort:** 1-2 days

### Phase 3: Advanced Features
**Steps:**
1. Add parameter/argument detail levels
2. Implement inline editing of parameters
3. Add search/filter capability
4. Show include relationships
5. Add icons and syntax highlighting

**Estimated Effort:** 2-3 days

## Technical Considerations

### Performance
- **Lazy Loading**: Only parse launch files when expanded in tree
- **Caching**: Cache parsed results, invalidate on file changes
- **Debouncing**: Debounce refresh on rapid file changes
- **Background Processing**: Parse launch files in background to avoid UI blocking

### Error Handling
- **Missing Python Dependencies**: Gracefully handle missing ROS 2 environment
- **Invalid Launch Files**: Show error nodes in tree with diagnostic info
- **Permission Issues**: Handle files that can't be read
- **Timeout**: Set timeout for `ros2_launch_dumper.py` execution

### Compatibility
- **ROS 2 Versions**: Test with Humble, Iron, Rolling
- **Platform Support**: Windows, Linux, macOS
- **Launch File Formats**: Python launch files (`.launch.py`)
- **XML Launch Files**: Phase 2 support for ROS 1-style XML (if needed)

### Testing Strategy
- **Unit Tests**: Test tree provider logic, parsing, and data structures
- **Integration Tests**: Test with sample launch files from `test/launch/`
- **Manual Testing**: Test with real ROS 2 packages (nav2, MoveIt)
- **Performance Tests**: Measure parsing time with large workspaces

## User Experience

### Activation
- **Auto-activate**: When workspace contains `.launch.py` files
- **Manual toggle**: Via Command Palette → "ROS2: Show Launch Tree"
- **View menu**: Check/uncheck "ROS 2 Launch Files" in View → Appearance

### Empty State
When no launch files found:
```
📦 ROS 2 Launch Files
  └─ ℹ️ No launch files found in workspace
      Click to learn how to create launch files
```

### Loading State
```
📦 ROS 2 Launch Files
  └─ ⏳ Scanning workspace for launch files...
```

### Error State
```
📦 ROS 2 Launch Files
  └─ ⚠️ ROS 2 environment not found
      Click to configure ROS 2 setup
```

## Success Criteria

**MVP Requirements:**
- ✅ Tree view appears in Explorer sidebar
- ✅ Shows packages → launch files → nodes hierarchy
- ✅ Clicking launch file opens it in editor
- ✅ Clicking node jumps to definition in launch file
- ✅ Refresh command updates tree
- ✅ Handles errors gracefully (missing ROS 2, invalid files)

**Quality Requirements:**
- ✅ Parses 100+ launch files in < 5 seconds
- ✅ No crashes on malformed launch files
- ✅ Works on Windows, Linux, macOS
- ✅ Follows VS Code UX patterns
- ✅ Includes unit tests with >80% coverage

## References

- [VS Code TreeView API Documentation](https://code.visualstudio.com/api/extension-guides/tree-view)
- [rqt_launchtree GitHub](https://github.com/pschillinger/rqt_launchtree)
- [ROS 2 Launch Documentation](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-Main.html)
- Existing Code: `src/debugger/configuration/resolvers/ros2/launch.ts`
- Launch Parser: `assets/scripts/ros2_launch_dumper.py`

## Open Questions

1. **Where should the tree view live?**
   - Option A: Separate ROS 2 activity bar icon (more discoverable)
   - Option B: File Explorer sidebar section (less clutter)
   - **Recommendation:** Start with File Explorer, add activity bar in Phase 2

2. **Should we support XML launch files?**
   - ROS 2 primarily uses Python launch files
   - XML is legacy ROS 1 format
   - **Recommendation:** Python only for MVP, consider XML in Phase 3

3. **How deep should the tree go?**
   - Show all parameters/args by default?
   - Collapse by default and expand on demand?
   - **Recommendation:** Auto-expand to Node level, collapse parameters

4. **Should we cache parsed data?**
   - Persist to disk or memory only?
   - Invalidation strategy?
   - **Recommendation:** Memory cache with file watcher invalidation

## Conclusion

The Launch File Tree Viewer will significantly improve the developer experience when working with ROS 2 launch files in VS Code. By providing a visual, navigable hierarchy of launch configurations, it will reduce cognitive load and improve productivity.

The phased approach allows for rapid delivery of core functionality while leaving room for advanced features based on user feedback.

**Next Steps:**
1. Review and approve this proposal
2. Create detailed implementation tickets
3. Begin Phase 1 development
4. Gather early user feedback
5. Iterate and enhance

---

**Author:** GitHub Copilot  
**Date:** 2026-01-31  
**Status:** Proposal - Awaiting Review
