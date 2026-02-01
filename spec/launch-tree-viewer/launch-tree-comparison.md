# Launch File Tree Viewer - Feature Comparison

This document compares the proposed VS Code Launch File Tree Viewer with the existing `rqt_launchtree` tool and outlines the advantages of native VS Code integration.

## Feature Comparison Matrix

| Feature | rqt_launchtree | VS Code Launch Tree | Notes |
|---------|----------------|---------------------|-------|
| **Core Features** |
| Hierarchical launch file view | ✅ | ✅ | Both show tree structure |
| Package grouping | ✅ | ✅ | Both organize by package |
| Node listing | ✅ | ✅ | Both show nodes in launch files |
| Parameter display | ✅ | ✅ | Both show node parameters |
| Argument display | ✅ | ✅ | Both show launch arguments |
| Remapping display | ✅ | ✅ | Both show topic remaps |
| Include/nested launches | ✅ | ✅ | Both show included launch files |
| **Navigation** |
| Click to open file | ❌ | ✅ | VS Code integrates with editor |
| Jump to definition | ❌ | ✅ | VS Code can jump to exact line |
| Find usages | ❌ | ✅ | VS Code can search workspace |
| Reveal in explorer | ❌ | ✅ | VS Code native integration |
| Go to node source | ❌ | ✅ (planned) | Jump to node's source code |
| **Filtering/Search** |
| Filter by type (nodes/params) | ✅ | 🟡 (Phase 2) | Both support filtering |
| Search by name | ❌ | 🟡 (Phase 2) | VS Code has better search |
| Filter by package | ❌ | 🟡 (Phase 2) | Can be implemented |
| **Execution** |
| Run launch file | ❌ | ✅ | Direct integration with roslaunch |
| Debug launch file | ❌ | ✅ | Native debugger integration |
| Run individual node | ❌ | 🟡 (possible) | Could add in future |
| **Integration** |
| Standalone application | ✅ | ❌ | RQt is separate GUI |
| IDE integration | ❌ | ✅ | VS Code is the IDE |
| Live updates | ✅ | ✅ | Both can refresh |
| File watcher | ❌ | ✅ | VS Code watches file changes |
| Version control integration | ❌ | ✅ | VS Code shows git status |
| **User Experience** |
| Dockable panels | ✅ | ✅ | Both support docking |
| Multiple workspaces | ❌ | ✅ | VS Code multi-root support |
| Custom themes | 🟡 | ✅ | VS Code respects user theme |
| Keyboard shortcuts | 🟡 | ✅ | VS Code has extensive shortcuts |
| Context menus | ✅ | ✅ | Both have right-click actions |
| **Platform Support** |
| Linux | ✅ | ✅ | Both support Linux |
| Windows | ✅ | ✅ | Both support Windows |
| macOS | ✅ | ✅ | Both support macOS |
| Remote development | ❌ | ✅ | VS Code supports SSH/containers |
| **Launch File Formats** |
| Python launch files | ✅ | ✅ | Both support .launch.py |
| XML launch files | ✅ | 🟡 (Phase 3) | RQt has legacy support |
| YAML launch files | 🟡 | 🟡 | Neither has full support |

**Legend:**
- ✅ = Fully supported
- 🟡 = Partially supported / Planned
- ❌ = Not supported

## Key Advantages of VS Code Integration

### 1. **Seamless Editor Integration**
**rqt_launchtree:** Separate application, no integration with code editor
**VS Code:** 
- Click launch file → Opens in same window
- Click node → Jumps to exact line in launch file
- Click parameter → Navigates to parameter definition
- Inline code editing with full IDE features (autocomplete, syntax highlighting)

### 2. **Debugging Workflow**
**rqt_launchtree:** Cannot debug from UI, must manually configure
**VS Code:**
- Right-click launch file → "Debug Launch File"
- Automatic debug configuration generation
- Integrated breakpoint management
- Live variable inspection during debug

### 3. **Code Intelligence**
**rqt_launchtree:** Static view only
**VS Code:**
- IntelliSense for parameter names/values
- "Find All References" for launch arguments
- "Go to Definition" for included files
- Rename refactoring across files
- Real-time error checking

### 4. **Workspace Management**
**rqt_launchtree:** Single-workspace view
**VS Code:**
- Multi-root workspace support
- Workspace-specific settings
- Per-workspace launch configurations
- Integrated terminal with ROS environment

### 5. **Developer Workflow**
**rqt_launchtree:** External tool, context switching required
**VS Code:**
- All tools in one window (editor, tree, terminal, debugger)
- Quick switch with keyboard shortcuts
- Persistent layout and state
- Integrated git support (see which launch files changed)

### 6. **Extensibility**
**rqt_launchtree:** Limited customization
**VS Code:**
- Custom commands and keybindings
- Integration with other extensions
- Customizable views and panels
- API for extension developers

## Use Case Comparison

### Use Case 1: Understanding a Complex Launch File

**With rqt_launchtree:**
1. Open rqt in terminal
2. Load rqt_launchtree plugin
3. Navigate to launch file
4. View tree structure
5. Switch to text editor to see source
6. Alt+Tab between rqt and editor

**With VS Code Launch Tree:**
1. Open VS Code (already running)
2. Expand launch tree in sidebar
3. Click node → Auto-jumps to definition
4. Edit code inline
5. Save → Tree auto-refreshes
6. All in one window

### Use Case 2: Debugging a Launch Configuration

**With rqt_launchtree:**
1. View tree in rqt
2. Note node parameters
3. Close rqt
4. Open VS Code
5. Create debug configuration manually
6. Set breakpoints
7. Run debugger

**With VS Code Launch Tree:**
1. Right-click launch file in tree
2. Select "Debug Launch File"
3. Auto-creates debug config
4. Breakpoints already set
5. Debug session starts immediately

### Use Case 3: Finding Where a Launch File is Used

**With rqt_launchtree:**
1. View launch tree
2. See included files
3. Manually search workspace with grep
4. Open files in editor

**With VS Code Launch Tree:**
1. Right-click launch file
2. Select "Find Usages"
3. Results appear in search panel
4. Click result → Jump to line
5. All results in searchable list

### Use Case 4: Modifying Parameters Across Multiple Files

**With rqt_launchtree:**
1. View parameters in rqt
2. Note parameter names
3. Switch to editor
4. Use Find & Replace
5. Manually verify changes

**With VS Code Launch Tree:**
1. Click parameter in tree
2. Select "Find All References"
3. See all usages highlighted
4. Use rename refactoring
5. Preview changes before applying

## Integration Opportunities

### 1. **Test Explorer Integration**
- Show test launch files in tree
- Run/debug tests from tree view
- Link to existing ROS Test Provider

### 2. **Lifecycle Node Management**
- Show lifecycle state in tree (🟢 active, 🔵 inactive)
- Trigger state transitions from tree
- Integrate with existing lifecycle commands

### 3. **ROS 2 Status Monitor**
- Show running nodes from launch file
- Highlight which nodes are currently active
- Link to ROS 2 Status Webview

### 4. **Build System Integration**
- Show which packages need rebuilding
- Run colcon build from tree
- Show build errors in tree

### 5. **Git Integration**
- Show modified launch files
- Show untracked launch files
- Compare launch file versions

## Migration Path from rqt_launchtree

**Users can continue using both tools:**
- rqt_launchtree for runtime visualization
- VS Code tree for development workflow

**VS Code tree offers superset of features:**
- All viewing capabilities of rqt_launchtree
- Plus: editing, debugging, navigation
- Reduces context switching

## Performance Considerations

### rqt_launchtree
- **Pros:** Can run on remote ROS master
- **Cons:** Separate process, memory overhead

### VS Code Launch Tree
- **Pros:** Integrated, cached, lazy loading
- **Cons:** Requires VS Code (already open for development)

**Conclusion:** VS Code tree is more efficient for development workflow since VS Code is already running.

## Future Enhancements Beyond rqt_launchtree

### 1. **Live Launch File Validation**
- Real-time syntax checking
- Parameter type validation
- Missing dependency warnings
- Invalid path detection

### 2. **Launch File Templates**
- Quick-create common launch patterns
- Snippet library for launch files
- Wizard for complex configurations

### 3. **Visual Launch File Editor**
- Drag-and-drop node arrangement
- Visual parameter editing
- Graph view of node connections

### 4. **Launch File Diff**
- Compare launch files side-by-side
- Show parameter differences
- Merge launch configurations

### 5. **AI-Assisted Launch Files**
- Copilot suggestions for launch files
- Auto-complete for node configurations
- Parameter recommendations

## Conclusion

The VS Code Launch File Tree Viewer provides all the core features of rqt_launchtree while adding powerful IDE integration features that significantly improve the developer workflow. By embedding the tree view directly in the development environment, developers can:

- Reduce context switching
- Navigate code faster
- Debug more efficiently
- Edit with full IDE support
- Leverage existing VS Code features

**Recommendation:** Implement the VS Code Launch Tree Viewer as a complement to (not replacement for) rqt_launchtree. Both tools serve different purposes:
- **rqt_launchtree:** Runtime visualization and monitoring
- **VS Code tree:** Development, editing, and debugging

---

**Author:** GitHub Copilot  
**Date:** 2026-01-31  
**Version:** 1.0
