# Launch File Tree Viewer - UI Mockups

This document provides ASCII art mockups of the proposed Launch File Tree Viewer interface.

## Mockup 1: Tree View in File Explorer (Recommended)

```
┌─────────────────────────────────────────────────────────┐
│ EXPLORER                                        ─ □ ✕   │
├─────────────────────────────────────────────────────────┤
│ 🔍 Search                                               │
├─────────────────────────────────────────────────────────┤
│ ▼ WORKSPACE                                             │
│   ▼ 📁 my_robot_workspace                               │
│     ▶ 📁 src                                            │
│     ▶ 📁 build                                          │
│     ▶ 📁 install                                        │
│     ▶ 📁 log                                            │
│                                                         │
│ ▼ ROS 2 LAUNCH FILES                         🔄        │
│   ▼ 📦 navigation_package                               │
│     ▼ 🚀 navigation.launch.py                          │
│       ▶ 📍 map_server                                   │
│       ▶ 📍 amcl                                         │
│       ▶ 📍 lifecycle_manager                            │
│       ▶ 🔗 Include: nav2_common.launch.py              │
│     ▶ 🚀 localization.launch.py                        │
│                                                         │
│   ▼ 📦 perception_package                               │
│     ▶ 🚀 camera.launch.py                              │
│     ▶ 🚀 lidar.launch.py                               │
│                                                         │
│   ▶ 📦 control_package                                  │
│                                                         │
│ ▼ OUTLINE                                               │
└─────────────────────────────────────────────────────────┘
```

## Mockup 2: Expanded Node Details

```
┌─────────────────────────────────────────────────────────┐
│ ROS 2 LAUNCH FILES                           🔄        │
├─────────────────────────────────────────────────────────┤
│ ▼ 📦 map_server_pkg                                     │
│   ▼ 🚀 map_server.launch.py                            │
│     ▼ 📍 map_server_node                                │
│       │ 📄 Package: nav2_map_server                     │
│       │ 🔧 Executable: map_server                       │
│       │ 🏷️  Namespace: /mapping                         │
│       ▼ 📊 Parameters (4)                               │
│         │ yaml_filename: ${map_file}                    │
│         │ use_sim_time: true                            │
│         │ frame_id: map                                 │
│         │ topic_name: /map                              │
│       ▼ 🔀 Remappings (2)                               │
│         │ /tf → /mapping/tf                             │
│         │ /map → /global_map                            │
│       ▼ 📝 Arguments (1)                                │
│         │ --ros-args --log-level info                   │
│                                                         │
│     ▼ 📍 lifecycle_manager                              │
│       │ 📄 Package: nav2_lifecycle_manager              │
│       │ 🔧 Executable: lifecycle_manager                │
│       ▼ 📊 Parameters (2)                               │
│         │ autostart: true                               │
│         │ node_names: ['map_server_node']               │
│                                                         │
│     ▼ 🔗 Include: nav2_bringup/params/nav2_params.yaml │
│       │ 🗂️  File exists: ✓                              │
│       │ 📂 Path: /opt/ros/humble/share/...             │
└─────────────────────────────────────────────────────────┘
```

## Mockup 3: Context Menu Actions

```
┌─────────────────────────────────────────────────────────┐
│ ▼ 📦 navigation_package                                 │
│   ▼ 🚀 navigation.launch.py ◄─┐                        │
│     ▶ 📍 map_server           │                        │
│     ▶ 📍 amcl                 │                        │
│                                │                        │
│     ┌──────────────────────────┴──────────────────────┐│
│     │  ▶ Run Launch File                              ││
│     │  ▶ Debug Launch File                            ││
│     │  ─────────────────────────────                  ││
│     │  ▶ Open File                                    ││
│     │  ▶ Reveal in Explorer                           ││
│     │  ▶ Copy Path                                    ││
│     │  ─────────────────────────────────              ││
│     │  ▶ Find Usages                                  ││
│     │  ▶ Find References                              ││
│     └─────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
```

## Mockup 4: Find Usages Results

```
┌─────────────────────────────────────────────────────────┐
│ SEARCH: Usages of navigation.launch.py                 │
├─────────────────────────────────────────────────────────┤
│ 3 results in 2 files                                    │
│                                                         │
│ 📁 robot_bringup/bringup.launch.py                     │
│   Line 15: IncludeLaunchDescription(                    │
│   Line 16:   'navigation_package', 'navigation.launch.py'│
│                                                         │
│ 📁 robot_bringup/sim.launch.py                         │
│   Line 23: include = IncludeLaunchDescription(          │
│   Line 24:   'navigation_package', 'navigation.launch.py'│
│                                                         │
│ 📁 test/test_navigation.launch.py                      │
│   Line 8: from navigation_package import navigation     │
└─────────────────────────────────────────────────────────┘
```

## Mockup 5: Empty State

```
┌─────────────────────────────────────────────────────────┐
│ ROS 2 LAUNCH FILES                           🔄        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│           ╔═══════════════════════════════╗            │
│           ║                               ║            │
│           ║    📭 No Launch Files Found   ║            │
│           ║                               ║            │
│           ║   No .launch.py files were    ║            │
│           ║   found in this workspace.    ║            │
│           ║                               ║            │
│           ║   [Create Launch File]        ║            │
│           ║   [Learn More]                ║            │
│           ║                               ║            │
│           ╚═══════════════════════════════╝            │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Mockup 6: Loading State

```
┌─────────────────────────────────────────────────────────┐
│ ROS 2 LAUNCH FILES                           🔄        │
├─────────────────────────────────────────────────────────┤
│ ⏳ Scanning workspace for launch files...               │
│                                                         │
│ ▼ 📦 navigation_package                                 │
│   ⏳ Parsing launch files...                            │
│                                                         │
│ ▼ 📦 perception_package                                 │
│   ✓ 2 launch files loaded                               │
│   ▶ 🚀 camera.launch.py                                │
│   ▶ 🚀 lidar.launch.py                                 │
└─────────────────────────────────────────────────────────┘
```

## Mockup 7: Error State

```
┌─────────────────────────────────────────────────────────┐
│ ROS 2 LAUNCH FILES                           🔄        │
├─────────────────────────────────────────────────────────┤
│ ⚠️  ROS 2 Environment Not Configured                    │
│                                                         │
│ The extension could not find a ROS 2 installation.     │
│ Please configure your ROS 2 setup script.               │
│                                                         │
│ [Open Settings] [Learn More]                            │
│                                                         │
├─────────────────────────────────────────────────────────┤
│ ▼ 📦 navigation_package                                 │
│   ▼ 🚀 broken.launch.py                                │
│     ⚠️  Parse Error: Invalid Python syntax              │
│     [View Error Details]                                │
│                                                         │
│   ▶ 🚀 working.launch.py                               │
└─────────────────────────────────────────────────────────┘
```

## Mockup 8: Separate Activity Bar (Alternative)

```
┌──┬──────────────────────────────────────────────────────┐
│  │ ROS 2                                      ─ □ ✕    │
│📁│ ──────────────────────────────────────────────────── │
│  │                                                      │
│🔍│ ▼ LAUNCH FILES                            🔄        │
│  │   ▼ 📦 navigation_package                            │
│🌿│     ▼ 🚀 navigation.launch.py                       │
│  │       ▶ 📍 map_server                                │
│⚙️│       ▶ 📍 amcl                                      │
│  │       ▶ 📍 lifecycle_manager                         │
│🤖│     ▶ 🚀 localization.launch.py                     │
│◀ │   ▼ 📦 perception_package                            │
│  │     ▶ 🚀 camera.launch.py                           │
│  │     ▶ 🚀 lidar.launch.py                            │
│  │                                                      │
│  │ ▼ LIFECYCLE NODES                                    │
│  │   🟢 map_server_node [active]                        │
│  │   🔵 amcl_node [inactive]                            │
│  │                                                      │
│  │ ▼ RUNNING NODES                                      │
│  │   ▶ /map_server                                      │
│  │   ▶ /amcl                                            │
└──┴──────────────────────────────────────────────────────┘
    ↑
  ROS 2 
  Activity
  Bar Icon
```

## Mockup 9: Inline Actions (Hover)

```
┌─────────────────────────────────────────────────────────┐
│ ▼ 📦 navigation_package                                 │
│   ▼ 🚀 navigation.launch.py  ▶️ 🐛 📄 🔍              │
│     │                          │  │  │  │              │
│     │                          │  │  │  └─ Find Usages │
│     │                          │  │  └──── Open File   │
│     │                          │  └─────── Debug       │
│     │                          └────────── Run         │
│     │                                                   │
│     ▼ 📍 map_server           🔍                       │
│       │ 📄 nav2_map_server                              │
│       │ 🔧 map_server                                   │
│       │                                                 │
│       ▼ 📊 Parameters                                   │
│         │ yaml_filename: ${map_file}  📝 ✏️            │
│         │                               │  │            │
│         │                               │  └─ Edit     │
│         │                               └──── Copy     │
└─────────────────────────────────────────────────────────┘
```

## Icon Legend

| Icon | Meaning |
|------|---------|
| 📦 | ROS Package |
| 🚀 | Launch File |
| 📍 | ROS Node |
| 🔗 | Include/Import |
| 📊 | Parameters Section |
| 🔀 | Remappings Section |
| 📝 | Arguments Section |
| 📄 | Package Name |
| 🔧 | Executable |
| 🏷️ | Namespace |
| 🗂️ | File/Path |
| ⏳ | Loading |
| ⚠️ | Warning/Error |
| ✓ | Success |
| 🔄 | Refresh Button |
| ▶️ | Run Action |
| 🐛 | Debug Action |
| 🔍 | Search/Find |
| 📝 | Edit Action |
| ✏️ | Edit Inline |

## Color Scheme (VS Code Dark Theme)

- **Packages**: Blue (#4EC9B0)
- **Launch Files**: Purple (#C586C0)
- **Nodes**: Yellow (#DCDCAA)
- **Parameters**: Light Blue (#9CDCFE)
- **Includes**: Green (#4EC9B0)
- **Errors**: Red (#F48771)
- **Success**: Green (#89D185)

## Interaction Patterns

### Single Click
- **Package**: Collapse/expand package
- **Launch File**: Collapse/expand file contents
- **Node**: Collapse/expand node details
- **Parameter**: Open launch file at parameter definition

### Double Click
- **Package**: Open package directory in explorer
- **Launch File**: Open file in editor
- **Node**: Jump to node definition in launch file
- **Include**: Open included file

### Right Click
- Shows context menu with available actions (see Mockup 3)

### Hover
- Shows tooltip with additional information
- Shows inline action buttons (see Mockup 9)

### Drag & Drop
- Drag launch file to editor to open
- Drag launch file to terminal to run

## Accessibility

- **Screen Reader**: All tree items have descriptive labels
- **Keyboard Navigation**: Full keyboard support (arrows, enter, space)
- **High Contrast**: Icons and colors support high contrast themes
- **Focus Indicators**: Clear focus states for keyboard navigation

---

**Note:** These mockups use ASCII art and may not render perfectly in all text editors. The actual implementation will use VS Code's native tree view components with proper icons and styling.
