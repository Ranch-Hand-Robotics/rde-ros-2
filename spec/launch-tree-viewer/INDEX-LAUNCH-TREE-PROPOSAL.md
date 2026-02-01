# 📋 Launch File Tree Viewer - Documentation Index

**Status:** ✅ Proposal Complete - Ready for Implementation  
**Date:** 2026-01-31  
**Issue:** [#318 - Launch file tree viewer](https://github.com/ms-iot/vscode-ros/issues/318)

---

## 📖 Quick Navigation

| Document | Purpose | Audience | Size |
|----------|---------|----------|------|
| [LAUNCH_TREE_VIEWER_PROPOSAL.md](./LAUNCH_TREE_VIEWER_PROPOSAL.md) | **Main design document** - Problem, solution, implementation plan | Decision makers, architects | 14 KB |
| [docs/QUICK_START_LAUNCH_TREE.md](./docs/QUICK_START_LAUNCH_TREE.md) | **Quick start guide** - What it is, how to use it | End users, testers | 7 KB |
| [docs/launch-tree-mockups.md](./docs/launch-tree-mockups.md) | **UI mockups** - Visual designs and interaction patterns | Designers, developers | 13 KB |
| [docs/launch-tree-comparison.md](./docs/launch-tree-comparison.md) | **Feature comparison** - vs rqt_launchtree | Product managers | 9 KB |
| [docs/launch-tree-technical-guide.md](./docs/launch-tree-technical-guide.md) | **Implementation code** - Complete TypeScript implementation | Developers | 36 KB |
| [docs/launch-tree-architecture.md](./docs/launch-tree-architecture.md) | **Architecture diagrams** - System design, data flows | Architects, developers | 15 KB |
| [docs/README-LAUNCH-TREE.md](./docs/README-LAUNCH-TREE.md) | **Implementation guide** - Roadmap, getting started | Team leads, developers | 8 KB |

**Total Documentation:** 7 files, ~102 KB

---

## 🎯 Read This First

### If you are a...

**👔 Decision Maker / Product Manager**
1. Start with: [LAUNCH_TREE_VIEWER_PROPOSAL.md](./LAUNCH_TREE_VIEWER_PROPOSAL.md)
2. Then read: [docs/launch-tree-comparison.md](./docs/launch-tree-comparison.md)
3. Review: Success criteria section in proposal

**🎨 Designer / UX Specialist**
1. Start with: [docs/launch-tree-mockups.md](./docs/launch-tree-mockups.md)
2. Then read: UI sections in [LAUNCH_TREE_VIEWER_PROPOSAL.md](./LAUNCH_TREE_VIEWER_PROPOSAL.md)
3. Review: Interaction patterns in mockups

**👨‍💻 Developer / Engineer**
1. Start with: [docs/README-LAUNCH-TREE.md](./docs/README-LAUNCH-TREE.md)
2. Then study: [docs/launch-tree-technical-guide.md](./docs/launch-tree-technical-guide.md)
3. Reference: [docs/launch-tree-architecture.md](./docs/launch-tree-architecture.md)

**🏗️ Architect / Tech Lead**
1. Start with: [docs/launch-tree-architecture.md](./docs/launch-tree-architecture.md)
2. Then read: [LAUNCH_TREE_VIEWER_PROPOSAL.md](./LAUNCH_TREE_VIEWER_PROPOSAL.md)
3. Review: Technical considerations section

**👤 End User / Tester**
1. Start with: [docs/QUICK_START_LAUNCH_TREE.md](./docs/QUICK_START_LAUNCH_TREE.md)
2. Then read: User experience section in proposal

---

## 📊 Executive Summary

### The Problem
> "Launch files melt my brain" - Issue #318

ROS 2 launch files are complex with multiple levels of includes, numerous nodes, and intricate configurations. Developers need visual understanding of launch file structure.

### The Solution
A **VS Code TreeView** that visualizes launch files in the Explorer sidebar:

```
🚀 ROS 2 LAUNCH FILES
  └─ 📦 Package
      └─ 🚀 Launch File
          └─ 📍 Node
              └─ 📊 Parameters, Remaps, Arguments
```

**Click any item** → Navigate to code  
**Right-click launch file** → Run or Debug  
**Auto-refresh** on file changes

### Key Benefits
- ✅ Understand structure at a glance
- ✅ Navigate with single click
- ✅ Debug directly from tree
- ✅ Find launch file dependencies
- ✅ No context switching

### Implementation Timeline
- **Phase 1 (MVP):** 2-3 days - Basic tree view
- **Phase 2:** 1-2 days - Enhanced navigation
- **Phase 3:** 2-3 days - Advanced features

**Total Effort:** ~1-2 weeks

---

## 🗺️ Document Map

### Design Documents

#### 1. LAUNCH_TREE_VIEWER_PROPOSAL.md
**The master design document**

Contains:
- Problem statement and user needs
- Proposed solution architecture
- Tree structure hierarchy (4 levels)
- View container configuration
- Click-to-navigate behavior
- Commands and interactions
- Phased implementation plan (3 phases)
- Technical considerations
- Success criteria
- References and open questions

**Key Sections:**
- § 1: Overview
- § 2: Problem Statement
- § 3: Proposed Solution
  - § 3.1: Tree View Integration
  - § 3.2: View Container Configuration
  - § 3.3: Data Model
  - § 3.4: Click-to-Navigate Behavior
  - § 3.5: Commands and Interactions
  - § 3.6: Advanced Features
- § 4: Implementation Plan
- § 5: Technical Considerations
- § 6: User Experience
- § 7: Success Criteria

#### 2. docs/launch-tree-mockups.md
**Visual design reference**

Contains:
- 9 detailed ASCII mockups
- Tree view layouts
- Context menu designs
- Empty/loading/error states
- Icon legend
- Color scheme (VS Code themes)
- Interaction patterns
- Accessibility notes

**Mockups:**
1. Tree in File Explorer (recommended)
2. Expanded node details
3. Context menu actions
4. Find usages results
5. Empty state
6. Loading state
7. Error state
8. Separate activity bar (alternative)
9. Inline actions (hover)

#### 3. docs/launch-tree-comparison.md
**Competitive analysis**

Contains:
- Feature comparison matrix (rqt_launchtree vs VS Code)
- Key advantages of VS Code integration
- Use case comparisons (4 scenarios)
- Integration opportunities
- Migration path
- Future enhancements

**Comparison Categories:**
- Core features
- Navigation
- Filtering/search
- Execution
- Integration
- User experience
- Platform support

### Technical Documents

#### 4. docs/launch-tree-technical-guide.md
**Implementation code reference**

Contains:
- Architecture overview
- File structure
- Complete TypeScript code for:
  - LaunchTreeDataProvider (300+ lines)
  - LaunchTreeItem (200+ lines)
  - LaunchFileParser (200+ lines)
  - Command handlers (100+ lines)
- Type definitions
- package.json configuration
- Testing strategy
- Performance optimizations
- Error handling

**Code Sections:**
- § 1: Architecture
- § 2: File Structure
- § 3: Type Definitions
- § 4: Implementation Details
  - § 4.1: LaunchTreeDataProvider
  - § 4.2: LaunchTreeItem
  - § 4.3: LaunchFileParser
  - § 4.4: Command Handlers
  - § 4.5: Extension Registration
- § 5: package.json Configuration
- § 6: Testing Strategy
- § 7: Performance & Error Handling

#### 5. docs/launch-tree-architecture.md
**System design diagrams**

Contains:
- System architecture diagram
- Data flow diagrams (5 flows)
- Class diagram
- Sequence diagrams
- State management
- File system watcher
- Extension lifecycle
- Integration points

**Diagrams:**
1. System architecture
2. Tree initialization flow
3. Launch file expansion flow
4. Node expansion flow
5. File navigation flow
6. Context menu flow
7. Class relationships
8. Refresh sequence

### Guide Documents

#### 6. docs/README-LAUNCH-TREE.md
**Implementation roadmap**

Contains:
- Documentation overview
- Implementation roadmap (3 phases)
- Key technical decisions
- Success metrics
- Getting started guide
- References
- Open questions (resolved)

**Roadmap:**
- Phase 1: Core Tree View (2-3 days)
- Phase 2: Enhanced Navigation (1-2 days)
- Phase 3: Advanced Features (2-3 days)

#### 7. docs/QUICK_START_LAUNCH_TREE.md
**End-user guide**

Contains:
- What is this feature?
- Before/after visualization
- Installation instructions
- Quick action guides
- Customization options
- Troubleshooting
- Tips & tricks
- Performance notes
- Benefits summary

**Quick Actions:**
- Navigate to launch file
- See node details
- Run launch file
- Debug launch file
- Find usages

---

## 🏗️ Technical Architecture

### High-Level Overview

```
VS Code Extension (TypeScript)
  ├─ LaunchTreeDataProvider (implements TreeDataProvider)
  │   ├─ manages tree state
  │   ├─ handles refresh events
  │   └─ creates LaunchTreeItem objects
  │
  ├─ LaunchFileParser
  │   ├─ finds ROS packages in workspace
  │   ├─ parses launch files via ros2_launch_dumper.py
  │   └─ manages cache (Map<path, data>)
  │
  └─ Command Handlers
      ├─ refresh()
      ├─ reveal()
      ├─ findUsages()
      ├─ run()
      └─ debug()

Python Subprocess (ROS 2 env)
  └─ ros2_launch_dumper.py
      ├─ parses .launch.py files
      └─ returns JSON (nodes, includes, args)
```

### Data Flow

```
1. User opens workspace
2. Extension scans for packages with launch files
3. User expands launch file in tree
4. Parser executes ros2_launch_dumper.py
5. Python script returns JSON data
6. Extension creates tree items
7. VS Code renders tree
8. User clicks item → Navigate to file/line
```

### Integration Points

- **ros2_launch_dumper.py** - Already exists, reused
- **ROS2.roslaunch** - Run command integration
- **Debug adapter** - Debug integration
- **File watcher** - Auto-refresh on changes

---

## ✅ Implementation Checklist

### Phase 1: Core Tree View (MVP)

**Files to Create:**
- [ ] `src/ros/launch-tree/types.ts`
- [ ] `src/ros/launch-tree/launch-tree-item.ts`
- [ ] `src/ros/launch-tree/launch-parser.ts`
- [ ] `src/ros/launch-tree/launch-tree-provider.ts`
- [ ] `src/ros/commands/launch-tree-commands.ts`
- [ ] `test/suite/launch-tree/launch-tree-provider.test.ts`
- [ ] `test/suite/launch-tree/launch-parser.test.ts`
- [ ] `test/suite/launch-tree/fixtures/simple.launch.py`

**Files to Modify:**
- [ ] `src/extension.ts` (register view & commands)
- [ ] `package.json` (add views & commands)

**Features:**
- [ ] Tree shows packages → launch files → nodes
- [ ] Click launch file opens in editor
- [ ] Click node jumps to definition
- [ ] Refresh command
- [ ] Basic error handling

### Phase 2: Enhanced Navigation

**Features:**
- [ ] "Reveal in Launch Tree" command
- [ ] Context menu actions (Run, Debug)
- [ ] "Find Usages" command
- [ ] File watcher for auto-refresh
- [ ] Improved error states

### Phase 3: Advanced Features

**Features:**
- [ ] Parameter detail levels
- [ ] Inline parameter editing
- [ ] Search/filter capability
- [ ] Include relationship visualization
- [ ] Enhanced icons and theming

---

## 📚 Additional Resources

### VS Code Extension API
- [TreeView API](https://code.visualstudio.com/api/extension-guides/tree-view)
- [Commands](https://code.visualstudio.com/api/references/commands)
- [FileSystemWatcher](https://code.visualstudio.com/api/references/vscode-api#FileSystemWatcher)

### ROS 2 Resources
- [Launch Files Documentation](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-Main.html)
- [rqt_launchtree GitHub](https://github.com/pschillinger/rqt_launchtree)

### Existing Extension Code
- `src/debugger/configuration/resolvers/ros2/launch.ts` - Launch parsing
- `assets/scripts/ros2_launch_dumper.py` - Python parser
- `src/test-provider/ros-test-provider.ts` - Example tree provider

---

## 🤝 Contributing

When implementing this feature:

1. **Start with Phase 1** - Get MVP working first
2. **Follow existing patterns** - Match extension code style
3. **Write tests** - All new code needs coverage
4. **Update docs** - Keep documentation in sync
5. **Small PRs** - Break work into logical commits
6. **Ask questions** - Clarify before coding

### Code Style
- Follow `.editorconfig` (2 spaces, LF)
- Use TypeScript strict mode
- Write JSDoc comments for public APIs
- Prefer async/await over callbacks

### Testing
- Unit tests for all logic
- Integration tests with sample launch files
- Manual testing in Extension Host

---

## 📞 Questions & Feedback

- **GitHub Issue:** Tag with `[launch-tree]` prefix
- **Pull Request:** Reference this proposal
- **Questions:** Ask in issue comments

---

## 📈 Success Metrics

**MVP Requirements:**
- ✅ Tree view appears in Explorer
- ✅ Shows 3-level hierarchy minimum
- ✅ Click navigation works
- ✅ Refresh updates tree
- ✅ Handles errors gracefully

**Quality Requirements:**
- ✅ Parses 100+ files in < 5s
- ✅ No crashes on malformed files
- ✅ Cross-platform (Win/Linux/Mac)
- ✅ Follows VS Code UX patterns
- ✅ >80% test coverage

---

## 🎉 Summary

This proposal provides **complete design and implementation specifications** for a Launch File Tree Viewer feature that will:

1. **Solve the problem** - Make launch files easier to understand
2. **Integrate seamlessly** - Work natively in VS Code
3. **Add value** - Improve developer productivity
4. **Be maintainable** - Well-documented and tested

**Status: Ready for Implementation** 🚀

---

**Prepared by:** GitHub Copilot  
**Date:** 2026-01-31  
**Version:** 1.0  
**Next Steps:** Review → Approve → Implement
