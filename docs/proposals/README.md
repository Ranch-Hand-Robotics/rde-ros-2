# Enhanced IntelliSense Path Discovery - Implementation Proposal

## Summary

This proposal addresses [Issue #168](https://github.com/ms-iot/vscode-ros/issues/168) - improving the mechanism for "Update C++ Properties" and "Update Python Path" commands to automatically discover include paths from extended/overlaid ROS 2 workspaces.

## Problem

The current implementation only checks top-level `<prefix>/include` directories and uses `PYTHONPATH` for Python paths. This causes IntelliSense to fail when:

- Working with extended workspaces (e.g., custom workspace extending MoveIt workspace)
- Headers are in package-specific locations like `install/<package>/include`
- Python packages use `sys.path.insert()` in their `__init__.py` files

Users experience red squiggles in the editor despite successful builds, requiring manual path configuration.

## Proposed Solution

Implement recursive directory discovery that:

1. **For C++ Paths**: Walk prefix directories to find all `include/` folders (not just top-level)
2. **For Python Paths**: Find Python packages by locating `__init__.py` files
3. **Performance**: Use caching, depth limiting, and smart filtering
4. **Compatibility**: Preserve backward compatibility with existing configurations

## Key Features

- ✅ Discovers package-specific include directories
- ✅ Handles workspace chaining/extension correctly
- ✅ Finds Python packages using `sys.path.insert()`
- ✅ Respects workspace overlay order
- ✅ Caching for performance (< 2 seconds for typical workspaces)
- ✅ Cross-platform compatible (Windows, Linux, macOS)
- ✅ Backward compatible with existing workflows

## Implementation Approach

### New Module: `src/ros/path-discovery.ts`

Core functions:
- `discoverIncludeDirectories(rootPath, maxDepth)` - Recursively find include directories
- `discoverPythonPackages(rootPath, maxDepth)` - Find Python packages by `__init__.py`
- `PathDiscoveryCache` - Cache discovered paths with TTL

### Modified Files

- `src/ros/ros2/ros2.ts` - Update `getIncludeDirs()` to use new discovery
- `src/ros/build-env-utils.ts` - Update `updatePythonPath()` to use new discovery
- `src/extension.ts` - Add cache invalidation on workspace changes

## Testing Strategy

- Unit tests for discovery functions
- Integration tests with various workspace layouts:
  - Single workspace
  - Extended workspace (A extends B)
  - Chained workspace (A extends B extends C)
  - Merged vs. per-package install layouts
- Performance tests to ensure < 2 second discovery time

## Timeline

- **Phase 1** (Days 1-3): Core infrastructure and discovery functions
- **Phase 2** (Days 4-6): Integration with existing commands
- **Phase 3** (Days 7-9): Testing and documentation
- **Phase 4** (Days 10-12): Refinement and release

**Total**: 2-3 weeks

## Full Proposal

See the complete proposal with implementation details, code samples, and analysis in:  
📄 **[docs/proposals/enhanced-intellisense-path-discovery.md](enhanced-intellisense-path-discovery.md)**

## Benefits

1. **Better User Experience**: IntelliSense works out-of-the-box with extended workspaces
2. **Reduced Support Load**: Fewer issues about missing headers/imports
3. **Improved Onboarding**: New users don't need to manually configure paths
4. **Accurate IntelliSense**: All workspace paths discovered automatically

## References

- Original Issue: https://github.com/ms-iot/vscode-ros/issues/168
- ROS 2 Workspace Docs: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
- Colcon Documentation: https://colcon.readthedocs.io/
