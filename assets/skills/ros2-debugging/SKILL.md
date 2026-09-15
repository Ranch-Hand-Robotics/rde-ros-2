---
name: ros2-debugging
description: "Use when debugging ROS 2 in VS Code: type ros2 launch configurations, process attach, C++ symbols, Python interpreters, launch files, installed versus source paths, and unbound breakpoints."
user-invocable: true
---

# ROS 2 Debugging in VS Code

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, active shell, architecture, ROS distribution/version, and local, remote, container, or Pixi extension-host environment.
- Locate compatible debugger binaries, target executable, compiler tooling, and Python interpreter before use.
- Discover shell-compatible ROS setup scripts; never assume installation paths or silently switch execution environments.
- Confirm the approved simulation versus hardware target and whether pausing a process is safe.
- Do not operate a live robot, arm, or drone without explicit approval; breakpoints can interrupt control loops and watchdogs.

## Identify the actual process
1. Inspect the installed extension's debugger schema and examples; use VS Code debug `type: ros2`, not legacy ROS 1 configuration.
2. Select launch versus attach based on whether the extension should create a process or debug an already running one.
3. Resolve the launch file and package executable from the active overlay and installed package metadata.
4. Record arguments, parameters, remappings, working directory, environment, and executable path from evidence.
5. For multi-node launch files, identify the specific child process; debugging the launcher is not debugging every node.
6. Check composable-node containers and lifecycle startup state when the expected standalone node process does not exist.

## C++ debugging
1. Verify debug symbols match the exact running binary and architecture; check optimization and stripping settings.
2. Build the affected package with an appropriate symbol-bearing CMake configuration and source the intended overlay.
3. Confirm the platform-appropriate C++ debug adapter/toolchain supports the target process and symbol format.
4. For attach, identify the PID using executable and arguments, not a potentially duplicated node name alone.
5. Inspect loaded modules and symbol status before changing breakpoints or rebuilding unrelated packages.
6. Configure source mappings only from observed debug-info paths and verified local files; never guess source paths.
7. Request appropriate attach permissions when blocked; do not globally weaken ptrace, code-signing, or security settings.

## Python debugging
1. Identify the interpreter actually running the node, including entry-point shebangs and native/Pixi/virtual environment boundaries.
2. Verify ROS Python modules and debugger support are available to that interpreter, not merely the editor's default interpreter.
3. Inspect the loaded module's actual filename to distinguish installed copies from source files or symlink installs.
4. Use launch or an explicitly authorized attach workflow supported by the installed extension/debugger versions.
5. Do not expose a debugger listener publicly; restrict binding and use approved authenticated transport when remote access is needed.
6. Confirm breakpoints bind in loaded code and explain optimized/native-extension regions where Python stepping cannot work.

## Diagnose launch and source mismatches
- Compare terminal and extension-host environments; opening a terminal does not retroactively source the extension host.
- Verify installed launch/config assets after edits; source changes may require rebuild/reinstall before debugging.
- Resolve stale overlay shadowing using actual package prefixes, process paths, and timestamps.
- For remote/container sessions, verify the debugger, process, and source mapping belong to the intended machine.
- Preserve child-node output and full error context; classify launch parsing, environment, attach, and application failures separately.
- Query lifecycle state before transitions; an unconfigured node may intentionally not produce data yet.

## Validation
- In approved simulation or a non-actuating fixture, hit a breakpoint in the intended node and inspect a known variable.
- Test continue, step, exception behavior, and stop/detach semantics without leaving orphaned processes.
- Confirm whether stop terminates launched nodes or detaches attached ones; verify graph/process cleanup explicitly.
- Record executable/interpreter paths, symbol evidence, configuration changes, and any unverified hardware behavior.
