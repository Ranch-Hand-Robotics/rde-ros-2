---
name: ros2-build
description: "Use when building or testing ROS 2 workspaces: colcon, ament_cmake, ament_python, CMake, underlay and overlay sourcing, dependency failures, compiler diagnostics, and test results."
user-invocable: true
---

# ROS 2 Build and Test

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect host OS, active shell, CPU architecture, ROS distribution/version, and active native, Pixi, container, or remote environment.
- Locate compatible `colcon`, ROS CLI, compiler, CMake, and Python binaries before use; never execute an unverified Unix shim on Windows.
- Discover actual setup scripts and shell-compatible activation; do not assume installation paths or silently switch environments.
- Confirm the approved simulation versus hardware target and isolation from other ROS graphs before runtime tests.
- Do not operate a live robot, arm, or drone without explicit approval; building does not authorize launching hardware.

## Inspect the workspace
1. Find workspace roots, `package.xml` files, build-type exports, and existing build presets or CI commands.
2. Distinguish `colcon` workspace orchestration from each package's ament build system.
3. For `ament_cmake`, inspect targets, dependency discovery, install rules, and `ament_package()`.
4. For `ament_python`, inspect `setup.py`/`setup.cfg`, package discovery, resource markers, and console entry points.
5. Record underlay prefixes, overlay order, Python interpreter, compiler ABI, and architecture compatibility.
6. Check missing dependencies using the project's supported dependency workflow; report gaps before proposing installation.

## Build incrementally
1. Start in a clean, correctly activated shell; source the intended underlay before building an overlay.
2. Inspect `colcon list` after preflight and compare discovered packages with expected workspace contents.
3. Use `colcon build --packages-up-to <package>` when workspace dependencies also need building.
4. Use `--packages-select <package>` only when its dependencies are already built and available.
5. Preserve repository options for merged/isolated installs and generators; do not mix incompatible build caches.
6. Use `--symlink-install` for supported development workflows, accounting for Windows permissions and file-copy exceptions.
7. Choose CMake build type deliberately; use debug symbols when diagnostics require them.
8. Capture the exit status immediately and read the first failing package's detailed log, not only the final summary.
9. Source the resulting overlay using the detected shell, then verify package prefixes resolve to that overlay.

## Test and diagnose
- Use [meaningful testing](../ros2-test/SKILL.md) for test design, regression
	sensitivity, and behavior-based assertions rather than mirroring implementation.
- Run scoped `colcon test --packages-select <package>` and inspect `colcon test-result --verbose`.
- Check test discovery and counts: a command exiting successfully with zero expected tests is not sufficient.
- Separate configure, compile, link, install, import, and runtime failures before changing code.
- Inspect `log/latest_build` or the configured log base and preserve the original compiler traceback.
- For missing headers/libraries, verify manifest dependencies, CMake target linkage, exports, and sourced prefixes.
- For Python import failures, verify the runtime interpreter, installed package contents, and entry-point shebang compatibility.
- For stale interfaces, rebuild generators and dependents; check overlay shadowing before deleting artifacts.
- Never recursively delete `build`, `install`, or logs without an explicit scope and approval for lost artifacts.

## Validation and handoff
- Reproduce from a fresh shell without sourcing a stale version of the overlay being rebuilt.
- Verify installed executables/resources as well as source-tree tests; a build alone does not prove installation works.
- Exercise an approved isolated smoke test only when runtime execution is authorized.
- Report commands, environment, package selection, exit statuses, test counts, and remaining failures.
- Distinguish verified host results from untested Windows, macOS, Linux, or cross-architecture assumptions.
