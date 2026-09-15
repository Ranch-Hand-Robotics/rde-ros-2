---
name: ros2-packaging
description: "Use when packaging or releasing ROS 2 packages: package.xml, rosdep, ament exports, interface packages, installed share resources, Python entry points, reproducible releases, bloom, rosdistro, and Pixi."
user-invocable: true
---

# ROS 2 Packaging and Releases

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, active shell, architecture, ROS distribution/version, and native, Pixi, container, or remote environment.
- Verify compatible packaging tools, compiler, Python, and any requested `rosdep` or `bloom` executable before use.
- Discover shell-compatible setup scripts; never assume installation paths, switch shells silently, or run incompatible shims.
- Confirm the approved simulation versus hardware target before installed-package smoke tests.
- Do not operate a live robot, arm, or drone without explicit approval; release preparation grants no runtime authority.

## Audit package metadata
1. Check package name/version consistency, description, maintainers, license, and build-type export in `package.xml`.
2. Distinguish build-tool, build, build-export, execution, and test dependencies; avoid undeclared transitive dependencies.
3. Resolve dependency keys against the selected ROS distribution and OS; do not substitute arbitrary system package names.
4. Use `rosdep check` or a supported dry-run workflow before requesting dependency installation.
5. For Pixi-managed environments, follow project manifests and lockfiles rather than assuming rosdep manages everything.
6. Do not use vcpkg; preserve the project's supported native or Pixi dependency strategy.

## Verify install contracts
1. For CMake libraries, install headers and targets and export targets/dependencies for downstream `find_package` consumers.
2. Ensure exported include paths are relocatable and do not embed source-tree or developer-machine paths.
3. Install ROS executables in the package-appropriate location and shared launch/config/model assets under `share/<package>`.
4. For Python packages, install the ament resource-index marker, `package.xml`, and required share resources.
5. Verify console entry points, importable modules, script install directories, and the interpreter used at runtime.
6. Resolve resources through the ament index rather than the current directory or a guessed source checkout.
7. Inspect the installed tree; source-file existence is not proof that the release contains it.

## Interface packages
- Check message/service/action definitions, referenced types, and generator dependencies.
- Declare runtime support and interface-package group membership appropriate to the selected ROS version.
- Ensure generated interfaces and type support are available to installed C++ and Python consumers.
- Prefer a separate interface package when application packaging or dependency cycles require it.
- Test a downstream consumer from a clean overlay; source-tree imports can hide missing exports.

## Reproducible release procedure
1. Record the source commit/tag, dependency resolution, toolchain, target platform, and build/install options.
2. Build and test in a clean environment with only declared dependencies; verify licenses and bundled third-party notices.
3. Confirm version/changelog consistency and exclude secrets, caches, local paths, and generated development artifacts.
4. For ROS build-farm releases, verify current bloom/rosdistro support, release repository, tracks, and target distribution policy.
5. Treat bloom-generated changes, tag pushes, and rosdistro submissions as separate publishing actions requiring approval.
6. For Pixi distribution, validate supported platform solves and lockfile reproducibility; it is not a bloom replacement for build-farm metadata.
7. Never claim byte-identical output unless independently reproduced; record known nondeterministic build inputs.

## Validation and edge cases
- Test installation without relying on the source directory, including launch/config lookup and console entry points.
- Check case-sensitive filenames, executable permissions, Windows DLL lookup, and platform-specific binary availability.
- Validate architecture/ABI compatibility and distinguish source releases from platform-specific binary artifacts.
- Report installed-file checks, downstream-consumer results, supported platforms, and publishing steps not performed.
