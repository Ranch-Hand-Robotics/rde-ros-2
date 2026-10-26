---
name: ros2-pixi
description: "Use when setting up, migrating, building, or troubleshooting ROS 2 with Pixi, RoboStack, or ruben-arts/pixi-ros: pixi.toml, pyproject.toml, pixi.lock, package.xml dependency mapping, channels, platforms, environments, activation, colcon tasks, and reproducible CI. Distinguishes pixi-ros init from the pixi-build-ros backend."
user-invocable: true
---

# ROS 2 with Pixi

## Required preflight

- Read [development ground rules](../ros2-development/SKILL.md); stop execution
  if unavailable. Confirm the execution host, OS, shell, architecture, ROS 2
  distro, workspace, and whether this is native, WSL, a container, or remote.
- Locate a compatible Pixi executable and check `pixi --version` and command
  help. On Windows use verified native executables, not extensionless Unix shims;
  PowerShell, cmd, and Bash have different activation and quoting rules.
- Inspect `pixi.toml` or `[tool.pixi]` in `pyproject.toml`, `pixi.lock`, package.xml
  files, CI, tasks, activation scripts, features, environments, and solve groups.
  Identify the exact manifest and environment; do not assume `default` is right.
- Separate a project-local Pixi workspace, an RDE-managed ROS installation, and
  an official ROS binary installation that uses Pixi for its dependencies.
  Do not convert one into another or combine their prefixes implicitly.
- Preserve uncommitted work. Inspect before editing; obtain approval for global
  tool installation, channel changes, source clones, destructive cleanup, and
  shell-profile or system changes. Never operate hardware without explicit approval.

## Choose the workflow

1. **Existing Pixi project:** maintain its manifest, lock, named environments,
   and tasks. Do not run an initializer merely to fix one missing dependency.
2. **ROS source workspace moving to Pixi:** use
   [pixi-ros initialization and caveats](./references/pixi-ros.md). It maps
   package.xml dependencies and creates a Pixi/colcon workflow; it is not the
   `ros2` CLI, a ROS distribution, or an automatic replacement for every rosdep key.
3. **New manual environment:** consult the
   [Pixi ROS 2 tutorial](https://pixi.prefix.dev/latest/tutorials/ros2/) and
   [RoboStack setup guide](https://robostack.github.io/GettingStarted.html).
   Select verified distro channels and only the requested target platforms.
   Adapt examples to the chosen distro; do not copy old Python/CMake pins blindly.
4. **One missing dependency:** use [dependency recovery](../rosdep/SKILL.md)
   for a targeted binary lookup or an approved peer-directory source checkout.
   Do not run native `rosdep install`, `conda install`, or `mamba install` into
   a Pixi-managed prefix; express changes through the project's Pixi manifest.
5. **Conda package artifacts or source-build backends:** distinguish the optional
   [pixi-build-ros backend](https://pixi.prefix.dev/latest/build/backends/pixi-build-ros/)
   from `pixi-ros`. Check current preview/version requirements; preserve colcon
   unless migration is requested. Publishing requires separate approval.

## Resolve dependencies and preserve reproducibility

1. Map the target to `win-64`, `linux-64`, `osx-64`, `osx-arm64`, or the actual
   supported architecture. A platform accepted by Pixi or pixi-ros does not prove
   RoboStack package availability. For Jetson/Linux ARM64, verify `linux-aarch64`
   binaries and JetPack/CUDA compatibility; never substitute x86_64 packages.
2. Check configured channels and their priority before adding packages. Inspect
   distro mutexes, Python ABI, compiler/runtime versions, and target dependencies.
   Keep unrelated ROS distros in separate environments; do not append channels
   from another distro to force a solve. Native ROS and RoboStack package names
   are not interchangeable.
3. Check requested package versions on each target using `pixi search` with
   explicit `--channel` and `--platform` options verified against local help.
   Review mappings and package.xml build/export/runtime/test dependencies.
   Local workspace packages should not silently become unrelated binary packages.
   Report network failures as unknown availability, not proof of missing packages.
4. Apply only scoped, authorized manifest changes and let Pixi generate the lock.
   Commit `pixi.lock` with the manifest for reproducible workspaces; do not edit
   it by hand, delete it to hide a solve conflict, or perform a blanket upgrade.
5. For a reviewed existing lock, prefer `pixi install --locked` and
   `pixi run --locked`, with `--manifest-path <manifest>` and
   `--environment <environment>` selecting the verified workspace and environment.
   `--locked` rejects an out-of-date lock; `--frozen` uses the lock without checking
   manifest freshness and is not an equivalent CI consistency check. Ordinary
   install/run can update the lock and install packages, so they are not read-only.
   For an approved new or changed manifest, solve intentionally, review the diff,
   then validate locked installation. See the
   [lockfile semantics](https://pixi.prefix.dev/latest/workspace/lock_file/).

## Activate, build, and test

- Begin in a clean shell without a conflicting system ROS, Conda base, or stale
  workspace overlay. Do not edit shell profiles automatically. `pixi run` applies
  the selected environment's activation; verify its scripts and resolved tools
  rather than blindly sourcing another ROS installation on top.
- Inspect task commands, working directories, dependencies, and target overrides
  before running them. Task names such as `build`, `test`, and `clean` are not
  guarantees of behavior. Shell helpers must match their execution context;
  a child cmd activation cannot modify its parent PowerShell environment.
- In the selected environment check `ros2 --help`, `colcon --help`, Python and
  compiler resolution, then follow [build and test](../ros2-build/SKILL.md) for
  a scoped build and `colcon test-result --verbose`. Use
  [meaningful testing](../ros2-test/SKILL.md); zero discovered tests is not success.
- Keep build activation on the intended underlay, not the stale overlay being
  rebuilt. Load the completed overlay for runtime checks only after a successful
  build. Guard first-build activation when `install/` does not yet exist.
- On Windows verify MSVC, the Windows SDK, DLL/Python ABI, and a shell-compatible
  setup script; Pixi alone does not install the full Microsoft toolchain. On macOS
  verify the SDK and native architecture; do not mix Rosetta and ARM64 libraries.
- For RDE, inspect `ROS2.rosSetupScript`, `ROS2.distro`, and the machine-specific
  `ROS2.pixiRoot` configuration. A manual `pixi.toml` alone does not select an RDE
  distribution: configure a verified wrapper that activates the chosen Pixi/ROS
  underlay, not just the local install overlay. Do not overwrite an existing
  manual selection or assume the install root is the project directory.

## Diagnose and hand off

Separate mapping/availability failures from solver conflicts, activation failures,
compiler errors, missing installed resources, and runtime DDS/GUI/device issues.
For bootstrap or installation errors use
[installation troubleshooting](../ros2-install-troubleshooting/SKILL.md).
Never bypass TLS, remove dependency declarations, or delete `.pixi`, build outputs,
or logs as a speculative fix. Runtime or network tests need an isolated graph and
an approved simulation/hardware target; successful installation is not runtime proof.

Report the manifest/environment, tool versions, distro/platform/channel choices,
file and lock changes, unresolved dependencies, commands and exit statuses, and
test counts. Distinguish source inspection, solver validation, host build results,
and platforms or hardware not tested.