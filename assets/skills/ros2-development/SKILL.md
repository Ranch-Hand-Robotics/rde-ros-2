---
name: ros2-development
description: "Use before ROS 2 development, builds, diagnostics, or robot operations to establish the OS, shell, architecture, distro, workspace, available tools, validation plan, and hardware safety boundaries."
user-invocable: true
---

# ROS 2 development ground rules

Use this preflight workflow before using a bundled ROS 2 agent or domain skill.
Establish the environment, inspect evidence, agree on safe validation, then report
results. These rules are not a substitute for tool permissions or a robot safety
controller.

## Establish the environment

- Identify the workspace/package, ROS 2 distribution and version, host OS, active
  shell, CPU architecture, language, and deployment target. Distinguish the local
  editor from its SSH, container, WSL, or remote robot environment.
- Check compatible executables before invoking them. Source the detected underlay
  and then the intended overlay using scripts for the active shell. Do not assume
  a distribution, installation path, system Python, or that a setup script run in
  a child shell changes its parent environment.
- Use Bash/Zsh syntax only in those shells. For Windows, distinguish PowerShell
  from cmd.exe, verify native executables, and never invoke extensionless Unix
  shims through file associations. Explain and obtain agreement before switching
  to WSL, a container, or another execution environment.
- Inspect only relevant environment variables; redact credentials and sensitive
  robot/network identifiers from reports. Never request passwords through chat.
- Use ROS 2 APIs for the installed distribution, not ROS 1 recipes. Verify version
  dependent CLI flags, plugin names, QoS settings, and config schemas against
  installed help/source or matching official documentation. State uncertainty.

## Work from evidence

- Read project instructions, manifests, launch files, and tests before edits.
  Preserve existing conventions and make the smallest change that solves the task.
- When a bug is discovered, capture its reproducer in a permanent regression test
  within the authorized scope. Assert intended behavior, demonstrate failure
  before the fix and success afterward when feasible, and retain the test in the
  discovered suite. Use [meaningful testing](../ros2-test/SKILL.md); report blocked
  automation and remaining regression risk rather than omitting coverage silently.
- Discover tools actually available in the current session. If the ROS 2 MCP server
  is available and enabled, prefer read-only introspection first. Otherwise use
  verified ROS 2 CLI tools or static inspection; do not invent MCP tool names or
  claim runtime observations from static files.
- Confirm exit status immediately and inspect the intended output/artifacts.
  Distinguish checks actually run from recommendations or blocked checks.
- Bound diagnostic sampling, topic recording, and tracing by time/data size.
  Stop only processes started for this task; do not terminate unrelated ROS nodes.
- Keep build dependencies reproducible. Respect the project's package manager;
  use Pixi where appropriate for Windows/macOS rather than assuming Linux APT.
  Do not mix system and environment-managed Python/native libraries or use vcpkg.

## Hardware and system changes

- Determine whether the target is simulation, a recorded dataset, or physical
  hardware before running launches, tests, publishers, services, or actions.
- Default to static checks, mocks, or isolated simulation. Ask for explicit approval
  before changing live parameters, lifecycle states, controllers, actuator commands,
  flight modes, or running any launch/test that could move hardware.
- Treat bag playback as publishing: isolate it from live command topics. Confirm
  namespaces, domain isolation, target identity, limits, emergency stop, and a
  responsible operator before any approved hardware test.
- Never bypass collision checks, limits, watchdogs, failsafes, security policies,
  or flight arming checks to make a test pass. Simulation success is not proof of
  hardware safety. Drone work defaults to SITL, with no arming or takeoff by default.
- Explain the impact and request approval before installing system packages,
  changing firewalls, device permissions, drivers, kernels, or boot configuration.
  Never flash firmware, upgrade JetPack, or run privileged operations implicitly.

## Report

Return the environment and evidence, diagnosis or changes, validation performed,
remaining uncertainty, and the next safe step. When delegated, stay within the
assigned scope and list changed files; do not delegate again or edit another
specialist's files without the coordinator's agreement.