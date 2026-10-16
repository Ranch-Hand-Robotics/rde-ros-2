---
name: ros2-launch
description: "Use when creating, reviewing, or debugging ROS 2 launch files: Python/XML/YAML, includes, substitutions, typed parameters, namespaces, remaps, composition, lifecycle readiness, startup ordering, shutdown, and launch_testing regressions."
user-invocable: true
---

# ROS 2 launch files

Read and follow the [development ground rules](../ros2-development/SKILL.md)
before work. ROS 2 Core owns launch orchestration; simulator-specific plugins and
bridges belong to the simulation specialist. Use the project's installed distro
and launch APIs, not ROS 1 syntax or examples from an incompatible release.

## Establish the launch contract

1. Identify the entry point, package/build type, installed launch resources,
   supported arguments, expected graph, and hardware versus simulation target.
   Read included files and parameter YAML before running anything. Python launch
   construction can execute arbitrary code: even `--show-args` is not a sandbox.
2. Define expected node/component names, namespaces, topic/service/action remaps,
   parameter values and types, readiness conditions, and failure/shutdown policy.
   Separate launch arguments (configuration inputs) from ROS node parameters.
3. Keep Python, XML, or YAML according to project conventions and installed
   frontend support. Choose Python for needed programmatic orchestration, not
   merely to rewrite working declarative files. Check exact distro API signatures.

## Implement predictable configuration

- Declare and document public arguments, defaults, valid choices, and units.
  Pass arguments explicitly through includes; avoid accidental dependency on
  inherited launch configuration. Use scoped groups for namespace/config changes.
- Use installed package-share lookup and path substitutions such as
  `FindPackageShare` and `PathJoinSubstitution` where supported. Do not depend on
  the current directory or a developer's source checkout. Ensure launch/config
  resources are installed by ament CMake or Python package data rules; use
  [packaging](../ros2-packaging/SKILL.md) for install/export problems.
- Substitutions are evaluated in launch context, not ordinary Python strings.
  Do not apply Python `bool()` to a `LaunchConfiguration` or to the string
  `"false"`. Use launch conditions for actions and explicit parameter typing
  (for example `ParameterValue(..., value_type=bool)` where supported) for nodes.
  Preserve literal strings that YAML might interpret as booleans/numbers.
- Check parameter-file node selectors after namespacing, override precedence,
  remap scope, absolute names, and component-specific parameter/remap propagation.
  Namespace isolation alone does not isolate TF frame IDs; configure them too.
- Use substitutions and argument lists instead of shell-concatenated commands.
  Inspect any `Command`/process substitutions and their quoting. Avoid unnecessary
  `OpaqueFunction`; when needed, resolve context there and return explicit actions.
- For composition, verify plugin availability, container name/namespace, executor
  choice, component loading errors, and shutdown behavior. Do not assume loading
  a component means its services or controller are ready.

## Readiness, clocks, and shutdown

1. Start dependencies using observable readiness: required service availability,
   valid initial data/TF, or confirmed lifecycle state. Process-start events only
   mean the process started; they do not prove ROS/application readiness.
2. Register event handlers before events can be emitted. Handle unsuccessful
   process exits and failed lifecycle transitions explicitly; do not start a
   dependent action on every exit regardless of its return code.
3. Avoid arbitrary `TimerAction` delays as readiness checks. Use bounded waits
   with actionable failures and a wall/steady-clock watchdog so absent or paused
   simulation time cannot hang startup or tests. Verify the chosen API's clock.
4. Coordinate lifecycle transitions with the project's lifecycle manager or
   explicit state/event checks. Use the
   [actions/services/lifecycle workflow](../ros2-actions-services-lifecycle/SKILL.md).
5. Propagate `use_sim_time` to all relevant nodes/components. Identify the single
   `/clock` authority; test absent, paused, and reset clocks. Keep simulator
   stepping/reset details in its own skill.
6. Define essential versus optional process failure behavior. Limit restart
   policies; avoid respawn loops that hide crashes or re-enable commands. On
   cancellation, startup failure, and shutdown, stop child processes and release
   resources. Never activate live hardware as a launch-inspection step.

## Validate observable outcomes

- First check parsing and installed resource discovery in a safe environment;
  then run an isolated bounded `launch_testing` test using the installed API.
  A launch-description object or successful process spawn alone is insufficient.
- Query the actual graph and effective parameters, and exercise a small public
  interface. Verify a namespaced instance does not leak endpoints into the root
  or interfere with a second instance. Assert types as well as parameter values.
- Cover disabled optional nodes, missing dependencies, failed startup, clean
  shutdown and child exit codes, plus relevant composed/non-composed modes.
  Use post-shutdown assertions where appropriate; always clean up on test failure.
- Retain a regression whenever a bug is discovered. Examples: a false-like
  argument wrongly enables a node, an include leaks namespace state, or a missing
  clock stalls cleanup. Establish expected behavior independently from the fix
  and demonstrate failure before/success after when feasible.
- Follow [testing](../ros2-test/SKILL.md). Report the exact launch command,
  arguments, environment, observed contract, cleanup result, and untested modes.
  For startup latency or callback bottlenecks, use
  [performance](../ros2-performance/SKILL.md).