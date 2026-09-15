# ROS 2 AI agents and skills

The extension bundles eleven agents (a ROS 2 expert coordinator and ten hidden
specialists) and sixteen reusable skills. They provide development workflows, not an autonomous robot
controller or a replacement for validating generated code.

## Getting started

1. Use VS Code **1.110 or newer** with GitHub Copilot Chat enabled and an available
   chat model. Open a trusted ROS 2 workspace.
2. Select **ROS 2 Expert**, the top-level orchestration agent, in the Chat agent
  picker for robot development and platform-specific help.
3. Describe the goal, distro, platform, relevant package, and whether the target
   is simulation or hardware. The coordinator delegates focused work as needed.
4. Type `/ros2-` to discover skills, or ask a relevant question to let Copilot
   load a skill automatically. For example, `/ros2-build` investigates build/test
   failures and `/ros2-actions-services-lifecycle` helps implement node contracts.

All ten specialists, including ROS 2 Simulation, ROS 2 OS, and ROS 2 Test, are subagent-only and hidden from the
agent picker. ROS 2 Expert is the single user-facing entry point and delegates
to the appropriate specialists. Agents use the selected chat model; no model
vendor or subscription tier is pinned.

## Agent inventory

| Agent | Responsibility |
| --- | --- |
| ROS 2 Expert | Coordinates work, delegates, integrates changes, and validates results |
| ROS 2 Core | Owns general launch orchestration; rclcpp/rclpy, executors, actions, services, lifecycle, parameters, TF2, performance |
| ROS 2 Networking | RMW, Fast DDS, Cyclone DDS, Connext DDS, Zenoh, discovery, QoS, security |
| ROS 2 Packaging | colcon/ament, manifests, dependencies, install/export rules, releases |
| ROS 2 MoveIt | MoveIt 2 planning, kinematics, collision scenes, trajectory execution |
| ROS 2 Navigation | Nav2, localization/SLAM, costmaps, planners/controllers, behavior trees |
| ROS 2 Hardware | ros2_control, sensors/drivers, buses, timing, hardware bringup |
| ROS 2 Drones | MAVROS ROS 2, MAVLink, PX4/ArduPilot, SITL, flight telemetry and frames |
| ROS 2 OS | Windows, macOS, Ubuntu, Jetson/JetPack, Pixi, toolchains, compatibility |
| ROS 2 Test | Behavior-based unit/integration tests, regression capture, pytest/GTest, launch_testing, CI results |
| ROS 2 Simulation | Gazebo, Omniverse Isaac Sim, MuJoCo adapters, simulator bringup, bridges, time, and validation |

## Skill inventory

| Skill / slash command | Workflow |
| --- | --- |
| `ros2-development` | Shared development ground rules, platform preflight, hardware safety, and validation |
| `ros2-install-troubleshooting` | ROS 2 installation diagnostics, environment checks, and scoped recovery |
| `ros2-build` | colcon, ament_cmake/ament_python, underlays/overlays, build and test diagnostics |
| `ros2-test` | Requirements-based tests, bug reproductions, regression prevention, isolation, and verified test results |
| `ros2-packaging` | package.xml, rosdep, interfaces, installed resources, release preparation |
| `ros2-actions-services-lifecycle` | Servers/clients, feedback, cancellation, concurrency, transitions and cleanup |
| `ros2-networking` | DDS/RMW discovery, endpoint QoS, multi-host connectivity and security |
| `ros2-debugging` | VS Code ROS 2 launch/attach, symbols, interpreters, source paths |
| `ros2-perception` | Cameras/LiDAR, calibration, TF/time, QoS, recorded datasets and GPU constraints |
| `ros2-manipulation` | MoveIt 2 configuration, planning scenes, controllers and simulation validation |
| `ros2-navigation` | Nav2 bringup, localization, sensor/costmap diagnosis and navigation tests |
| `ros2-gazebo` | Modern Gazebo integration, ROS bridges, simulation time, and Gazebo Classic migration distinctions |
| `ros2-omniverse` | NVIDIA Omniverse Isaac Sim ROS 2 bridge, USD scenes, sensors, and version compatibility |
| `ros2-mujoco` | MuJoCo models, stepping, control, and integration through an explicit ROS adapter |
| `ros2-launch` | Launch arguments, substitutions, includes, namespaces, composition, event handling, and installed launch resources |
| `ros2-performance` | Evidence-based profiling, latency/throughput measurements, executors, tracing, and regression comparisons |

### Launch, simulation, and performance

ROS 2 Core owns general launch orchestration through `/ros2-launch` and uses
`/ros2-performance` for measurement-driven optimization. ROS 2 Simulation handles
simulator-specific bringup and integration, using those shared workflows and
`/ros2-test` for validation rather than duplicating launch ownership.

Choose the simulator workflow explicitly: `/ros2-gazebo` distinguishes modern
Gazebo (`gz`, `ros_gz`) from legacy Gazebo Classic (`gazebo`, `gazebo_ros`); their
plugins and APIs are not interchangeable. `/ros2-omniverse` targets NVIDIA
Omniverse Isaac Sim and its version-compatible ROS 2 bridge, not a generic
Omniverse application. `/ros2-mujoco` requires an explicit ROS adapter; MuJoCo
alone does not provide ROS topics, services, TF, or simulation clock integration.

### Meaningful tests and regression prevention

Use `/ros2-test` to design tests from requirements, public behavior, and realistic
failure modes—not to duplicate the implementation or inflate coverage totals.
The testing specialist explains what defect each test catches and how its expected
outcome was established. It covers pytest/unittest, GTest, and ROS 2 launch/integration
tests, with deterministic execution, isolation, and cleanup.

Whenever a bug is discovered, agents capture a minimal reproducer in a permanent
regression test, demonstrate failure before the fix and success afterward when
feasible, and retain the test in the discovered suite/CI. Blocked automation and
remaining risks must be reported, not hidden behind a passing build or skipped test.

## Tools, platforms, and safety

Static code help does not require ROS or the MCP server to be running. The bundled
agents enable built-in file, search, web, and execution tools; the coordinator also
enables delegation. Runtime CLI checks require a correctly configured ROS 2
environment. You can enable available ROS 2 MCP tools in Chat for runtime
introspection; the existing **ROS2: Start MCP Server** command is separate from
agent/skill registration. Missing tools must be reported rather than assumed.

The installer's Copilot-help clipboard workflow loads `ros2-install-troubleshooting`
and includes `ros2-development` inline with diagnostic context. It does not depend
on Chat automatically discovering the skills or resolving pasted relative links.

The OS expert checks the execution host, shell, architecture, distro, and package
availability. Jetson guidance includes JetPack/L4T and CUDA compatibility; it does
not assume that Jetson can use arbitrary Ubuntu upgrades. Not every ROS package
supports every platform. These chat contribution points target VS Code; support
in Cursor and other clients depends on that client's customization capabilities.

All agents and task-specific skills load the shared `ros2-development` skill. They default to
static inspection, mocks, or isolated simulation and require explicit approval
before live parameter changes, lifecycle/controller activation, motion, or drone
operations. Bag playback can publish commands, and a breakpoint can stall a
control loop. Keep tool confirmations enabled and use real safety interlocks and
operator supervision. Prompt instructions alone cannot enforce hardware safety.

## Troubleshooting discovery

- Confirm the installed extension version includes these assets and reload the
  window after updating it. Merely editing this repository does not update your
  installed extension; use an Extension Development Host or install a new VSIX.
- Check that Copilot Chat and custom agents/skills are enabled by your settings
  and organization policy. Inspect Chat customization diagnostics for load errors.
- Expect only **ROS 2 Expert** in the picker; all ten specialists are
  intentionally available only through subagent invocation. Delegated activity
  may still appear in Chat; hiding agents from the picker does not hide execution.
- If delegation is unavailable, invoke the matching skill directly. No automatic
  changes to user settings, workspace `.github` files, or MCP configuration occur.

## Extending the collection

Sources live in `assets/agents/*.agent.md` and `assets/skills/<name>/SKILL.md`, with
shared ground rules in `assets/skills/ros2-development/SKILL.md`.
The `chatAgents` and `chatSkills` entries in
`package.json` register them directly, following the URDF editor's declarative
approach. ROS 2 already ships `assets/` in its VSIX, so no webpack copy is needed.

When adding a specialist, give it a unique name and focused description, register
its path, and add its exact name to the coordinator's `agents` list. Keep
specialists non-recursive and set `user-invocable: false` while leaving
`disable-model-invocation: false`. For skills, match the directory name to frontmatter
`name`, add a manifest entry, and include a concrete workflow and validation.
Preserve relative links so everything works from an installed extension.

Run the tests in `test/suite/ai-customizations.test.ts` after changes. They verify
the inventory, frontmatter, delegation allowlist, local-link reachability, and
VSIX file selection. These structural checks do not prove runtime delegation,
agent behavior, or simulator correctness; those require execution-based validation.