---
name: ros2-mujoco
description: "Use when connecting ROS 2 to MuJoCo: official runtime and MJCF/URDF import, evidenced third-party/custom ROS adapters, actuator and transmission calibration, ctrl versus joint targets, qpos/qvel freejoint layouts, sensor/TF mappings, fixed-step clocks, pause/reset, and deterministic bounded regression tests."
user-invocable: true
---

# ROS 2 MuJoCo

Read [common platform and safety rules](../ros2-development/SKILL.md) first;
stop execution if unavailable. Use mocks or isolated simulation, not live robots.
Require approval before hardware access, installations, or system modifications.

## Verify runtime and integration evidence

1. Identify the official MuJoCo runtime/bindings version, language/interpreter,
   OS/architecture, model format, ROS distro and selected integration package.
   Distinguish official `mujoco` bindings from legacy `mujoco-py` or other backends;
   verify their own support matrices rather than assuming interchangeable APIs.
2. MuJoCo physics is not a built-in native ROS 2 bridge. Inspect the actual
   third-party adapter repository/version, package manifest, launch files, source,
   message contracts and tests, or identify the project's custom adapter explicitly.
   If none exists, agree on an adapter design; do not invent package/plugin names.
3. Define who owns stepping, control updates, ROS execution and state snapshots.
   Record supported command/state interfaces, QoS, timestamps and reset behavior;
   a claimed ros2_control integration needs evidence of its actual plugin contract.

## Start with a simulator-only fixture

1. Use the release-matched [modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html).
   MJCF is native; URDF import represents a subset and has different compiler
   defaults. Expand Xacro first and resolve meshes; inspect the compiled model,
   since URDF import does not establish working actuators, sensors or ROS plugins.
2. Preserve the source model and conversion settings. Add MJCF-specific actuators,
   sensors and contacts deliberately, keeping regeneration separate from additions.
   Check visual/collision retention, fixed-body fusion, mass/inertia and joint limits.
3. Compile a minimal MJCF fixture with ground and one hinge/slide actuator. Step
   without ROS or rendering for a fixed count; expect finite state, advancing
   `data.time`, and a mechanically reasoned response with no numerical warnings.
4. Connect the evidenced adapter: first joint-state/clock output, then one bounded
   command, then sensors. Only after that add application nodes with
   [launch orchestration](../ros2-launch/SKILL.md); isolate ROS discovery endpoints.

## Map state, controls, and frames explicitly

- Resolve names to compiled IDs and use `jnt_qposadr` and `jnt_dofadr`, not joint
  enumeration as array offsets. `qpos` has length `nq`; `qvel` has length `nv`.
  A freejoint uses seven positions (XYZ plus WXYZ quaternion) and six velocities
  (linear plus angular); a ball joint uses four positions and three velocities.
  Do not copy quaternion components into JointState angles or differentiate them
  as ordinary scalars. Convert WXYZ to ROS XYZW and verify velocity reference frames.
- Read the selected version's [state and actuation semantics](https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework).
  `data.ctrl` contains actuator inputs, not necessarily joint angles or torques.
  Inspect actuator type, transmission target, gear/sign, gain/bias and activation
  dynamics; position servos target transmission coordinates, motors generate force.
- Calibrate zero offsets and reduction with a small known input and independently
  expected motion/effort. Map actuator/control addresses from the compiled model;
  do not assume one actuator per joint, especially with tendons or coupled drives.
  Check joint range, control and force limits separately, plus adapter speed limits.
- Establish consistent SI units and MJCF compiler angle settings; compiled angular
  state is radians. Check body/site/sensor frames and camera-to-ROS optical rotation.
  Validate inertia, friction and collision geometry independently of visible meshes;
  ordinary mesh collision uses convex hulls, so concavity needs deliberate modeling.

## Stepping, clock, pause, and reset

- Give one loop ownership of stepping with a fixed timestep and explicit control
  decimation. Serialize commands/reset against stepping; snapshot coherent state
  for ROS rather than allowing callbacks to race writes to shared simulation data.
- Derive the sole ROS `/clock` from simulation time and set `use_sim_time` on
  consumers. Align sensor stamps with the state actually evaluated: derived fields
  can lag the integrated state after a step; refresh required stages deliberately.
- Pause stops stepping/time advancement, not the wall-time watchdog. Specify
  whether queued commands expire while paused; never replay stale motion on resume.
- Reset simulation data/keyframe and controller/adapter buffers, including activation
  and applied inputs; recompute derived state before publication. Handle backward
  time jumps in TF consumers and record whether episode time restarts or is offset.

## Diagnose and prove the contract

- Wrong joint after adding a floating base: inspect qpos/qvel addresses, not QoS.
  Positive command moves backward: inspect transmission sign and actuator mode.
  Stable native fixture but frozen ROS state: inspect adapter scheduling, snapshot
  publication and [networking](../ros2-networking/SKILL.md), not solver tuning.
- Follow [meaningful testing](../ros2-test/SKILL.md): pin model/runtime, initial
  state, seeds and timestep; bound step counts and wall duration. Test known hinge
  response, floating-base mapping, saturation, stale inputs, reset and shutdown.
  Expectations must come from independent mechanics/interface requirements, not
  the adapter under test. Exact repeatability is not promised across versions/OSes.
- Retain permanent regressions for discovered bugs with before/after evidence when
  feasible; report unavailable adapters/runtime and coverage gaps. Use
  [runtime performance](../ros2-performance/SKILL.md) for profiling after correctness.
  Return integration provenance, mappings, actual checks and model-fidelity limits;
  passing simulated contacts or control tests does not establish hardware safety.