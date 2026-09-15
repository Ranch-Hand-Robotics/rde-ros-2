---
name: ros2-omniverse
description: "Use when integrating ROS 2 with NVIDIA Omniverse-based Isaac Sim: verify Isaac Sim SDK/runtime, ROS bridge and platform/GPU compatibility, USD/URDF articulations, OmniGraph or API execution, sensors, joint commands, simulation clock/reset, and isolated reproducible tests; distinguish generic Omniverse and Isaac Lab training."
user-invocable: true
---

# ROS 2 NVIDIA Omniverse / Isaac Sim

Read [common platform and safety rules](../ros2-development/SKILL.md) first;
stop execution if unavailable. Default to isolated scenes and mock ROS consumers.
Require approval for hardware operations, installations, or GPU/system changes.

## Identify the actual product and supported stack

1. Establish whether the task uses Isaac Sim, a generic Omniverse/Kit application,
   or Isaac Lab. Omniverse alone does not establish an Isaac Sim ROS bridge;
   Isaac Lab training, policies, and vectorized environments are not that bridge.
2. Record Isaac Sim release, Kit/SDK and Python runtime, bridge version, ROS distro,
   RMW, host OS/architecture, GPU/VRAM/driver, and workstation/container/headless mode.
3. Consult the release-matched [requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
   and [ROS setup](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html).
   Verify supported combinations before naming extension IDs, API imports, graph
   node IDs, package/install flags, or launchers. Do not promise every GPU/OS works.
4. Inspect whether the bridge loads bundled ROS libraries or a sourced external
   installation. Verify Python/native ABI and custom-message compatibility; never
   repair import errors by mixing arbitrary system packages into the runtime.

## Minimal scene, then ROS

1. Use a versioned local USD scene with ground and one simple articulation. Record
   asset dependencies and importer settings; resolve missing references separately
   from physics errors. Keep expensive rendering and randomization out initially.
2. Verify physics-scene and articulation-root configuration with ROS disconnected.
   Initialize the selected runtime correctly, start the timeline, and advance a
   bounded number of physics steps. Expect finite poses and correct gravity/contact.
3. Enable the installed ROS bridge using its documented lifecycle. Use a minimal
   OmniGraph or supported API publisher, with one simulation clock and joint state
   output. Check graph execution connections/ticks, ROS context, target prim paths,
   and actual bridge startup logs; extension enablement alone proves no data flow.
4. Observe stamped ROS joint states before accepting any command. Add one bounded
   simulated joint command, then one sensor with a known calibration target.
   Expect named-joint motion and sensor measurements consistent with the fixture.
5. Integrate application nodes only after this works; use
   [launch orchestration](../ros2-launch/SKILL.md) for readiness and cleanup, and
   [networking](../ros2-networking/SKILL.md) for discovery/QoS and container boundaries.

## Validate import and physical meaning

- Inspect USD stage units/up axis and every conversion to ROS metres/radians;
  distinguish USD angular attributes from API/ROS units. Verify base, joint, world,
  and camera optical frames, quaternion ordering, and one authority per TF edge.
- Compare URDF source with the imported articulation: fixed versus floating base,
  joint axes/order/names, limits, mimic behavior, mass/inertia, and collision proxies.
  Photorealistic materials do not establish correct contacts or sensor physics.
- Calibrate drive type, stiffness/damping, effort/velocity limits, zero offsets,
  and transmission reduction/sign. Do not assume URDF transmissions were imported
  or that position and effort interfaces are interchangeable. Avoid competing
  internal drives and external controllers; prove one-joint direction and scale.
- Separate physics timestep, control period, rendering cadence, and sensor rate.
  Validate image intrinsics, depth units, frame IDs and acquisition stamps against
  a known scene, not visual appearance alone; document idealized noise/latency.

## Clock and episode lifecycle

- Publish one simulator-derived `/clock`; enable `use_sim_time` on consumers and
  avoid bag-clock or second-scene competition. Timestamp data at acquisition.
- Check pause versus stop: simulation time freezes while paused without stepping,
  but the selected stop/reset policy may restart or preserve published time.
  Record the observed policy rather than assuming timeline reset resets ROS time.
- On reset, restore initial articulation and controller state, discard queued
  commands, and reinitialize invalid runtime handles when the API requires it.
  Handle backward time jumps in TF/ROS consumers; bound waits by wall time.

## Fault isolation and verification

- No ROS samples: first inspect timeline/graph execution and bridge load errors;
  then ROS context, message types, domain/discovery and QoS. Missing custom types
  with working standard types suggests interface build/runtime ABI, not physics.
- State arrives but motion is wrong: compare prim target, joint-name mapping,
  drive mode and degree/radian conversion. Black images with valid joint states:
  inspect render product, camera, renderer readiness, and GPU support separately.
- Follow [meaningful testing](../ros2-test/SKILL.md): independent expected poses,
  distances and limits; pinned scene/runtime, seeds, fixed steps and wall deadlines.
  Test pause/reset, stale-command rejection and shutdown using isolated simulation,
  never live robots. Do not promise bitwise GPU reproducibility across platforms.
- Preserve discovered bugs as permanent regressions; show failing/passing evidence
  where feasible and state coverage blockers. For throughput or latency use
  [runtime performance](../ros2-performance/SKILL.md), not unmeasured physics changes.
  Report tested stack, bridge evidence, checks run and sim-to-real limitations;
  scene fidelity and successful training do not demonstrate physical safety.