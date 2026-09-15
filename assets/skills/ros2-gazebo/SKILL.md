---
name: ros2-gazebo
description: "Use when integrating or diagnosing ROS 2 with Gazebo: current gz/ros_gz versus Gazebo Classic/gazebo_ros, distro pairing, SDF/URDF models, simulator plugins, ros2_control, sensors, bridge types/directions/QoS, simulated time, pause/reset, and deterministic isolated regression tests."
user-invocable: true
---

# ROS 2 Gazebo

Read [common platform and safety rules](../ros2-development/SKILL.md) first;
stop execution if unavailable. Use test-owned simulation, never live robot topics.
Obtain approval before hardware operations or system/package/driver changes.

## Establish the version contract

1. Record ROS distro, Gazebo release/library majors, ros_gz version, physics
   backend, OS/architecture, and native/container execution context.
2. Check the [official pairing guide](https://gazebosim.org/docs/latest/ros_installation/)
   against the installed release documentation. Prefer that distro's supported
   pairing; do not replace it with the newest Gazebo or mix incompatible binaries.
3. Distinguish current Gazebo (`gz`, with older Ignition naming in some releases)
   and `ros_gz` from Gazebo Classic (`gazebo`) and `gazebo_ros`. Classic plugins,
   launch arguments, and service schemas are not interchangeable with current ones.
4. Inspect package manifests and model plugin declarations. Verify actual library
   filenames, exported classes, ABI, search paths, and matching examples before
   adding plugins. Distinguish `gz_ros2_control` from Classic `gazebo_ros2_control`.

## Prove the smallest world before ROS integration

1. Use a local minimal SDF world with ground and one simple articulated model;
   pin resource versions and resolve meshes/includes without unbounded downloads.
   Inspect URDF-to-SDF conversion for fixed-joint merging and plugin placement.
2. Start paused in an isolated environment with no hardware drivers. Confirm
   model creation, finite poses, physics-system loading, and bounded manual steps.
   A visible GUI alone does not prove that server physics or sensors are running.
3. Add one sensor or control system. Inspect its native Gazebo Transport output:
   advancing simulation stamps and a known scene measurement should be observable.
   If native data is missing, fix the world/system/sensor before changing ROS QoS.
4. Add only the required `ros_gz_bridge` mappings. Record Gazebo and ROS names,
   message types, direction, QoS, and supported conversion for each endpoint.
   Verify world-scoped names; avoid bidirectional clock or command feedback loops.
5. Expect the same physical measurement and simulation stamp on the ROS side.
   Only then add robot_state_publisher, controllers, and the application launch;
   use [launch orchestration](../ros2-launch/SKILL.md) for readiness and shutdown.

## Calibrate model and control fidelity

- Check metres/radians, gravity, link/joint frames, sensor optical frames, masses,
  inertia tensors, collision geometry, contact friction, and initial penetration.
  Rendering a mesh correctly does not validate its collision shape or dynamics.
- Map joint names and axes to command/state interfaces; verify limits, zero offsets,
  transmission reductions/signs, mimic joints, and position/velocity/effort modes.
  Use one small bounded simulated input: expected direction and scale must match
  the mechanical specification, not merely the controller's reported command.
- For a wheel fixture, compare known wheel radius and rotation with displacement;
  separate slip/contact effects from incorrect radius or transmission calibration.
  For a sensor, use a target at a known distance and verify TF, range, and units.

## Clock, pause, and reset

- Use one simulator-derived ROS `/clock` publisher, bridged Gazebo-to-ROS only;
  inspect the actual clock topic and set `use_sim_time` on all relevant ROS nodes.
- While paused, simulation time must not advance without explicit steps. Use
  wall-clock readiness/watchdog deadlines so a paused world cannot hang a test.
- Distinguish resetting model poses from resetting time/world state using the
  installed service contract. On a backward time jump, reset affected TF buffers,
  controller integrators and queues; discard pre-reset commands before resuming.

## Isolate failures and retain meaningful tests

- Plugin load error: compare library/class/ABI and resource paths before tuning
  physics. Native sensor data but no ROS samples: inspect bridge conversion,
  direction, resolved topic, discovery, then QoS with [networking](../ros2-networking/SKILL.md).
- TF extrapolation after reset: inspect clock authority and consumer reset handling,
  not just transform tolerance. Exploding joints: inspect inertia, overlaps, control
  sign/gain and timestep independently; do not disable limits to obtain a pass.
- Follow [meaningful testing](../ros2-test/SKILL.md): derive assertions from an
  independent contract, fix seed/world/initial state/step size, bound steps and wall
  time, and clean up only test-owned processes. A domain ID alone is not isolation.
- Test known sensor range, bounded joint response, command expiry, pause/step,
  reset recovery, and shutdown. Require numerical tolerances justified by the
  fixture; retain permanent regressions for discovered bugs with before/after
  evidence where feasible. Report blocked tests rather than claiming coverage.
- Record real-time factor and sensor/control rates; use
  [runtime performance](../ros2-performance/SKILL.md) before optimizing throughput.
  Report versions, native-versus-ROS evidence, executed checks, and fidelity gaps;
  simulated success does not certify hardware safety or other platforms.