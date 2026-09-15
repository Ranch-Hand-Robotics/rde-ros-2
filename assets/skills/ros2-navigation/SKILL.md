---
name: ros2-navigation
description: "Use when developing ROS 2 Nav2 navigation: map odom base_link TF, lifecycle, costmaps, sensors, planner, controller, behavior trees, localization or SLAM, simulation clock, obstacle and cancellation tests."
user-invocable: true
---

# ROS 2 Navigation with Nav2

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, shell, architecture, ROS distribution/version, Nav2 version, and active native, Pixi, container, or remote environment.
- Locate compatible ROS, simulator, localization/SLAM, and Nav2 tools before use; discover shell-compatible setup scripts.
- Do not assume installation paths, silently change environments, or execute incompatible platform shims.
- Confirm the approved simulation versus hardware target, navigation action endpoint, and command-velocity destination.
- Do not operate a live robot, arm, or drone without explicit approval; default to offline configuration or approved simulation.

## Establish transforms and time
1. Verify the `map -> odom -> base_link` chain or documented equivalent frame names, plus all sensor transforms.
2. Identify the authority for each transform; eliminate competing localization/SLAM publishers of the same transform.
3. Check odometry continuity, units, covariance, timestamp freshness, and robot-relative velocity conventions.
4. Choose localization against a known map or a SLAM workflow; confirm map resolution, origin, frame, and initial pose requirements.
5. Use one clock authority and consistent `use_sim_time`; verify `/clock` advances before diagnosing transform timeouts.
6. Investigate timestamp jumps and stale sensor transforms instead of merely increasing TF tolerances.

## Configure Nav2 components
1. Validate parameter names and installed planner, controller, smoother, and behavior-tree plugins against the actual Nav2 version.
2. Match planner/controller selection to robot kinematics, turning constraints, velocity limits, and stopping behavior.
3. Verify footprint or radius against physical collision geometry, including carried loads and extensions.
4. Configure global and local costmap frames, resolution, update rates, bounds, and rolling/static behavior deliberately.
5. Check obstacle/voxel observation sources, topic types, QoS, ranges, heights, marking, clearing, and sensor persistence.
6. Set inflation and obstacle handling using required clearance; do not suppress obstacles merely to allow a route.
7. Inspect behavior-tree goals, replanning, recovery, and cancellation behavior; recovery actions can themselves move the robot.
8. Verify command message type and topic for the installed version, plus velocity smoother and safety-monitor routing.

## Lifecycle and bringup
- Inspect lifecycle-manager node lists, namespaces, autostart policy, bonds, and dependency ordering.
- Configure and activate only after TF, clock, odometry, sensor inputs, and localization are ready.
- Query actual states and transition results; repeated forced activation can hide underlying configuration failures.
- Keep output isolated from live base controllers during bringup; namespace/domain separation alone must be verified in practice.
- Validate that deactivation, shutdown, and controller failure stop command output appropriately.

## Approved simulation tests
1. Launch only the approved isolated simulation and verify the goal server and velocity consumer identities before sending a goal.
2. Test a reachable goal, confirm path generation, and compare tracking, clearance, progress, and final pose tolerances.
3. Insert representative static/dynamic obstacles and verify costmap updates, replanning, and safe stopping.
4. Test blocked routes, unreachable goals, localization loss, missing sensors, stale odometry, and paused/jumping simulation time.
5. Cancel during planning, following, and recovery; verify terminal action status and actual simulated base stop.
6. Exercise timeout and server/controller loss; do not automatically resend goals when the outcome is unknown.
7. Check command-velocity arbitration and watchdog behavior, including any residual command after navigation shutdown.
8. Hardware tests require separate explicit approval, a clear operating area, an operator, and a verified emergency-stop procedure.

## Validation and handoff
- Record map/localization mode, frame authorities, clock configuration, lifecycle states, plugin versions, and costmap settings.
- Report success/failure/cancellation cases with path, tracking, obstacle-clearance, and stopping evidence.
- Separate simulation results from unvalidated hardware behavior; never bypass collision or safety monitoring to pass a test.
