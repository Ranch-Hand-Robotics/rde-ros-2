---
name: ros2-manipulation
description: "Use when developing ROS 2 manipulation with MoveIt 2: SRDF, planning scenes, IK, planners, trajectories, ros2_control, joint limits, collision checking, simulation, and safe execution validation."
user-invocable: true
---

# ROS 2 Manipulation

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, shell, architecture, ROS distribution/version, MoveIt 2 version, and active native, Pixi, container, or remote environment.
- Verify compatible ROS, simulator, planner, and controller tooling; locate shell-compatible setup scripts rather than assuming paths.
- Do not silently switch environments or run incompatible platform shims; check plugin availability for this platform/version.
- Confirm the approved simulation versus hardware target, controller endpoint, and whether execution is authorized separately from planning.
- Do not operate a live robot, arm, or drone without explicit approval; default to offline planning or approved simulation.

## Validate the robot model
1. Inspect URDF/Xacro, SRDF, joint names/types, frames, planning groups, end effectors, and passive or mimic joints.
2. Check SRDF virtual joints, named states, and disabled-collision pairs against the actual robot configuration.
3. Verify collision geometry, tool geometry, payload assumptions, and joint-state coverage; visual geometry alone is insufficient.
4. Reconcile position, velocity, acceleration, and applicable jerk/effort limits across the model, planner, and controllers.
5. Treat missing or contradictory limits as a blocker; never increase limits simply to make a plan succeed.

## Configure planning
1. Verify the current state is fresh, complete, and in the planning frame using TF and joint-state evidence.
2. Select an installed IK plugin and confirm group topology, solver parameters, timeout, and seed-state behavior.
3. Choose an available planner/pipeline appropriate to the task and inspect its version-specific configuration schema.
4. Populate the planning scene with obstacles, attached objects, touch links, and the correct world-to-robot transform.
5. Confirm scene updates have arrived before planning; stale scenes can produce apparently valid but unsafe paths.
6. Set explicit goal tolerances, workspace bounds, constraints, planning time, and velocity/acceleration scaling.
7. Inspect success/error codes and trajectory completeness; do not execute a partial Cartesian path as a complete task.
8. Validate collision clearance and limits along the trajectory, including time parameterization, not only at the final pose.

## Integrate ros2_control
- Inspect hardware interfaces, controller manager, controller states, joint mappings, and command/state interface compatibility.
- Match trajectory action endpoints and joint ordering to the selected controller; namespaces must be verified, not guessed.
- Ensure only the intended controller owns command interfaces; avoid competing controllers or test publishers.
- Check trajectory timing, start-state tolerance, feedback, execution monitoring, and goal/cancel behavior.
- Reject execution on stale joint states, unavailable controllers, unexpected hardware interfaces, or unverified clock behavior.

## Simulation-first execution
1. Use an approved simulator/fake hardware setup isolated from live controllers and verify target identity immediately before sending goals.
2. Run plan-only tests, inspect the full trajectory, and then execute conservatively within the approved simulated limits.
3. Exercise collision obstacles, unreachable IK goals, singularities, limit violations, stale scenes, and missing transforms.
4. Test execution rejection, controller loss, timeout, cancellation, and recovery without automatically resending motion commands.
5. Verify stopped motion and terminal action state after cancellation; request acceptance alone is not evidence of stopping.
6. Hardware execution requires explicit approval, verified workcell clearance, an operator, and an available emergency-stop procedure.
7. Never disable collision checking, safety interlocks, or protective stops to bypass a failing test.

## Validation and handoff
- Record model/SRDF versions, solver/planner configuration, scene contents, controller mappings, and limits used.
- Report plan validity, simulated execution results, cancellation evidence, and remaining hardware-only checks.
- A successful plan or simulation is not a certification of safe physical execution.
