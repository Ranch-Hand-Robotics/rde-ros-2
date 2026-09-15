---
name: ros2-perception
description: "Use when developing ROS 2 perception pipelines: camera, lidar, point clouds, image_transport, calibration, TF, timestamps, synchronization, sensor QoS, GPU or Jetson acceleration, and recorded-data tests."
user-invocable: true
---

# ROS 2 Perception

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, shell, architecture, ROS distribution/version, active environment, and local versus remote sensor access.
- Verify compatible ROS, bag, image/point-cloud tools, and any GPU runtime binaries before invoking them.
- Discover shell-compatible setup scripts; do not assume installation paths, silently switch environments, or execute incompatible shims.
- Confirm the approved simulation versus hardware target; prefer recorded inputs with outputs isolated from robot control.
- Do not operate a live robot, arm, or drone without explicit approval, including starting attached sensor/actuator drivers.

## Establish the data contract
1. Inventory camera images, `CameraInfo`, lidar scans, `PointCloud2`, detections, and downstream consumers.
2. Inspect actual topic types, frame IDs, encodings, dimensions, units, point fields, and expected rates.
3. Record sensor calibration provenance and distinguish raw versus rectified images and registered versus unregistered depth.
4. Check camera intrinsics, distortion model, image resolution, and camera-to-body extrinsics for consistency.
5. Confirm optical-frame conventions and lidar mounting transforms; do not fix coordinate errors with unexplained sign flips.

## Build the pipeline
1. Verify TF connectivity at each message timestamp, including static sensor transforms and moving-platform transforms.
2. Trace timestamp provenance: device time, host receipt time, synchronization source, and any conversion offset.
3. Select exact synchronization only for matching stamps; otherwise define an approximate-time tolerance and bounded queues.
4. Inspect missing pairs, time jumps, delayed TF, and queue overflow before increasing queue sizes.
5. Match sensor QoS to actual publishers; inspect reliability and durability rather than assuming all subscriptions are reliable.
6. Choose `image_transport` plugins available on the target platform and measure compression cost versus network bandwidth.
7. Preserve `CameraInfo` association and image encodings through transforms; validate depth scale and invalid-value handling.
8. For point clouds, inspect field offsets/types, endianness, organized layout, NaNs, and frame before filtering or conversion.
9. Bound memory and processing queues; decide explicitly whether to drop stale frames or reduce input rate.

## GPU and Jetson paths
- Establish a CPU reference result before enabling acceleration; compare output accuracy as well as throughput.
- Verify architecture, OS/vendor stack, driver, CUDA/runtime, and acceleration package compatibility from installed versions.
- On Jetson, validate the supported JetPack/L4T matrix, memory budget, power mode, and thermal constraints.
- Do not assume desktop GPU packages or x86 binaries work on ARM; use supported platform artifacts.
- Measure transfer/conversion overhead and end-to-end latency; kernel speed alone is not pipeline performance.
- Provide a documented fallback when the GPU is unavailable; do not silently change numerical behavior.

## Recorded-data validation
1. Select an authorized recording containing representative images/scans/clouds, calibration, and required TF.
2. Inspect bag metadata, topic QoS, storage plugins, and duration before replaying in an isolated approved graph.
3. Choose one clock authority and configure consumers consistently for bag/simulation time; avoid mixed wall and simulated clocks.
4. Replay fixed segments and compare output counts, latency, frame alignment, calibration residuals, and reference results.
5. Test dropped frames, missing calibration, invalid depth/points, delayed transforms, timestamp jumps, and absent GPU.
6. Ensure replay outputs cannot reach live navigation or manipulation command consumers.

## Handoff
- Report recording provenance, calibration versions, topic/QoS contracts, timing assumptions, and measured resource use.
- Separate reproducible recorded/simulation evidence from live-sensor and hardware performance not yet validated.
