---
name: ros2-performance
description: "Use when measuring or diagnosing ROS 2 performance: end-to-end latency, jitter, throughput, missed deadlines, callback/executor contention, tracing/profiling, DDS queues, CPU/memory/GPU bottlenecks, or simulator real-time factor."
user-invocable: true
---

# ROS 2 performance

Read and follow the [development ground rules](../ros2-development/SKILL.md)
before work. Measure before optimizing. A faster incorrect system is not an
improvement, and a passing benchmark does not establish hard real-time safety.

## Define the requirement and baseline

1. Identify the user-visible requirement: sensor-to-decision latency, control
   deadline, action response, sustained throughput, startup time, or resource
   ceiling. Specify units, workload, measurement endpoints, acceptable loss,
   latency distribution/tail budget, and duration before changing code.
2. Record ROS distro, RMW/vendor versions, OS/architecture, build mode, CPU/GPU,
   executor/callback groups, QoS, payload sizes/rates, topology, and container/VM
   boundaries. On embedded targets record thermal throttling and power mode.
   Do not compare a debug build with an optimized build as a code improvement.
3. Create a representative reproducible workload with fixed input provenance,
   warm-up, bounded duration, and cleanup. Separate startup from steady state;
   include burst/overload and recovery where relevant. Isolate from live command
   endpoints; a bag replay can publish actuator commands and is not inherently safe.
4. Preserve an unmodified baseline and correctness checks. Repeat matched runs,
   reporting sample count, duration, variation, percentiles and observed maximum
   alongside loss/deadline counts. An observed maximum is not a worst-case bound.

## Collect trustworthy measurements

- Use a monotonic clock for local elapsed time. For cross-host timestamps verify
  synchronization and bound its error before subtracting timestamps; otherwise
  measure local segments or a clearly labeled round trip. ROS/simulation time can
  pause/jump and must not be mixed with wall time or a different clock domain.
- Distinguish message age, queue residence, callback execution, scheduling delay,
  transport delay, and full end-to-end latency. Correlate samples by sequence or
  trace identifiers, not timestamp coincidence; account for dropped samples.
- Check tools actually installed and supported by the host and ROS build. Options
  include `ros2_tracing`/LTTng on supported Linux systems, native CPU/memory
  profilers, Python profilers, and vendor GPU tools. Verify tracepoints, symbols,
  permissions, and version-specific commands before relying on them. Missing
  tooling is a stated blocker or reason to choose another measurement method.
- Start with a short bounded capture and prove it contains the expected events.
  Record instrumentation overhead, buffer loss, sampling limitations, and profiler
  configuration. Compare instrumented and uninstrumented runs where feasible.
  Protect trace/bag contents and do not upload sensitive data without approval.
- `ros2 topic hz`/`bw` describe what that subscriber observes, affected by QoS and
  its own processing. They do not prove publisher rate, end-to-end latency, or
  absence of loss. Extra observers, logging, and recording can change the system.

## Localize the bottleneck

| Evidence | Investigate next |
| --- | --- |
| Short callbacks but late execution | Executor scheduling, callback-group serialization, lock contention, CPU saturation |
| Long callback execution | Blocking I/O/waits, algorithms, allocation/copying, Python GIL, expensive logging |
| Growing message age or queues | Producer/consumer imbalance, stale work, backpressure, history/depth and reliability |
| Good local timing but slow cross-host delivery | Serialization, transport, bandwidth, fragmentation, DDS settings and clock error |
| Memory growth over repeated work | Retained messages/futures, unbounded queues, leaks versus allocator caching |
| GPU pipeline delay | CPU/GPU transfers, synchronization, batching, rendering and sensor cadence |
| Low simulator real-time factor | Physics/contact solver cost, rendering, bridge/control cadence, host contention |

Use [debugging](../ros2-debugging/SKILL.md) for correctness/deadlocks and
[networking](../ros2-networking/SKILL.md) for DDS/discovery/QoS causes. Route
executor/application changes to Core, simulator stepping/rendering to Simulation,
and privileged host changes to OS through the coordinator rather than creating
another performance specialist. Use [launch](../ros2-launch/SKILL.md) for startup.

## Optimize and retain evidence

1. State a hypothesis supported by the capture, change one factor, rerun the same
   workload, and compare against the baseline including correctness and tail
   behavior. Report tradeoffs rather than only the best run or mean throughput.
2. Bound queues and choose stale-data/backpressure policy from the application
   contract. Do not silently sacrifice delivery, sensor fidelity, cancellation,
   security, or safety checks to improve a metric. More executor threads do not
   automatically improve scheduling; verify callback concurrency and contention.
3. Verify intra-process/loaned-message support for the specific client library,
   RMW, message type, and topology before claiming zero-copy benefits. Measure
   actual copies/allocations rather than inferring them from a configuration flag.
4. Report simulator real-time factor as simulation-time advance divided by wall
   elapsed time over a stated non-paused interval. Separate physics, rendering,
   sensor and control rates. Faster-than-real-time throughput does not establish
   real-world control latency; increasing timestep can change dynamics/accuracy.
5. Never casually change real-time scheduling, CPU affinity/governors, kernel
   settings, firewall rules, or GPU drivers. Require explicit approval, a bounded
   safe experiment, and rollback for privileged or system-wide tuning.
6. Preserve discovered defects with permanent regressions using
   [testing](../ros2-test/SKILL.md): for example bounded queue growth and prompt
   cancellation under a slow consumer, or recovery after overload. Derive limits
   from requirements, not current timings. Keep noisy hardware benchmarks separate
   from deterministic unit tests; do not hide failures with retries or widen
   thresholds merely to pass CI. Document unavailable hardware/tooling explicitly.

Return the requirement, workload and environment, raw-evidence locations,
baseline/change comparison, measurement uncertainty, correctness results, and
remaining bottlenecks. Label proposed gains as unmeasured until demonstrated.