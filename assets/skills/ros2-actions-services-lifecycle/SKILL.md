---
name: ros2-actions-services-lifecycle
description: "Use when implementing ROS 2 action servers and clients, services, feedback, cancellation, rejection, timeouts, executor concurrency, lifecycle transitions, resource cleanup, and integration tests."
user-invocable: true
---

# ROS 2 Actions, Services, and Lifecycle

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, shell, architecture, ROS distribution/version, client-library versions, and active environment.
- Verify compatible ROS CLI, compiler or Python interpreter, and test tools; use only discovered shell-compatible setup scripts.
- Never assume installation paths, silently change environments, or execute an incompatible platform shim.
- Confirm the approved simulation versus hardware target and isolate test clients from production endpoints.
- Do not operate a live robot, arm, or drone without explicit approval; service calls and lifecycle transitions can actuate hardware.

## Choose the communication contract
1. Use a service for a bounded request/response; use an action for long-running work with feedback and cancellation.
2. Define input constraints, result/error semantics, deadlines, retry policy, and side effects before implementing callbacks.
3. Inspect existing generated interfaces and namespace/remapping conventions; avoid incompatible changes to deployed types.
4. Decide whether repeated requests are idempotent or need application-level request identifiers.

## Implement action servers and clients
1. Validate each goal and explicitly accept or reject it before allocating execution resources.
2. Define a concurrent-goal policy: reject, queue with bounds, or preempt with a documented handoff.
3. Keep long-running work off executor-blocking callbacks; select callback groups and executor threading deliberately.
4. Publish meaningful, bounded-rate feedback tied to the goal; avoid racing feedback with goal teardown.
5. Check cancellation cooperatively and release resources before reporting the appropriate canceled terminal state.
6. Complete every accepted goal exactly once with success, abort, or cancellation, including exception paths.
7. On the client, bound server discovery, goal acknowledgment, result waiting, and cancellation-response waits.
8. Handle rejection, server disappearance, stale feedback, and shutdown without leaving unresolved application futures.
9. A client timeout or cancel request does not prove execution stopped; verify the terminal result or use an approved recovery path.

## Services and concurrency
- Validate requests and return structured failures rather than hanging or throwing through middleware callbacks.
- Prefer asynchronous clients; avoid synchronous waits inside callbacks that need the same executor to complete.
- A multithreaded executor alone does not fix a mutually exclusive callback-group deadlock.
- Protect shared state across service, action, timer, and lifecycle callbacks; bound queues and worker lifetimes.
- Treat a timed-out service as an unknown outcome: ordinary service requests do not provide action-style cancellation.
- Retry side-effecting requests only when their idempotency or deduplication contract makes it safe.

## Lifecycle procedure
1. Query actual state and available transitions before requesting any transition; never infer state from launch order.
2. Allocate resources in configure, enable operation in activate, and stop output/work in deactivate.
3. Release configured resources in cleanup and implement shutdown/error handling for partial initialization failures.
4. Reject or defer work in inappropriate states; coordinate outstanding goals before deactivation or cleanup.
5. Make cleanup repeatable, cancel timers, stop/join workers, and release publishers/drivers without races.
6. Verify each transition's result and resulting state; do not force repeated activation after an unexplained failure.

## Validation
- Test success, invalid goal rejection, feedback, cancellation before/during execution, abort, timeout, and server loss.
- Test concurrent clients, repeated requests, shutdown during execution, and executor starvation/deadlock scenarios.
- Exercise configure/activate/deactivate/cleanup cycles and failed configuration without leaked resources or continued output.
- Use bounded waits and deterministic mock work in an isolated graph; assert terminal states, not only log messages.
- Report client-library versions, transition traces, test evidence, and unresolved side-effect outcomes.
