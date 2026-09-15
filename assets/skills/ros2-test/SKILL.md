---
name: ros2-test
description: "Use when designing, writing, running, or reviewing meaningful ROS 2 tests: behavior contracts, regression reproduction, pytest/unittest, GTest, launch_testing, colcon test/test-result, simulation, flaky tests, coverage, and CI."
user-invocable: true
---

# ROS 2 Meaningful Testing

## Preflight and evidence

- Read [common platform and safety guidance](../ros2-development/SKILL.md); stop
  execution if unavailable. Confirm OS, shell, architecture, distro, interpreter,
  workspace, underlay/overlay, and compatible test executables before commands.
- Inspect requirements, reported failures, public interfaces, existing tests,
  package.xml test dependencies, build registration, and CI configuration.
- Default to pure unit tests, mocks, or isolated simulation. Obtain explicit
  approval before tests access hardware, open serial ports, activate controllers,
  replay command topics, or otherwise change a live robot's state.

## Define what correctness means

1. State the intended observable behavior independently of the implementation.
   Use an agreed requirement, protocol, domain invariant, or manually reasoned
   input/output example. If the requirement is unclear, ask or document the
   assumption; label characterization tests as current behavior, not correctness.
2. Map each test to a requirement or risk: scenario, expected outcome, source of
   that expectation, and plausible defect that should make it fail. A test unable
   to distinguish correct from realistically broken behavior adds little value.
3. Prioritize user impact: valid operation, meaningful boundaries, invalid inputs,
   failure handling, and recovery. Test units, frame conventions, numerical
   tolerances, ordering, or timing only where the contract requires them.
4. Do not copy production algorithms into assertions, generate expected values
   with the function under test, or bless snapshots of arbitrary current output.
   Assert public outcomes and required side effects, not private method names,
   incidental call order, or internal data layouts. Use mocks at external seams;
   call-count assertions are appropriate only for contractual effects such as
   preventing duplicate commands, not as a substitute for testing behavior.
5. Prefer parameterized boundaries, independently known examples, and property
   or metamorphic tests grounded in real invariants. Coverage identifies untested
   branches; it does not prove assertions are meaningful. Avoid redundant tests
   whose only purpose is increasing line coverage or test totals.

## Capture discovered bugs as regressions

- Whenever a bug is discovered, add or extend a permanent automated regression
   test within the authorized scope. Minimize the failing input, event sequence,
   or fixture and assert the intended outcome, not the buggy behavior.
- Name the test for the broken contract and retain relevant issue/reproducer
   context in a short comment when available. Use sanitized, small fixtures, not
   secrets or unnecessary production recordings.
- Prefer writing the test before the fix. Confirm it fails for the actual defect,
   not an unrelated setup error, then passes with the fix. Retain it in the normal
   discovered test suite/CI so reintroducing the defect is detected later.
- Extend an existing test when it already represents the contract; do not create
   duplicates merely to associate one test with every issue. Include a nearby
   boundary or recovery case when it guards the same underlying failure mechanism.
- If hardware, dependencies, authorization, or nondeterminism blocks automation,
   report the minimal reproducer, reason, and remaining regression risk. Provide
   a safe manual check or proposed follow-up; do not claim missing coverage exists
   or silently broaden scope to fix unrelated bugs.

## Choose and implement the right layer

1. **Unit:** Use pytest/unittest for Python and GTest for C++ according to project
   conventions. Separate pure decisions from transport where practical, exercise
   real logic, and use representative fixtures instead of a fully mocked system.
2. **Package integration:** Inspect `BUILD_TESTING`, `ament_cmake_gtest`,
   `ament_cmake_pytest`, or the ament_python pytest configuration as applicable.
   Register tests and test dependencies for the supported distro; a test file
   that CI never discovers is not coverage. Verify installed resources where
   source-tree execution could hide a packaging failure.
3. **ROS integration:** Use `launch_testing` (and `launch_testing_ament_cmake`
   when appropriate) for actual nodes/processes. Coordinate readiness with
   `ReadyToTest` and bounded condition waits, assert topic/service/action behavior,
   and check post-shutdown exit codes and cleanup. Do not substitute ROS 1 rostest.
4. **Contract examples:** A cancelled action must reach its promised terminal
   result and stop subsequent work; a failed lifecycle configure must leave the
   specified state and release resources; an unavailable service must time out
   predictably rather than hang. Merely asserting that cancel/configure/call was
   invoked does not prove these behaviors. Adapt examples to the actual contract.
5. **Isolation:** Use test-owned nodes, namespaces, temporary resources, and an
   available `ROS_DOMAIN_ID` with appropriate discovery/network isolation. A
   domain ID is not a security or hardware-safety boundary. Clean up only test-owned
   nodes, processes, executors, contexts, and files, including on assertion failure.
6. **Determinism:** Control seeds, fixtures, and clocks; distinguish ROS simulated
   time from wall-clock deadlines. Wait for observable readiness with time bounds,
   not arbitrary sleeps. Avoid dependence on ambient robots, network services,
   shared device state, or test execution order. Keep hardware tests opt-in.

## Execute and challenge the tests

1. Use [build](../ros2-build/SKILL.md) to build affected packages/dependencies and
   activate the correct overlay. Confirm test discovery before execution.
2. Run the smallest relevant scope, then affected integration tests. Use
   `colcon test --packages-select <package>` and inspect
   `colcon test-result --verbose`; honor configured build/result bases and verify
   result timestamps and scope so stale XML cannot masquerade as a fresh pass.
   Capture both exit statuses, including reporting when the test command fails.
3. For fast iteration use the verified interpreter's `-m pytest` with a selected
   file/node ID, or CTest/GTest filters for the built target. Check local command
   help and existing CI flags rather than assuming colcon plugin options exist.
   Use VS Code Test Explorer for supported cases, but verify discovery and runner
   support; use the native package runner for unsupported launch/integration tests.
4. For a regression, show the new test failing for the intended reason before the
   fix and passing afterward when feasible. Otherwise consider a safe, temporary
   mutation (wrong sign, omitted validation, ignored cancellation) and confirm the
   test catches it. Preserve user edits and restore only your mutation. Report
   unperformed checks explicitly; never claim sensitivity from a passing test alone.
5. Diagnose failures by requirement, product bug, test bug, or environment issue.
   Investigate flakes with bounded reruns and captured evidence; do not mask them
   with automatic retries, relaxed assertions, arbitrary timeout increases, or
   unexplained skips. Preserve failure logs and CI result artifacts.

## Report

Summarize requirements exercised, realistic regressions detected, exact commands
and environment, fresh discovered/passed/failed/skipped counts, and failure logs.
Treat zero tests where tests were expected as a validation failure. Explain skips,
unverified platforms/hardware, and remaining risks. Do not equate a green build,
high coverage, or a large test count with evidence that the requirements are met.