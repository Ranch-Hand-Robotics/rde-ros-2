# Common source-dependency recipes

Use these entries only after checking the selected environment's installed
packages and current RoboStack channel metadata. Inclusion is not evidence that
a package has no binaries. Availability varies by distro, platform, version,
and channel pins. Network or dashboard failures mean **availability unknown**.
Do not clone anything solely because it appears here; the skill's approval,
workspace safety, and verification steps still apply.

## Diagnostics: diagnostic_updater, diagnostic_aggregator, self_test

**Canonical source:** https://github.com/ros/diagnostics.git

**License:** BSD-3-Clause; check the selected revision before redistribution.

**Source metadata reviewed:** 2026-10-04. Repository and branch refs verified;
no Windows, macOS, or Jetson source build was performed for this recipe.

### Package-to-repository mapping

| Missing ROS package | Path inside the diagnostics repository | Target to build |
| --- | --- | --- |
| `diagnostic_updater` | `diagnostic_updater/package.xml` | `diagnostic_updater` |
| `diagnostic_aggregator` | `diagnostic_aggregator/package.xml` | `diagnostic_aggregator` |
| `self_test` | `self_test/package.xml` | `self_test` |

Clone this repository at most once into `diagnostics` beside the requesting
package directory, following the skill's peer-directory rule. For example,
`/ws/camera/package.xml` implies `/ws/diagnostics`, not `/ws/camera/src/diagnostics`.
Check and show the exact destination before offering the clone. It contains
additional packages, including a `diagnostics` metapackage. Do not build that
metapackage or all siblings just to satisfy `diagnostic_updater`.
`diagnostic_msgs` is a separate dependency, not supplied by this repository.

### Check binaries first

Search the configured RoboStack channel for the requested ROS package on the
actual target subdirectory (`win-64`, `osx-arm64`, `osx-64`, or `linux-aarch64`
for typical Jetson targets). For the updater, search both naming conventions
when relevant: `ros-<distro>-diagnostic-updater` and `ros2-diagnostic-updater`.
Inspect the returned package's distro constraints, including `ros2-distro-mutex`
where present, rather than assuming the name determines compatibility.

Use the skill's direct platform/channel search first. The
[RoboStack Lyrical dashboard](https://robostack.github.io/lyrical.html) is an
optional reference, not a required investigation step. A failed query or missing
dashboard row does not establish absence; a successful complete package-index
search with no compatible result does. No fixed availability matrix is maintained
in this recipe.

### Distro-specific source candidates

These are starting points from upstream's distro guidance, not immutable pins.
Recheck upstream and rosdistro metadata at use time, verify the requested package
at the chosen revision, and resolve the ref to a commit SHA for the clone offer.
A `.repos` manifest is optional, not a prerequisite. Do not silently use a
different distro's branch.

| ROS distro | Upstream branch candidate | Observed rosdistro source ref at review |
| --- | --- | --- |
| Humble | `ros2-humble` | `ros2-humble` |
| Jazzy | `ros2-jazzy` | `ros2-jazzy` |
| Kilted | `ros2-kilted` | `ros2-kilted` |
| Lyrical | `ros2-lyrical` | `ros2` — conflict; investigate before selecting |
| Rolling | `ros2` | `ros2` |

**Lyrical caution:** upstream's README identifies `ros2` as Rolling and
`ros2-lyrical` as Lyrical, while Lyrical's rosdistro entry pointed to `ros2` at
review. The inspected Lyrical updater manifest was version 4.4.7, matching the
upstream version portion of Lyrical's 4.4.7-1 release entry. This is corroborating
metadata, not a cross-platform build guarantee. Verify the current branch/tag
and dependency API requirements instead of automatically checking out `ros2`.
For any unlisted distro, research its supported revision rather than guessing.

### Optional follow-up: dependencies and build scope

Only use this section if the user requests an install or build. It is not a
checklist to complete before offering the repository and peer-directory clone.

At the inspected Lyrical revision:

- `diagnostic_updater` uses `ament_cmake`, `ament_cmake_python`,
  `ament_cmake_ros`, `diagnostic_msgs`, `rclcpp`, `rclpy`, and `std_msgs`.
  Its CMake configuration requires CMake 3.20 and defaults to C++17.
- `diagnostic_aggregator` additionally needs `pluginlib` and `rcl_interfaces`;
  read its own manifest rather than treating the updater's list as sufficient.
- `self_test` depends on `diagnostic_updater`, `diagnostic_msgs`, `rclcpp`,
  and build-time `ros_environment`, in addition to its ament build tool.
- Testing adds dependencies such as ament gtest/pytest/lint tooling,
  `rclcpp_lifecycle`, and launch testing packages, depending on the target.
  Inspect both `package.xml` and CMake at the selected commit for the exact set.
  Do not automatically disable tests just because test dependencies are missing.

Reuse compatible dependencies from the underlay; do not clone `rclcpp` or another
ROS core package merely because a prefix is not activated. Check all package names
in an existing diagnostics checkout before adding another copy. If the checkout
would override packages already supplied by the underlay, assess API/ABI and
downstream rebuild requirements; do not silence override warnings automatically.

After approval, acquire the selected commit and verify it. For an updater-only
recovery, use `--packages-up-to diagnostic_updater` in the platform-appropriate
colcon build. For `self_test`, the same selection mechanism includes its workspace
updater dependency. Keep unrelated diagnostics packages out of the build closure.

Source the built overlay, verify `ros2 pkg prefix diagnostic_updater` resolves to
it, and test the selected package using `colcon test --packages-select
diagnostic_updater` followed by `colcon test-result --verbose`. Use the equivalent
target names for the aggregator or self-test package. Rebuild the original
requesting package as the final integration check. Report unavailable tests and
platform limitations; neither this recipe nor an upstream Windows CMake setting
proves that the chosen environment builds successfully.

### Evidence to revisit

- [Upstream overview and distro branches](https://raw.githubusercontent.com/ros/diagnostics/ros2/README.md)
- [Lyrical updater manifest](https://raw.githubusercontent.com/ros/diagnostics/ros2-lyrical/diagnostic_updater/package.xml)
- [Lyrical updater CMake configuration](https://raw.githubusercontent.com/ros/diagnostics/ros2-lyrical/diagnostic_updater/CMakeLists.txt)
- [Lyrical aggregator manifest](https://raw.githubusercontent.com/ros/diagnostics/ros2-lyrical/diagnostic_aggregator/package.xml)
- [Lyrical self-test manifest](https://raw.githubusercontent.com/ros/diagnostics/ros2-lyrical/self_test/package.xml)
- [Lyrical rosdistro metadata](https://raw.githubusercontent.com/ros/rosdistro/master/lyrical/distribution.yaml)

For other distros, inspect their own distribution metadata and manifests at the
selected ref. Do not generalize the Lyrical dependency snapshot to every release.