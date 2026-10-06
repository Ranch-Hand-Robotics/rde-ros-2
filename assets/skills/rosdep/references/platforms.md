# Platform checks for source dependencies

Apply only the section for the actual build host. Source availability does not
imply platform support. Stay within the workspace's chosen dependency ecosystem.
These notes are for a specific blocker or a requested install/build, not a
mandatory preflight for index queries or a source-clone offer. Do not activate
compilers or audit SDKs just to find a dependency's binary or repository.

## NVIDIA Jetson

- Record the Jetson model, JetPack/L4T version, Ubuntu version, `aarch64`
  architecture, ROS distro, and native versus container environment. Do not treat
  an Ubuntu x86_64 build recipe as a Jetson recipe.
- On supported native Ubuntu installations, rosdep can still be appropriate.
  Diagnose the exact key or version failure instead of bypassing all rosdep
  mappings simply because the device is a Jetson.
- Check each package's actual `linux-aarch64` availability when using Pixi;
  `linux-64` is not a substitute. Do not falsify glibc or CUDA requirements to
  force an environment solve.
- Match CUDA-dependent sources to the installed JetPack stack. Kernel modules,
  camera backends, TensorRT, and vendor SDKs can be blockers even when a ROS
  wrapper builds. Do not upgrade drivers or replace system libraries implicitly.
- Budget RAM, disk, and build parallelism. Propose a small targeted build; ask
  before adding swap, changing system services, or rebuilding a large underlay.

## Windows with Pixi/RoboStack

- Use the actual PowerShell or CMD syntax and verified Windows executables.
  Bash `source`, Linux paths, and extensionless Unix shims are not substitutes.
- Activate a complete Visual Studio C++ compiler and Windows SDK environment
  before Pixi/ROS. Use the extension's build preflight where available; missing
  compiler tools are a prerequisite issue, not a reason to clone ROS packages.
- Preserve the selected manifest, named environment, Python, and target
  architecture. Query `win-64` packages only for a compatible x64 target; do not
  infer native ARM64 support from x64 package availability.
- Prefer the extension's colcon build task, which sources the environment and
  uses `--merge-install` on Windows. Do not assume symlink privileges or silently
  change Windows security settings.
- Check upstream Windows CI, POSIX-only APIs, compiler requirements, DLL export
  handling, and native SDK support. Linux-only packages may need a port rather
  than a different Git branch. Suggest WSL/container alternatives only explicitly
  and explain device-access implications before switching environments.

## macOS with Pixi/RoboStack

- Distinguish Apple Silicon `osx-arm64` from Intel `osx-64`, and confirm the
  deployment target. Do not mix architectures or silently run under Rosetta.
- Verify Apple Command Line Tools/Xcode, the active SDK, and compatible CMake,
  Python, and compiler selection before diagnosing missing source dependencies.
- Use the selected Pixi environment for native libraries as well as ROS. Do not
  casually mix Homebrew or system Python libraries into a RoboStack build;
  framework paths, ABI differences, and RPATH can cause misleading errors.
- Check platform-specific device backends and upstream support. Successful
  compilation is not evidence of camera, USB, GPU, or driver functionality.

## All Pixi platforms

The RoboStack FAQ documents limitations in rosdep's Pixi integration. Check the
installed tooling and current upstream guidance instead of assuming that having
`rosdep` installed means it can modify the correct Pixi environment. Read-only
dependency inspection may still be useful.

Do not source a system ROS installation into an unrelated RoboStack environment.
Respect the existing channel order and lockfile; add compatible packages to the
correct feature/environment, not a new implicit default environment. Distinguish
an unavailable binary from conflicting pins, missing activation, or network
failure. Optional dependencies may be disabled only with the user's agreement
and an explicit description of the lost functionality.