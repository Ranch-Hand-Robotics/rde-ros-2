# ROS 2 Installation Guide

The Robot Developer Extensions for ROS 2 includes an integrated ROS 2 installer that simplifies the process of setting up ROS 2 on your development machine.

## Features

- **Automatic Detection**: The extension automatically detects when you're working in a ROS workspace (by finding `package.xml` files) and prompts you to install ROS 2 if it's not already installed.
- **Multiple Platforms**: Supports installation on Linux, Windows, and macOS using the appropriate package manager for each platform.
- **Distro Selection**: Choose from current ROS 2 distributions with clear marking of LTS (Long-Term Support) releases.
- **Pixi Integration**: On Windows and macOS, the extension can automatically install and configure Pixi for ROS 2 development.
- **AI-Powered Troubleshooting**: If installation fails, get help from GitHub Copilot to diagnose and fix issues.

## Installation Methods

### Linux (APT)

On Linux systems, ROS 2 is installed using the APT package manager. This requires:

- Ubuntu (recommended versions depend on the ROS 2 distro)
- sudo privileges (you'll be prompted for your password)
- Internet connection

The extension will:
1. Set up the ROS 2 repository and GPG keys
2. Install the `ros-<distro>-desktop` package
3. Display all commands in a terminal so you can monitor progress

### Windows (Pixi)

On Windows, ROS 2 is installed using Pixi, a modern cross-platform package manager from prefix.dev.

The extension will:
1. Check if Pixi is installed
2. If not, offer to install Pixi using Windows Package Manager (`winget`)
3. Create a Pixi workspace for ROS 2
4. Install the ROS 2 distribution via its distribution-specific RoboStack channel

**Prerequisites**:
- Windows 10 or later
- Visual Studio 2019 or later (for building C++ packages)
- PowerShell execution policy that allows script execution

### macOS (Pixi)

On macOS, ROS 2 is installed using Pixi.

The extension will:
1. Check if Pixi is installed
2. If not, offer to install Pixi using curl
3. Create a Pixi workspace for ROS 2
4. Install the ROS 2 distribution via its distribution-specific RoboStack channel

## How to Use

### Automatic Prompt

When you open a ROS workspace (containing `package.xml` files) and ROS 2 is not detected, you'll see a prompt:

> "ROS 2 is not detected on this system, but this appears to be a ROS workspace. Would you like to install ROS 2?"

You have three options:
- **Yes**: Starts the installation process
- **No**: Dismisses the prompt (you'll be asked again next time)
- **Never for this workspace**: Remembers your choice and won't prompt again for this workspace

### Manual Installation

You can also manually trigger the installation:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Search for "ROS2: Install ROS 2"
3. Select it and follow the prompts

### Readiness preflight

Before Pixi bootstrap, writing the installation target, or changing system
packages/repositories, the installer performs a non-destructive readiness scan.
**Any blocker aborts installation.** There is no "install anyway" override for
blockers and the extension does not repair the OS automatically.

| Check | Blocking conditions |
|-------|---------------------|
| Existing target | A nonempty target, a target symlink/non-directory, or already installed packages for the selected APT ROS distribution. |
| Ubuntu compatibility | Wrong Ubuntu codename, unsupported architecture, or inability to establish compatibility. Humble requires Jammy; Jazzy/Kilted require Noble. Newer distributions and Rolling are checked against ROS distribution metadata. |
| APT/dpkg state | Missing prerequisite tools, active package operations, incomplete package configuration, broken dependencies, or failure to inspect package-manager state. |
| Restart state | Ubuntu's reboot-required marker, or a pending Windows reboot/installer operation. |
| Storage | Less than 8 GiB on the target filesystem, 1 GiB of writable temporary space, 2 GiB on the APT cache filesystem, or fewer than 20,000 available target inodes where reported. These are conservative floors, not exact download/install-size estimates. |
| Native host/Pixi | Unsupported host architecture, failed host-readiness inspection, broken Pixi, or missing bootstrap prerequisites when Pixi is absent. |
| Dependency plan | An APT simulation that cannot resolve dependencies without removals, or a failed Pixi dependency solve. |

Warnings require explicit confirmation. Examples include other distributions
alongside the selected target, intentional held packages, JetPack compatibility
limitations, missing macOS build tools, and dependency changes shown by APT.
**Install ROS 2 is a fresh-install command, not an upgrade or repair command.**
Use the independent health command for an existing installation. For Pixi, choose
a different root rather than overwriting an existing or partial environment.

When the ROS package is already present in local APT metadata, preflight runs an
unprivileged simulation with removals disabled. On a fresh machine without the
ROS repository, it reports that full dependency resolution must wait until
repository setup; it does not claim to have solved the missing package.
Installation itself also refuses package removals. Pending package updates alone
are not classified as corruption, and no `upgrade`, `fix-broken`, package
reconfiguration, lock deletion, or reboot is performed automatically.

For Pixi, a candidate manifest is restricted to the selected distro and host.
`pixi lock` resolves it in the run's diagnostic staging directory without
installing packages. The resolved manifest and lockfile are copied exclusively
to the new target, then installation uses the lockfile with `--locked`. If Pixi
is missing, system readiness is checked first; the dependency solve follows
explicit bootstrap approval, so a solver failure can leave Pixi itself installed
but does not create the ROS workspace. Repository metadata caches may also change.

Blocking checks include actionable guidance in the persistent report and
**Diagnose Step** drafts. Fix the reported condition, then invoke installation
again to run a fresh scan. Preflight cannot guarantee network availability or
that another process will not change the system afterward; execution-time
failure handling and post-install validation still apply.

## Selecting a ROS 2 Distribution

When installing, you'll be prompted to select a ROS 2 distribution:

| Distribution | Release Date | Type | Supported Platforms |
|--------------|--------------|------|---------------------|
| Kilted Kaiju | May 2024 | Latest | Ubuntu 24.04, Windows, macOS |
| Jazzy Jalisco | May 2024 | **LTS** | Ubuntu 24.04, Windows, macOS |
| Iron Irwini | November 2023 | Standard | Ubuntu 22.04, Windows, macOS |
| Humble Hawksbill | May 2022 | **LTS** | Ubuntu 22.04, Windows, macOS |

**LTS (Long-Term Support)** releases receive 5 years of support and are recommended for production use.

## Configuration

### Pixi Root Directory

By default, Pixi workspaces are created in `c:\pixi_ws` on Windows. You can change this:

1. Open Settings (`Ctrl+,` or `Cmd+,`)
2. Search for "ROS2 Pixi Root"
3. Change the `ROS2.pixiRoot` setting to your preferred location

### Never Install Prompt

If you selected "Never for this workspace", you can re-enable the prompt:

1. Open workspace settings (`.vscode/settings.json`)
2. Remove or set `"ROS2.neverInstallRos": false`

## Troubleshooting with Copilot

Every installation records named steps, command output, host information, the
generated installation script, and a JSON result in the extension's persistent
storage. The Output channel prints the exact paths. Ubuntu release, architecture,
WSL kernel detection, remote extension-host type, and Jetson L4T information help
distinguish otherwise similar Linux installations.

On failure, choose **View Report** or **Diagnose Step**. You can also run
**ROS2: Show ROS 2 Installation Report** later, including after reloading VS Code.
Choose a package-installation or health-check step to create an AI troubleshooting
draft. It includes the step results, relevant log tail, and recovery limitations.
An early step's log remains available even when subsequent output is lengthy.

The draft opens locally for review and editing. Common credential patterns are
redacted, but this is **not a guarantee that all sensitive data is removed**.
Raw local logs are not redacted; they and the draft can include paths, repository
addresses, and package-manager output. The full process environment is not
collected. Review the draft before selecting **Copy Reviewed Draft** for any AI
assistant or **Copy and Open Copilot**, then paste it yourself. Nothing is sent
automatically and no AI-suggested repair commands are executed.

Reports are retained until you remove their individual run directories from the
reported storage location. A report left in `running` state after a reload is
**not evidence of success**: the original process may still be running or may have
been interrupted. Inspect the terminal and log before retrying.

## Common Issues

### Linux

**Problem**: "GPG key error"
- **Solution**: The extension handles this automatically, but if you see this error, ensure you have internet connectivity and can reach ROS package servers.

**Problem**: "Permission denied"
- **Solution**: The installation requires sudo privileges. Make sure you enter your password when prompted.

**Problem**: "Unsupported Ubuntu version"
- **Solution**: Check the ROS 2 distro requirements. You may need to select a different distro or upgrade your Ubuntu version.

### Windows

**Problem**: "Pixi installation failed - Execution Policy"
- **Solution**: Run PowerShell as Administrator and execute: `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

**Problem**: "Visual Studio not found"
- **Solution**: Install Visual Studio 2019 or later with C++ development tools.

### macOS

**Problem**: "Pixi installation fails"
- **Solution**: Ensure you have `curl` installed and can access pixi.sh. Check your firewall settings.

## Manual Installation Alternative

If automated installation doesn't work for your setup, you can install ROS 2 manually:

- **Linux**: Follow the [official ROS 2 documentation](https://docs.ros.org/en/rolling/Installation.html)
- **Windows/macOS**: Follow the [Pixi setup guide](https://pixi.sh/latest/) and [RoboStack instructions](https://robostack.github.io/)

After manual installation:
1. Reload the VS Code window (`Ctrl+Shift+P` → "Developer: Reload Window")
2. The extension should automatically detect your ROS 2 installation

## Post-Installation

Package-manager exit code zero is no longer sufficient for a successful install.
The installer first runs the runtime health checks below against the exact target,
then offers to reload VS Code. If validation fails, installed packages remain in
place and the report distinguishes runtime failure from package-installation failure.

On Linux the target is `/opt/ros/<distro>/setup.bash`. On Windows and macOS it is
the selected Pixi environment in `<ROS2.pixiRoot>/<distro>` (default roots:
`c:\pixi_ws` and `~/pixi_ws`, respectively). Installer-generated Pixi manifests do
not activate a workspace's `install/setup.*` overlay before it has been built.

### Check an installation independently

Run **ROS2: Check ROS 2 Installation Health**, select the expected ROS distribution,
then choose the default installation, configured setup script, a custom setup
script, or a `pixi.toml`. This also works for a partial installation that the
extension cannot yet discover. Run the command in the target extension host:
Remote-WSL for WSL and Remote-SSH for a Jetson, not in the desktop host's environment.
Only run setup scripts and Pixi manifests you trust; activation executes their code.

The bounded checks exercise activation of the selected environment, the expected
ROS version/distribution, CLI availability, package discovery, Python/native ROS
libraries, and an isolated local publish/subscribe exchange. They do not install
or repair packages. The probe has a 45-second execution deadline and a 128-KiB
output limit; CLI execution and message exchange each have an 8-second deadline.
Timeouts terminate the probe process tree. The local message check supports Fast
DDS and Cyclone DDS; other middleware produces an explicit unsupported result.
A passed report does **not** establish C++ compiler/build
health, cross-machine DDS connectivity, GPU/CUDA compatibility, or successful
VS Code environment discovery after reload. **ROS2: Doctor** remains available
for additional diagnostics.

Other extensions or integration tests can call the same command with an explicit
target to skip selection and receive a structured result without interactive
result prompts:

```typescript
const report = await vscode.commands.executeCommand("ROS2.checkInstallation", {
  kind: "setup",
  distro: "jazzy",
  setupScript: "/opt/ros/jazzy/setup.bash",
});
// Pixi target: { kind: "pixi", distro: "jazzy", workspace: "/path/to/pixi_ws/jazzy" }
// Result: { healthy, target, checks: [{ id, status, detail, durationMs }], ... }
```

For a native Windows `.bat` setup, a compatible `python.exe` is resolved from
the original PATH before the health environment is sanitized. A caller may set
`pythonExecutable` on a `kind: "setup"` target to choose it explicitly. Pixi
targets always use Python inside the named environment, never the system Python.
Windows setup/probe/interpreter paths containing CMD expansion characters are
rejected explicitly rather than interpolated into a shell command.

The health-check module also exports `validateInstallation(target, probePath)`;
the same function is used by post-install validation and the command. A runtime
failure returns `healthy: false`; orchestration/reporting errors reject the command.

### Recovery and rollback limitations

**The installer does not provide transactional rollback.** A failed or interrupted
installation must not be assumed to have restored the previous system.

| Path | Changes that may remain | Recovery considerations |
|------|-------------------------|-------------------------|
| Ubuntu / Ubuntu in WSL | APT packages and shared dependencies, locales, universe enablement, ROS repository and key | Inspect the failed step and APT/dpkg history. Removing the desktop metapackage does not undo dependency upgrades or system configuration. |
| Windows / macOS Pixi | Pixi itself and PATH changes, caches, newly created manifest/lockfile and partial environment | Existing nonempty targets are refused rather than overwritten. A failed newly created target needs manual assessment before retrying; there is no environment snapshot/rollback. |
| Jetson | The same APT changes, on top of an existing JetPack/L4T installation | Preserve the device's JetPack baseline. Generic runtime validation does not prove CUDA or NVIDIA package compatibility. |
| Reload / terminal interruption | Any changes already performed; a process may outlive the extension host | Inspect actual process state before retrying. The in-memory concurrent-install guard does not survive extension-host reloads. |

The active command uses one Linux APT path for Ubuntu, WSL and Jetson, rather
than the older separate Jetson installer. It records the host distinction but
does not add Jetson-specific packages or validate JetPack compatibility.
Preflight checks the Ubuntu release and architecture; the distribution picker
alone is not a compatibility guarantee.

Before installing over a valuable existing environment, use a VM/disk snapshot
or a separate Pixi workspace root. On failure, use the recorded step evidence to
plan recovery rather than blindly running package purges, `autoremove`, or
deleting an environment.

## Related Settings

| Setting | Description | Default |
|---------|-------------|---------|
| `ROS2.distro` | ROS distribution to source | (empty) |
| `ROS2.rosSetupScript` | Path to ROS setup script | (auto-detected) |
| `ROS2.pixiRoot` | Pixi workspace root directory | `c:\pixi_ws` |
| `ROS2.neverInstallRos` | Never prompt to install ROS for this workspace | `false` |

## Getting Help

If you encounter issues:

1. Check the **ROS 2 output channel** in VS Code (View → Output → ROS 2)
2. Use **ROS2: Check ROS 2 Installation Health** for runtime validation or **ROS2: Doctor** for additional diagnostics
3. Use **ROS2: Show ROS 2 Installation Report** and **Diagnose Step** to review evidence for AI assistance
4. Consult the [troubleshooting guide](./troubleshooting.md)
5. File an issue on [GitHub](https://github.com/ranchhandrobotics/rde-ros-2/issues)

## Next Steps

After installing ROS 2:

- Learn about [ROS 2 debugging features](./debug-support.md)
- Set up [IntelliSense for ROS 2](./intellisense.md)
- Explore [launch file debugging](./launchdebugging.md)
- Try the [tutorials](./tutorials.md)
