// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import { PreflightCheck, PreflightRunner } from "./preflight-types";

// Missing registry values are normal; inaccessible keys must fail the readiness query.
const WIN_REBOOT_SCRIPT = `
$ErrorActionPreference = 'Stop'
$session = Get-ItemProperty -LiteralPath 'HKLM:\\SYSTEM\\CurrentControlSet\\Control\\Session Manager' -ErrorAction Stop
$renames = $session.PSObject.Properties['PendingFileRenameOperations']
$developerModeEnabled = $false
$developerModeKnown = $true
try {
  $developerModePath = 'HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\AppModelUnlock'
  if (Test-Path -LiteralPath $developerModePath -ErrorAction Stop) {
    $developerMode = Get-ItemProperty -LiteralPath $developerModePath -ErrorAction Stop
    $setting = $developerMode.PSObject.Properties['AllowDevelopmentWithoutDevLicense']
    $developerModeEnabled = ($null -ne $setting -and [int]$setting.Value -eq 1)
  }
} catch {
  $developerModeKnown = $false
}
$longPathsEnabled = $false
$longPathsKnown = $true
try {
  $fileSystem = Get-ItemProperty -LiteralPath 'HKLM:\\SYSTEM\\CurrentControlSet\\Control\\FileSystem' -ErrorAction Stop
  $setting = $fileSystem.PSObject.Properties['LongPathsEnabled']
  $longPathsEnabled = ($null -ne $setting -and [int]$setting.Value -eq 1)
} catch {
  $longPathsKnown = $false
}
$result = @{
  cbsRebootPending = Test-Path -LiteralPath 'HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Component Based Servicing\\RebootPending' -ErrorAction Stop
  wuRebootRequired = Test-Path -LiteralPath 'HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\WindowsUpdate\\Auto Update\\RebootRequired' -ErrorAction Stop
  pendingFileRenames = ($null -ne $renames -and @($renames.Value | Where-Object { $_ }).Count -gt 0)
  installerInProgress = Test-Path -LiteralPath 'HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Installer\\InProgress' -ErrorAction Stop
  developerModeEnabled = $developerModeEnabled
  developerModeKnown = $developerModeKnown
  longPathsEnabled = $longPathsEnabled
  longPathsKnown = $longPathsKnown
}
ConvertTo-Json -InputObject $result -Compress
`.trim();

async function windowsPreflight(arch: string, run: PreflightRunner): Promise<PreflightCheck[]> {
  const checks: PreflightCheck[] = [];

  if (arch !== "x64") {
    checks.push({
      id: "windows-arch",
      status: "blocked",
      detail: `Architecture '${arch}' is not supported on Windows; only x64 is confirmed to have Pixi ROS packages.`,
      remediation: "Use a Windows x64 machine.",
    });
    return checks;
  }
  checks.push({ id: "windows-arch", status: "passed", detail: `Architecture ${arch} supported.` });

  const verResult = await run("cmd.exe", ["/d", "/c", "ver"], 10_000);
  if (verResult.exitCode !== 0 || verResult.error) {
    checks.push({
      id: "windows-version",
      status: "blocked",
      detail: `Could not determine Windows version: ${verResult.error ?? verResult.stderr}`,
      remediation: "Ensure cmd.exe can run and this extension has sufficient permissions.",
    });
  } else {
    const match = verResult.stdout.match(/\[[^\d]*(\d+)\.\d+/);
    if (!match) {
      checks.push({
        id: "windows-version",
        status: "blocked",
        detail: `Windows version output was unrecognizable: ${verResult.stdout.trim()}`,
        remediation: "Windows 10 or later is required.",
      });
    } else {
      const major = parseInt(match[1], 10);
      if (major < 10) {
        checks.push({
          id: "windows-version",
          status: "blocked",
          detail: `Windows version ${verResult.stdout.trim()} is too old (major=${major}); Windows 10+ required.`,
          remediation: "Upgrade to Windows 10 or Windows 11.",
        });
      } else {
        checks.push({ id: "windows-version", status: "passed", detail: `Windows version major=${major} (>=10) is supported.` });
      }
    }
  }

  const psResult = await run(
    "powershell.exe",
    ["-NoProfile", "-NonInteractive", "-Command", WIN_REBOOT_SCRIPT],
    15_000,
  );

  if (psResult.exitCode !== 0 || psResult.error) {
    checks.push({
      id: "windows-reboot",
      status: "blocked",
      detail: `Reboot-pending query failed (exit ${psResult.exitCode}): ${psResult.error ?? psResult.stderr}`,
      remediation: "Ensure PowerShell is available and unrestricted enough to read the registry.",
    });
    checks.push({
      id: "windows-installer-busy",
      status: "blocked",
      detail: "Could not determine installer-in-progress state because the registry query failed.",
    });
    return checks;
  }

  let parsed: {
    cbsRebootPending: boolean;
    wuRebootRequired: boolean;
    pendingFileRenames: boolean;
    installerInProgress: boolean;
    developerModeEnabled: boolean;
    developerModeKnown: boolean;
    longPathsEnabled: boolean;
    longPathsKnown: boolean;
  };
  try {
    parsed = JSON.parse(psResult.stdout.trim());
    if (typeof parsed !== "object" || parsed === null ||
        typeof parsed.cbsRebootPending !== "boolean" ||
        typeof parsed.wuRebootRequired !== "boolean" ||
        typeof parsed.pendingFileRenames !== "boolean" ||
        typeof parsed.installerInProgress !== "boolean" ||
        typeof parsed.developerModeEnabled !== "boolean" ||
        typeof parsed.developerModeKnown !== "boolean" ||
        typeof parsed.longPathsEnabled !== "boolean" ||
        typeof parsed.longPathsKnown !== "boolean") {
      throw new Error("unexpected shape");
    }
  } catch (e) {
    checks.push({
      id: "windows-reboot",
      status: "blocked",
      detail: `Registry query returned malformed output: ${psResult.stdout.trim()}`,
      remediation: "Investigate PowerShell execution policy or antivirus interference.",
    });
    checks.push({ id: "windows-installer-busy", status: "blocked", detail: "Could not parse installer state." });
    return checks;
  }

  const rebootRequired = parsed.cbsRebootPending || parsed.wuRebootRequired;
  if (rebootRequired) {
    const reasons: string[] = [];
    if (parsed.cbsRebootPending) { reasons.push("CBS component servicing"); }
    if (parsed.wuRebootRequired) { reasons.push("Windows Update"); }
    checks.push({
      id: "windows-reboot",
      status: "blocked",
      detail: `A system reboot is required before installation: ${reasons.join(", ")}.`,
      remediation: "Reboot Windows and retry.",
    });
  } else if (parsed.pendingFileRenames) {
    checks.push({
      id: "windows-reboot",
      status: "warning",
      detail: "Windows has queued file rename or deletion operations for a future restart. These can be unrelated to ROS/Pixi, so installation may proceed; restart if installation encounters file-in-use errors.",
      remediation: "Review the queued-operation warning. Restart Windows to apply the operations, or choose Proceed with Installation to continue.",
    });
  } else {
    checks.push({ id: "windows-reboot", status: "passed", detail: "No pending Windows Update or component servicing reboot detected." });
  }

  if (parsed.installerInProgress) {
    checks.push({
      id: "windows-installer-busy",
      status: "blocked",
      detail: "Windows Installer has another installation in progress.",
      remediation: "Wait for the current installation to finish, then retry.",
    });
  } else {
    checks.push({ id: "windows-installer-busy", status: "passed", detail: "No installer currently in progress." });
  }

  if (!parsed.developerModeKnown || !parsed.developerModeEnabled) {
    checks.push({
      id: "windows-developer-mode",
      status: "warning",
      detail: parsed.developerModeKnown
        ? "Windows Developer Mode is disabled. Enabling it can allow package tools to create symbolic links without elevation."
        : "Windows Developer Mode could not be determined.",
      remediation: "Recommended: search Windows Settings for 'For developers' and enable Developer Mode. Administrator approval or organization policy may be required.",
    });
  } else {
    checks.push({ id: "windows-developer-mode", status: "passed", detail: "Windows Developer Mode is enabled." });
  }

  if (!parsed.longPathsKnown || !parsed.longPathsEnabled) {
    checks.push({
      id: "windows-long-paths",
      status: "warning",
      detail: parsed.longPathsKnown
        ? "Win32 long-path support is disabled. Enabling it helps applications that opt in handle paths beyond the legacy MAX_PATH limit."
        : "Win32 long-path support could not be determined.",
      remediation: "Recommended: enable 'Enable Win32 long paths' in Local Group Policy (Computer Configuration > Administrative Templates > System > Filesystem), or set HKLM\\SYSTEM\\CurrentControlSet\\Control\\FileSystem\\LongPathsEnabled to 1. Administrator rights may be required; restart VS Code and related tools afterward.",
    });
  } else {
    checks.push({ id: "windows-long-paths", status: "passed", detail: "Win32 long-path support is enabled." });
  }

  return checks;
}

async function macosPreflight(arch: string, run: PreflightRunner): Promise<PreflightCheck[]> {
  const checks: PreflightCheck[] = [];

  if (arch !== "x64" && arch !== "arm64") {
    checks.push({
      id: "macos-arch",
      status: "blocked",
      detail: `Architecture '${arch}' is not supported on macOS; only x64 and arm64 are supported.`,
      remediation: "Use a macOS x64 or arm64 (Apple Silicon) machine.",
    });
    return checks;
  }
  checks.push({ id: "macos-arch", status: "passed", detail: `Architecture ${arch} supported.` });

  const verResult = await run("sw_vers", ["-productVersion"], 10_000);
  if (verResult.exitCode !== 0 || verResult.error) {
    checks.push({
      id: "macos-version",
      status: "blocked",
      detail: `Could not read macOS version: ${verResult.error ?? verResult.stderr}`,
      remediation: "Ensure sw_vers is available (standard macOS tool).",
    });
  } else {
    const version = verResult.stdout.trim();
    if (!/^\d+\.\d+(?:\.\d+)?$/.test(version)) {
      checks.push({
        id: "macos-version",
        status: "blocked",
        detail: `sw_vers output was unrecognizable: ${version}`,
      });
    } else {
      checks.push({ id: "macos-version", status: "passed",
        detail: `macOS ${version} detected. Package-specific OS requirements will be checked by the staged Pixi solve.` });
    }
  }

  const xcodeResult = await run("xcode-select", ["-p"], 10_000);
  if (xcodeResult.exitCode !== 0 || xcodeResult.error || !xcodeResult.stdout.trim()) {
    checks.push({
      id: "macos-build-tools",
      status: "warning",
      detail: "Xcode Command Line Tools could not be confirmed; C/C++ build tools may be unavailable.",
      remediation: "Run: xcode-select --install",
    });
  } else {
    checks.push({
      id: "macos-build-tools",
      status: "passed",
      detail: `Xcode Command Line Tools found at ${xcodeResult.stdout.trim()}.`,
    });
  }

  return checks;
}

export async function nativePreflight(platform: NodeJS.Platform, arch: string, run: PreflightRunner): Promise<PreflightCheck[]> {
  if (platform === "win32") {
    return windowsPreflight(arch, run);
  }
  if (platform === "darwin") {
    return macosPreflight(arch, run);
  }
  // Other platforms are handled by the parent (Linux) or are unsupported.
  return [];
}
