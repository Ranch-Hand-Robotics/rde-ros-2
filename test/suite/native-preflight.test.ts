// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import { nativePreflight } from "../../src/ros/installer/native-preflight";
import { PreflightRunner, PreflightCommandResult } from "../../src/ros/installer/preflight-types";

function makeRunner(responses: Record<string, PreflightCommandResult>): PreflightRunner {
  return async (command, args) => {
    const key = [command, ...args].join(" ");
    // match by command name prefix
    for (const [k, v] of Object.entries(responses)) {
      if (key.startsWith(k) || command === k) {
        return v;
      }
    }
    return { exitCode: 0, stdout: "", stderr: "" };
  };
}

const HEALTHY_WIN_PS_OUTPUT = JSON.stringify({
  cbsRebootPending: false,
  wuRebootRequired: false,
  pendingFileRenames: false,
  installerInProgress: false,
});

const WIN_VER_OUTPUT = "Microsoft Windows [Version 10.0.19045.4651]";

describe("nativePreflight - Windows", () => {
  it("passes on a healthy Windows 10 x64 system", async () => {
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: HEALTHY_WIN_PS_OUTPUT, stderr: "" },
    });

    it("disables cmd AutoRun and uses strict read-only registry queries including the MSI subkey", async () => {
      const run: PreflightRunner = async (command, args) => {
        if (command === "cmd.exe") {
          assert.deepStrictEqual(args, ["/d", "/c", "ver"]);
          return { exitCode: 0, stdout: "Microsoft Windows [VersionLocale 10.0.19045.4651]", stderr: "" };
        }
        const script = args[args.length - 1];
        assert.ok(script.includes("$ErrorActionPreference = 'Stop'"));
        assert.ok(script.includes("Installer\\InProgress"));
        assert.ok(script.includes("PSObject.Properties['PendingFileRenameOperations']"));
        assert.ok(!script.includes("SilentlyContinue"));
        assert.ok(!script.includes("catch {}"));
        return { exitCode: 0, stdout: HEALTHY_WIN_PS_OUTPUT, stderr: "" };
      };
      const checks = await nativePreflight("win32", "x64", run);
      assert.ok(checks.every((check) => check.status === "passed"));
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.ok(checks.every(c => c.status === "passed"), JSON.stringify(checks));
  });

  it("blocks unsupported architecture immediately without running other checks", async () => {
    let ran = false;
    const run: PreflightRunner = async () => { ran = true; return { exitCode: 0, stdout: "", stderr: "" }; };
    const checks = await nativePreflight("win32", "arm64", run);
    assert.ok(!ran, "should not run any commands for unsupported arch");
    assert.strictEqual(checks.length, 1);
    assert.strictEqual(checks[0].id, "windows-arch");
    assert.strictEqual(checks[0].status, "blocked");
  });

  it("blocks when CBS reboot-pending flag is set", async () => {
    const ps = JSON.stringify({ cbsRebootPending: true, wuRebootRequired: false, pendingFileRenames: false, installerInProgress: false });
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: ps, stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    const reboot = checks.find(c => c.id === "windows-reboot");
    assert.strictEqual(reboot?.status, "blocked");
    assert.match(reboot.detail, /CBS/);
  });

  it("blocks when Windows Update reboot is required", async () => {
    const ps = JSON.stringify({ cbsRebootPending: false, wuRebootRequired: true, pendingFileRenames: false, installerInProgress: false });
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: ps, stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-reboot")?.status, "blocked");
  });

  it("blocks when PendingFileRenameOperations is non-empty", async () => {
    const ps = JSON.stringify({ cbsRebootPending: false, wuRebootRequired: false, pendingFileRenames: true, installerInProgress: false });
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: ps, stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-reboot")?.status, "blocked");
  });

  it("blocks when Windows Installer is in progress", async () => {
    const ps = JSON.stringify({ cbsRebootPending: false, wuRebootRequired: false, pendingFileRenames: false, installerInProgress: true });
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: ps, stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-installer-busy")?.status, "blocked");
    assert.strictEqual(checks.find(c => c.id === "windows-reboot")?.status, "passed");
  });

  it("blocks on malformed PowerShell output (not silently pass)", async () => {
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: "not json at all", stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-reboot")?.status, "blocked");
    assert.strictEqual(checks.find(c => c.id === "windows-installer-busy")?.status, "blocked");
  });

  it("blocks when PowerShell exits with non-zero code", async () => {
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: 1, stdout: "", stderr: "Access denied" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-reboot")?.status, "blocked");
  });

  it("blocks when PowerShell times out (exitCode null with error)", async () => {
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: WIN_VER_OUTPUT, stderr: "" },
      "powershell.exe": { exitCode: null, stdout: "", stderr: "", error: "timed out" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-reboot")?.status, "blocked");
  });

  it("blocks on Windows version below 10", async () => {
    const run = makeRunner({
      "cmd.exe": { exitCode: 0, stdout: "Microsoft Windows [Version 6.1.7601]", stderr: "" },
      "powershell.exe": { exitCode: 0, stdout: HEALTHY_WIN_PS_OUTPUT, stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-version")?.status, "blocked");
  });

  it("blocks when ver command fails", async () => {
    const run = makeRunner({
      "cmd.exe": { exitCode: 1, stdout: "", stderr: "error" },
      "powershell.exe": { exitCode: 0, stdout: HEALTHY_WIN_PS_OUTPUT, stderr: "" },
    });
    const checks = await nativePreflight("win32", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "windows-version")?.status, "blocked");
  });

  it("blocks when spawn fails (error field set)", async () => {
    const run: PreflightRunner = async () => ({ exitCode: null, stdout: "", stderr: "", error: "ENOENT" });
    const checks = await nativePreflight("win32", "x64", run);
    assert.ok(checks.some(c => c.status === "blocked"), JSON.stringify(checks));
  });
});

describe("nativePreflight - macOS", () => {
  it("passes on a healthy macOS 14 arm64 system with Xcode CLT", async () => {
    const run = makeRunner({
      "sw_vers": { exitCode: 0, stdout: "14.4.1", stderr: "" },
      "xcode-select": { exitCode: 0, stdout: "/Library/Developer/CommandLineTools", stderr: "" },
    });
    const checks = await nativePreflight("darwin", "arm64", run);
    assert.ok(checks.every(c => c.status === "passed"), JSON.stringify(checks));
  });

  it("passes on macOS x64", async () => {
    const run = makeRunner({
      "sw_vers": { exitCode: 0, stdout: "13.6.0", stderr: "" },
      "xcode-select": { exitCode: 0, stdout: "/Library/Developer/CommandLineTools", stderr: "" },
    });
    const checks = await nativePreflight("darwin", "x64", run);
    assert.ok(checks.every(c => c.status === "passed"), JSON.stringify(checks));
  });

  it("blocks unsupported architecture", async () => {
    const run: PreflightRunner = async () => ({ exitCode: 0, stdout: "", stderr: "" });
    const checks = await nativePreflight("darwin", "ia32", run);
    assert.strictEqual(checks.length, 1);
    assert.strictEqual(checks[0].id, "macos-arch");
    assert.strictEqual(checks[0].status, "blocked");
  });

  it("leaves package-specific macOS version requirements to the Pixi solver", async () => {
    const run = makeRunner({
      "sw_vers": { exitCode: 0, stdout: "10.15.7", stderr: "" },
      "xcode-select": { exitCode: 0, stdout: "/Library/Developer/CommandLineTools", stderr: "" },
    });
    const checks = await nativePreflight("darwin", "x64", run);
    assert.strictEqual(checks.find(c => c.id === "macos-version")?.status, "passed");
    assert.match(checks.find(c => c.id === "macos-version").detail, /staged Pixi solve/);
  });

  it("warns (not blocks) when Xcode CLT is missing", async () => {
    const run = makeRunner({
      "sw_vers": { exitCode: 0, stdout: "14.0", stderr: "" },
      "xcode-select": { exitCode: 2, stdout: "", stderr: "xcode-select: error: unable to get active developer directory" },
    });
    const checks = await nativePreflight("darwin", "arm64", run);
    const buildTool = checks.find(c => c.id === "macos-build-tools");
    assert.strictEqual(buildTool?.status, "warning");
    assert.strictEqual(checks.find(c => c.id === "macos-version")?.status, "passed");
  });

  it("blocks on unrecognizable sw_vers output", async () => {
    const run = makeRunner({
      "sw_vers": { exitCode: 0, stdout: "banana", stderr: "" },
      "xcode-select": { exitCode: 0, stdout: "/Library/Developer/CommandLineTools", stderr: "" },
    });
    const checks = await nativePreflight("darwin", "arm64", run);
    assert.strictEqual(checks.find(c => c.id === "macos-version")?.status, "blocked");
  });

  it("blocks when sw_vers fails to spawn", async () => {
    const run = makeRunner({
      "sw_vers": { exitCode: null, stdout: "", stderr: "", error: "ENOENT" },
      "xcode-select": { exitCode: 0, stdout: "/Library/Developer/CommandLineTools", stderr: "" },
    });
    const checks = await nativePreflight("darwin", "arm64", run);
    assert.strictEqual(checks.find(c => c.id === "macos-version")?.status, "blocked");
  });
});

describe("nativePreflight - other platforms", () => {
  it("returns an empty array for linux (handled by parent)", async () => {
    const run: PreflightRunner = async () => ({ exitCode: 0, stdout: "", stderr: "" });
    const checks = await nativePreflight("linux", "x64", run);
    assert.deepStrictEqual(checks, []);
  });
});
