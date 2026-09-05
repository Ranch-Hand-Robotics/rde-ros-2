// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as os from "os";
import { preflightInstallation, PreflightServices, PathReadiness, parseOsRelease } from "../../src/ros/installer/install-preflight";
import { HealthTarget } from "../../src/ros/installer/health-check";

describe("ROS installation preflight", () => {
  let services: PreflightServices;
  let target: HealthTarget;
  let files: Record<string, string>;
  let paths: Record<string, Partial<PathReadiness>>;
  let commands: string[];

  beforeEach(() => {
    target = { kind: "setup", distro: "jazzy", setupScript: "/opt/ros/jazzy/setup.bash" };
    files = { "/etc/os-release": 'ID=ubuntu\nVERSION_ID="24.04"\nVERSION_CODENAME=noble\n' };
    paths = {};
    commands = [];
    services = {
      platform: "linux", arch: "x64",
      readFile: async (filename) => files[filename],
      inspectPath: async (filename) => ({
        exists: false, directory: true, symbolicLink: false, entries: [],
        freeBytes: 20 * 1024 ** 3, freeInodes: 1000000, writable: true,
        ...paths[filename],
      }),
      ubuntuPlatforms: async () => ["noble"],
      run: async (command, args) => {
        commands.push([command, ...args].join(" "));
        return { exitCode: 0, stdout: "", stderr: "" };
      },
    };
  });

  it("accepts a healthy new target without running any mutating command", async () => {
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, true, JSON.stringify(report));
    assert.ok(report.checks.some((check) => check.id === "apt-plan" && check.status === "warning"));
    assert.strictEqual(new Set(report.checks.map((check) => check.id)).size, report.checks.length);
    assert.strictEqual(commands.filter((command) => command.startsWith("ps ")).length, 1);
    assert.ok(commands.includes("apt-get --simulate check"));
    assert.ok(!commands.some((command) => /^sudo |apt-get update|--fix-broken|dpkg --configure/.test(command)));
  });

  it("blocks incomplete dpkg state and provides repair guidance", async () => {
    const original = services.run;
    services.run = (command, args) => command === "dpkg"
      ? Promise.resolve({ exitCode: 0, stdout: "Packages are unpacked but not configured: libc6", stderr: "" })
      : original(command, args);
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    const check = report.checks.find((entry) => entry.id === "dpkg-audit");
    assert.strictEqual(check.status, "blocked");
    assert.match(check.remediation, /manually/);
  });

  it("blocks broken dependencies, even with a successful package audit", async () => {
    const original = services.run;
    services.run = (command, args) => command === "apt-get"
      ? Promise.resolve({ exitCode: 100, stdout: "", stderr: "Unmet dependencies" })
      : original(command, args);
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.match(report.checks.find((entry) => entry.id === "apt-dependencies").detail, /Unmet dependencies/);
  });

  it("blocks pending reboot, wrong Ubuntu release and unsupported architecture", async () => {
    files["/var/run/reboot-required"] = "*** System restart required ***";
    files["/etc/os-release"] = 'ID=ubuntu\nVERSION_ID="22.04"\nVERSION_CODENAME=jammy\n';
    services.arch = "ia32";
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((entry) => entry.id === "ubuntu-compatibility").status, "blocked");
    assert.strictEqual(report.checks.find((entry) => entry.id === "reboot-required").status, "blocked");
  });

  it("blocks active package-manager processes rather than removing locks", async () => {
    const original = services.run;
    services.run = (command, args) => command === "ps"
      ? Promise.resolve({ exitCode: 0, stdout: "systemd\napt-get\ndpkg\n", stderr: "" })
      : original(command, args);
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.match(report.checks.find((entry) => entry.id === "package-manager-busy").remediation, /Do not delete lock files/);
  });

  it("accepts Ubuntu's idle unattended-upgrade shutdown waiter, but blocks actual upgrades", async () => {
    let argumentsLine = "/usr/bin/python3 /usr/share/unattended-upgrades/unattended-upgrade-shutdown --wait-for-signal";
    const original = services.run;
    services.run = (command, args) => command === "ps"
      ? Promise.resolve({ exitCode: 0, stdout: `unattended-upgr ${argumentsLine}\n`, stderr: "" })
      : original(command, args);
    let report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, true, JSON.stringify(report));
    argumentsLine = "/usr/bin/python3 /usr/bin/unattended-upgrade";
    report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
  });

  it("does not overwrite an existing or symlinked target, but permits other distros alongside it", async () => {
    paths["/opt/ros"] = { exists: true, entries: ["humble"] };
    let report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, true);
    assert.strictEqual(report.checks.find((entry) => entry.id === "other-installations").status, "warning");
    paths["/opt/ros/jazzy"] = { exists: true, entries: ["setup.bash"] };
    report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    paths["/opt/ros/jazzy"] = { exists: true, symbolicLink: true };
    report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
  });

  it("blocks existing ROS packages even if their installation files have disappeared", async () => {
    const original = services.run;
    services.run = (command, args) => command === "dpkg-query"
      ? Promise.resolve({ exitCode: 0, stdout: "ros-jazzy-rclpy\tinstalled\n", stderr: "" })
      : original(command, args);
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((entry) => entry.id === "existing-ros-packages").status, "blocked");
  });

  it("blocks low disk space and inode exhaustion", async () => {
    paths["/opt/ros/jazzy"] = { freeBytes: 1024, freeInodes: 10 };
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((entry) => entry.id === "target-space").status, "blocked");
    assert.strictEqual(report.checks.find((entry) => entry.id === "target-inodes").status, "blocked");
  });

  it("accepts the documented resource boundaries and rejects one byte below the target floor", async () => {
    paths["/opt/ros/jazzy"] = { freeBytes: 8 * 1024 ** 3, freeInodes: 20000 };
    paths[os.tmpdir()] = { freeBytes: 1024 ** 3 };
    paths["/var/cache/apt/archives"] = { freeBytes: 2 * 1024 ** 3 };
    let report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, true);
    paths["/opt/ros/jazzy"].freeBytes--;
    report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((entry) => entry.id === "target-space").status, "blocked");
  });

  it("simulates candidates with no removals and blocks a solver conflict", async () => {
    const original = services.run;
    services.run = async (command, args) => {
      if (command === "apt-cache") {
        return { exitCode: 0, stdout: "  Candidate: 0.11.0\n", stderr: "" };
      }
      if (command === "apt-get" && args.includes("install")) {
        assert.ok(args.includes("--simulate"));
        assert.ok(args.includes("--no-remove"));
        return { exitCode: 100, stdout: "Remv critical-package\n", stderr: "held broken packages" };
      }
      return original(command, args);
    };
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.match(report.checks.find((entry) => entry.id === "apt-plan").detail, /held broken packages/);
  });

  it("does not treat unknown readiness or a timed-out command as success", async () => {
    services.ubuntuPlatforms = async () => { throw new Error("metadata unavailable"); };
    services.run = async () => ({ exitCode: null, stdout: "", stderr: "", error: "timed out" });
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((entry) => entry.id === "ubuntu-compatibility").status, "blocked");
    assert.strictEqual(report.checks.find((entry) => entry.id === "apt-tools").status, "blocked");
  });

  it("warns about held packages and JetPack instead of changing them automatically", async () => {
    files["/etc/nv_tegra_release"] = "# R36 (release), REVISION: 4.0";
    const original = services.run;
    services.run = (command, args) => command === "apt-mark"
      ? Promise.resolve({ exitCode: 0, stdout: "nvidia-l4t-core\n", stderr: "" })
      : original(command, args);
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, true);
    assert.strictEqual(report.checks.find((entry) => entry.id === "held-packages").status, "warning");
    assert.strictEqual(report.checks.find((entry) => entry.id === "jetpack").status, "warning");
  });

  it("reads os-release as data, never shell source", () => {
    assert.deepStrictEqual(parseOsRelease("ID=ubuntu\nVERSION_ID=\"24.04\"\nEVIL=$(touch /tmp/file)\n"),
      { ID: "ubuntu", VERSION_ID: "24.04" });
  });

  it("blocks a Windows pending reboot before allowing Pixi bootstrap", async () => {
    services.platform = "win32";
    target = { kind: "pixi", distro: "jazzy", workspace: "/pixi_ws/jazzy" };
    services.run = async (command) => {
      commands.push(command);
      if (command === "cmd.exe") {
        return { exitCode: 0, stdout: "Microsoft Windows [Version 10.0.19045]", stderr: "" };
      }
      if (command === "powershell.exe") {
        return { exitCode: 0, stderr: "", stdout: JSON.stringify({
          cbsRebootPending: false, wuRebootRequired: true, pendingFileRenames: false, installerInProgress: false,
        }) };
      }
      if (command === "pixi") {
        return { exitCode: null, stdout: "", stderr: "", error: "ENOENT" };
      }
      return { exitCode: 0, stdout: "version", stderr: "" };
    };
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((check) => check.id === "windows-reboot").status, "blocked");
    assert.strictEqual(report.checks.find((check) => check.id === "pixi-bootstrap").status, "warning");
    assert.ok(!commands.some((command) => command.includes("install")));
  });

  it("blocks a broken Pixi executable rather than attempting to bootstrap over it", async () => {
    services.platform = "darwin";
    target = { kind: "pixi", distro: "jazzy", workspace: "/pixi_ws/jazzy" };
    services.run = async (command) => {
      commands.push(command);
      return command === "pixi"
        ? { exitCode: 1, stdout: "", stderr: "broken dynamic library" }
        : { exitCode: 0, stdout: command === "sw_vers" ? "14.2" : "/Library/Developer/CommandLineTools", stderr: "" };
    };
    const report = await preflightInstallation(target, services);
    assert.strictEqual(report.ready, false);
    assert.strictEqual(report.checks.find((check) => check.id === "pixi-readiness").status, "blocked");
    assert.ok(!commands.includes("curl"));
  });
});
