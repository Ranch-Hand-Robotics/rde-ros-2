// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import * as yaml from "js-yaml";
import { HealthTarget, runHealthProcess } from "./health-check";
import { nativePreflight } from "./native-preflight";
import { PreflightCheck, PreflightReport, PreflightRunner } from "./preflight-types";

const gib = 1024 ** 3;

export interface PathReadiness {
  exists: boolean;
  directory: boolean;
  symbolicLink: boolean;
  entries: string[];
  freeBytes: number;
  freeInodes?: number;
  writable: boolean;
}

export interface PreflightServices {
  platform: NodeJS.Platform;
  arch: string;
  run: PreflightRunner;
  readFile: (filename: string) => Promise<string | undefined>;
  inspectPath: (filename: string) => Promise<PathReadiness>;
  ubuntuPlatforms: (distro: string) => Promise<string[]>;
}

async function readOptionalFile(filename: string): Promise<string | undefined> {
  try {
    return await fs.promises.readFile(filename, "utf8");
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code === "ENOENT") {
      return undefined;
    }
    throw error;
  }
}

async function inspectPath(filename: string): Promise<PathReadiness> {
  let existing = filename;
  let stat: fs.Stats | undefined;
  while (!stat) {
    try {
      stat = await fs.promises.lstat(existing);
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code !== "ENOENT" || path.dirname(existing) === existing) {
        throw error;
      }
      existing = path.dirname(existing);
    }
  }
  const disk = await fs.promises.statfs(existing);
  let writable = true;
  try {
    await fs.promises.access(existing, fs.constants.W_OK | fs.constants.X_OK);
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code !== "EACCES" && (error as NodeJS.ErrnoException).code !== "EPERM") {
      throw error;
    }
    writable = false;
  }
  return {
    exists: existing === filename,
    directory: stat.isDirectory(),
    symbolicLink: stat.isSymbolicLink(),
    entries: existing === filename && stat.isDirectory() ? await fs.promises.readdir(existing) : [],
    freeBytes: disk.bavail * disk.bsize,
    freeInodes: disk.files > 0 ? disk.ffree : undefined,
    writable,
  };
}

async function ubuntuPlatforms(distro: string): Promise<string[]> {
  const stable: Record<string, string[]> = { humble: ["jammy"], jazzy: ["noble"], kilted: ["noble"] };
  if (stable[distro]) {
    return stable[distro];
  }
  // Rolling changes targets over time; newer distros must be confirmed rather than guessed.
  const response = await fetch(`https://raw.githubusercontent.com/ros/rosdistro/master/${distro}/distribution.yaml`, {
    signal: AbortSignal.timeout(15000),
  });
  if (!response.ok) {
    throw new Error(`ROS platform metadata returned HTTP ${response.status}.`);
  }
  const text = await response.text();
  if (text.length > 2 * 1024 * 1024) {
    throw new Error("ROS platform metadata exceeded the size limit.");
  }
  const metadata = yaml.load(text) as { release_platforms?: { ubuntu?: unknown } };
  const platforms = metadata?.release_platforms?.ubuntu;
  if (!Array.isArray(platforms) || platforms.length === 0 ||
      !platforms.every((value) => typeof value === "string" && /^[a-z]+$/.test(value))) {
    throw new Error("ROS metadata does not identify supported Ubuntu releases.");
  }
  return platforms;
}

export const runPreflightCommand: PreflightRunner = (command, args, limitMs = 15000) => runHealthProcess({
  command, args, cwd: os.tmpdir(), env: { ...process.env, LC_ALL: "C", LANG: "C" },
}, limitMs);

const defaults: PreflightServices = {
  platform: process.platform, arch: process.arch, run: runPreflightCommand,
  readFile: readOptionalFile, inspectPath, ubuntuPlatforms,
};

export function parseOsRelease(text: string): Record<string, string> {
  const values: Record<string, string> = {};
  for (const line of text.split(/\r?\n/)) {
    const match = /^([A-Z_]+)=(?:"([^"]*)"|'([^']*)'|([^#\s]*))$/.exec(line);
    if (match) {
      values[match[1]] = match[2] ?? match[3] ?? match[4];
    }
  }
  return values;
}

/** No sudo, package changes, repository writes, manifest writes, or automatic repair. */
export async function preflightInstallation(
  target: HealthTarget,
  services: PreflightServices = defaults
): Promise<PreflightReport> {
  const checks: PreflightCheck[] = [];
  const add = (id: string, status: PreflightCheck["status"], detail: string, remediation?: string) =>
    checks.push({ id, status, detail, remediation });
  const guarded = async (id: string, action: () => Promise<void>) => {
    try {
      await action();
    } catch (error) {
      add(id, "blocked", `Readiness could not be determined: ${String(error)}`, "Resolve this diagnostic error and rerun installation. No repair was attempted.");
    }
  };
  if (!target || !/^[a-z]+$/.test(target.distro) || !["setup", "pixi"].includes(target.kind)) {
    return { ready: false, checks: [{ id: "target", status: "blocked", detail: "Invalid installation target." }] };
  }
  const targetDirectory = target.kind === "pixi" ? target.workspace : path.dirname(target.setupScript);
  if (!path.isAbsolute(targetDirectory)) {
    return { ready: false, checks: [{ id: "target", status: "blocked", detail: "The installation target must be an absolute path." }] };
  }
  await guarded("target", async () => {
    const current = await services.inspectPath(targetDirectory);
    if (current.exists && (current.symbolicLink || !current.directory || current.entries.length > 0)) {
      add("target", "blocked", `The installation target already contains files or is a link: ${targetDirectory}`,
        "Run Check ROS 2 Installation Health for an existing installation. For Pixi, choose a different root. Back up and resolve partial/unrelated targets manually; the installer will not overwrite them.");
    } else {
      add("target", "passed", `Unused target: ${targetDirectory}`);
    }
    if (target.kind === "pixi" && !current.writable) {
      add("target-permissions", "blocked", "The target or its nearest existing parent is not writable/searchable.",
        "Choose a writable Pixi root or correct its permissions.");
    }
    add("target-space", current.freeBytes >= 8 * gib ? "passed" : "blocked",
      `${(current.freeBytes / gib).toFixed(1)} GiB available; installer requires at least 8 GiB on the target filesystem.`,
      current.freeBytes >= 8 * gib ? undefined : "Free space or choose another target. This is a conservative floor, not an exact package-size estimate.");
    if (current.freeInodes !== undefined && current.freeInodes < 20000) {
      add("target-inodes", "blocked", `Only ${current.freeInodes} filesystem inodes remain.`, "Free at least 20000 inodes or choose a different filesystem.");
    }
    const parent = await services.inspectPath(path.dirname(targetDirectory));
    const others = parent.entries.filter((entry) => entry !== target.distro);
    if (others.length) {
      add("other-installations", "warning", `Other entries exist alongside the target: ${others.slice(0, 20).join(", ")}`,
        "Other ROS distributions may coexist. Confirm shared package changes will not affect environments you depend on.");
    }
  });
  await guarded("temporary-space", async () => {
    const temporary = await services.inspectPath(os.tmpdir());
    add("temporary-space", temporary.writable && temporary.freeBytes >= gib ? "passed" : "blocked",
      `${(temporary.freeBytes / gib).toFixed(1)} GiB available in the temporary filesystem; at least 1 GiB and write access are required.`,
      temporary.writable && temporary.freeBytes >= gib ? undefined : "Free temporary space or correct temporary-directory permissions.");
  });

  if (services.platform === "linux" && target.kind === "setup") {
    await guarded("ubuntu-compatibility", async () => {
      const release = parseOsRelease(await services.readFile("/etc/os-release") ?? "");
      const supported = await services.ubuntuPlatforms(target.distro);
      const compatible = release.ID === "ubuntu" && supported.includes(release.UBUNTU_CODENAME || release.VERSION_CODENAME) &&
        ["x64", "arm64"].includes(services.arch);
      add("ubuntu-compatibility", compatible ? "passed" : "blocked",
        `${target.distro} requires Ubuntu ${supported.join("/")} on x64/arm64; found ${release.ID || "unknown"} ${release.VERSION_ID || "unknown"} (${services.arch}).`,
        compatible ? undefined : "Choose a matching ROS distribution or a supported Ubuntu image. Do not perform an in-place OS upgrade through this installer.");
      if (await services.readFile("/etc/nv_tegra_release")) {
        add("jetpack", "warning", "Jetson/L4T detected. Ubuntu compatibility is checked, but JetPack/CUDA compatibility is not guaranteed.",
          "Preserve your JetPack baseline and review proposed package upgrades before proceeding.");
      }
    });
    await guarded("reboot-required", async () => {
      const reboot = await services.readFile("/var/run/reboot-required");
      add("reboot-required", reboot === undefined ? "passed" : "blocked",
        reboot === undefined ? "No Ubuntu reboot-required marker." : "Ubuntu reports a pending reboot.",
        reboot === undefined ? undefined : "Restart the target system (or WSL instance as appropriate), then rerun preflight.");
    });
    await guarded("apt-readiness", async () => {
      for (const executable of ["sudo", "apt-get", "apt-cache", "dpkg", "dpkg-query", "apt-mark", "ps"]) {
        const available = await services.run("/bin/bash", ["--noprofile", "--norc", "-c", 'command -v "$1"', "preflight", executable]);
        if (available.error || available.exitCode !== 0) {
          add("apt-tools", "blocked", `Required tool ${executable} is unavailable. ${available.error || available.stderr}`,
            "Install or repair the Ubuntu package-management prerequisites manually.");
          return;
        }
      }
      const processes = await services.run("ps", ["-eo", "comm=,args="]);
      const busy = processes.stdout.split(/\r?\n/).filter((line) => {
        const name = line.trim().split(/\s+/)[0];
        const shutdownWaiter = /\bunattended-upgrade-shutdown\s+--wait-for-signal(?:\s|$)/.test(line);
        return /^(apt|apt-get|dpkg|unattended-upgr.*)$/.test(name) && !shutdownWaiter;
      });
      const unknownProcesses = processes.error || processes.exitCode !== 0;
      add("package-manager-busy", unknownProcesses || busy.length ? "blocked" : "passed",
        unknownProcesses ? `Unable to inspect running package operations: ${processes.error || processes.stderr}`
          : busy.length ? `Package operations are running: ${busy.join(", ")}` : "No active APT/dpkg/unattended-upgrade process detected.",
        unknownProcesses || busy.length ? "Wait for package operations to finish, or resolve process-inspection permissions. Do not delete lock files or kill a package manager mid-operation." : undefined);
      const installed = await services.run("dpkg-query", ["-W", "-f=${binary:Package}\\t${db:Status-Status}\\n", `ros-${target.distro}-*`]);
      if (/\tinstalled$/m.test(installed.stdout)) {
        add("existing-ros-packages", "blocked", `ROS ${target.distro} packages are already installed:\n${installed.stdout}`,
          "Check the existing installation's health. Use your package manager for an intentional repair or upgrade; this installer only creates new targets.");
      } else if (installed.error || (installed.exitCode !== 0 &&
          !(installed.exitCode === 1 && !installed.stdout.trim() && /no packages found matching/i.test(installed.stderr)))) {
        add("existing-ros-packages", "blocked", installed.error || installed.stderr, "Resolve package-database access before retrying.");
      }
      const audit = await services.run("dpkg", ["--audit"]);
      const auditFailed = audit.error || audit.exitCode !== 0 || audit.stdout.trim() || audit.stderr.trim();
      add("dpkg-audit", auditFailed ? "blocked" : "passed", auditFailed
        ? `dpkg reports an incomplete or damaged package state:\n${audit.error || ""}\n${audit.stdout}\n${audit.stderr}`
        : "dpkg reports no incomplete package installations.",
        auditFailed ? "Inspect dpkg --audit and package-manager history. Repair interrupted packages manually before installing ROS." : undefined);
      const dependencies = await services.run("apt-get", ["--simulate", "check"], 30000);
      const broken = dependencies.error || dependencies.exitCode !== 0;
      add("apt-dependencies", broken ? "blocked" : "passed",
        broken ? `APT dependency checking failed:\n${dependencies.error || ""}\n${dependencies.stdout}\n${dependencies.stderr}` : "APT dependency check passed without making changes.",
        broken ? "Resolve broken dependencies, package-manager locks or repository errors before retrying. No fix-broken operation was run." : undefined);
      const held = await services.run("apt-mark", ["showhold"]);
      if (held.error || held.exitCode !== 0) {
        add("held-packages", "blocked", held.error || held.stderr || "Could not inspect held packages.", "Repair package-manager access before retrying.");
      } else {
        add("held-packages", held.stdout.trim() ? "warning" : "passed",
          held.stdout.trim() ? `Held packages:\n${held.stdout}` : "No held packages.",
          held.stdout.trim() ? "Keep intentional holds. Any solver conflict involving held packages must be resolved manually." : undefined);
      }
      const candidate = await services.run("apt-cache", ["policy", `ros-${target.distro}-desktop`]);
      if (candidate.error || candidate.exitCode !== 0) {
        add("apt-candidate", "blocked", candidate.error || candidate.stderr || "Unable to inspect ROS package candidates.", "Repair APT metadata before retrying.");
      } else if (/Candidate:\s+(?!\(none\))\S+/.test(candidate.stdout)) {
        const plan = await services.run("apt-get", ["--simulate", "--no-remove", "install", `ros-${target.distro}-desktop`], 30000);
        const conflict = plan.error || plan.exitCode !== 0 || /^Remv /m.test(plan.stdout);
        add("apt-plan", conflict ? "blocked" : "warning",
          `Simulated ROS package changes (no packages changed):\n${plan.error || ""}\n${plan.stdout}\n${plan.stderr}`,
          conflict ? "Resolve the dependency conflict or required removals before retrying." : "Review proposed installations and upgrades. Existing repository metadata may be stale.");
      } else {
        add("apt-plan", "warning", "The ROS package is not yet in local APT metadata. Full dependency resolution is deferred until repository setup.",
          "Compatibility and existing package health were checked, but a successful dependency solve cannot yet be promised.");
      }
      const cache = await services.inspectPath("/var/cache/apt/archives");
      add("apt-cache-space", cache.freeBytes >= 2 * gib ? "passed" : "blocked",
        `${(cache.freeBytes / gib).toFixed(1)} GiB free on the APT cache filesystem; at least 2 GiB is required.`,
        cache.freeBytes >= 2 * gib ? undefined : "Free package-cache space before retrying.");
    });
  } else if (target.kind === "pixi" && ["win32", "darwin"].includes(services.platform)) {
    await guarded("native-host", async () => {
      checks.push(...await nativePreflight(services.platform, services.arch, services.run));
    });
    await guarded("pixi-readiness", async () => {
      const pixi = await services.run("pixi", ["--version"]);
      if (pixi.error && /ENOENT|not found/i.test(pixi.error)) {
        const tool = services.platform === "win32" ? "winget.exe" : "curl";
        const prerequisite = await services.run(tool, ["--version"]);
        const missing = prerequisite.error || prerequisite.exitCode !== 0;
        add("pixi-bootstrap", missing ? "blocked" : "warning",
          missing ? `Pixi and its bootstrap prerequisite ${tool} are unavailable.` : "Pixi is not installed. It will require explicit bootstrap approval.",
          missing ? `Install or repair ${tool} before retrying.` : "Dependency solving will run after Pixi bootstrap, before creating the ROS workspace.");
      } else if (pixi.error || pixi.exitCode !== 0) {
        add("pixi-readiness", "blocked", `Pixi is present but cannot run: ${pixi.error || pixi.stderr}`, "Repair Pixi before retrying.");
      } else {
        add("pixi-readiness", "passed", pixi.stdout.trim());
      }
    });
  } else {
    add("platform", "blocked", `Unsupported installer target on ${services.platform}.`);
  }
  return { ready: !checks.some((check) => check.status === "blocked"), checks };
}
