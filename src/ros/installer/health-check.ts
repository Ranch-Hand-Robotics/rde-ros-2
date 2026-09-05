// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import { execFile, spawn } from "child_process";
import * as fs from "fs";
import * as path from "path";

export type HealthTarget =
  | { kind: "setup"; distro: string; setupScript: string; pythonExecutable?: string }
  | { kind: "pixi"; distro: string; workspace: string };

export interface HealthCheck {
  id: string;
  status: "passed" | "failed";
  detail: string;
  durationMs: number;
}

export interface HealthReport {
  target: HealthTarget;
  healthy: boolean;
  checks: HealthCheck[];
}

export interface HealthCommand {
  command: string;
  args: string[];
  cwd: string;
  env: NodeJS.ProcessEnv;
  windowsVerbatimArguments?: boolean;
}

export interface HealthProcessResult {
  stdout: string;
  stderr: string;
  exitCode: number | null;
  signal?: string;
  error?: string;
}

export const HEALTH_SENTINEL = "RDE_ROS_HEALTH:";
const requiredChecks = [
  "environment", "prefix", "ros2-cli", "package-discovery", "rclpy-import", "rmw-init", "pub-sub",
];
const outputLimit = 128 * 1024;
const timeoutMs = 45000;

/** Start with only OS essentials: never reuse the extension's activated ROS/Python environment. */
function cleanEnvironment(platform: NodeJS.Platform, inherited: NodeJS.ProcessEnv): NodeJS.ProcessEnv {
  const env: NodeJS.ProcessEnv = {};
  const allowed = /^(HOME|USER|LOGNAME|USERPROFILE|HOMEDRIVE|HOMEPATH|SYSTEMROOT|WINDIR|COMSPEC|TEMP|TMP|TMPDIR|LANG|LC_ALL|LC_CTYPE|LOCALAPPDATA|APPDATA|PROGRAMDATA)$/i;
  for (const key of Object.keys(inherited)) {
    if (allowed.test(key)) {
      env[key] = inherited[key];
    }
  }
  const systemRoot = inherited.SystemRoot || inherited.SYSTEMROOT || "C:\\Windows";
  env.PATH = platform === "win32"
    ? [path.win32.join(systemRoot, "System32"), systemRoot].join(";")
    : "/usr/bin:/bin:/usr/sbin:/sbin";
  env.PYTHONNOUSERSITE = "1";
  env.PYTHONDONTWRITEBYTECODE = "1";
  env.ROS_LOCALHOST_ONLY = "1";
  env.ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST";
  env.ROS_STATIC_PEERS = "";
  return env;
}

async function requireFile(filename: string): Promise<void> {
  if (!(await fs.promises.stat(filename)).isFile()) {
    throw new Error(`Not a file: ${filename}`);
  }
}

async function findPixi(platform: NodeJS.Platform, env: NodeJS.ProcessEnv): Promise<string> {
  const paths = (env.PATH || env.Path || "").split(platform === "win32" ? ";" : path.delimiter);
  const home = env.HOME || env.USERPROFILE;
  if (home) {
    paths.push(path.join(home, ".pixi", "bin"));
  }
  for (const directory of paths) {
    // Never resolve executables relative to the caller's workspace.
    if (!path.isAbsolute(directory)) {
      continue;
    }
    const filename = path.join(directory, platform === "win32" ? "pixi.exe" : "pixi");
    try {
      await requireFile(filename);
      await fs.promises.access(filename, platform === "win32" ? fs.constants.F_OK : fs.constants.X_OK);
      return filename;
    } catch {
      // Try the next explicit PATH entry.
    }
  }
  throw new Error("Pixi executable not found; install Pixi separately before checking this workspace.");
}

async function findWindowsPython(explicit: string | undefined, inherited: NodeJS.ProcessEnv): Promise<string> {
  if (explicit) {
    if (!path.isAbsolute(explicit) || !/\.exe$/i.test(explicit)) {
      throw new Error("The Windows Python interpreter must be an absolute .exe path.");
    }
    await requireFile(explicit);
    return explicit;
  }
  for (const directory of (inherited.PATH || inherited.Path || "").split(";")) {
    if (!path.isAbsolute(directory) || /[\\/]WindowsApps(?:[\\/]|$)/i.test(directory)) {
      continue;
    }
    const python = path.join(directory, "python.exe");
    try {
      await requireFile(python);
      return python;
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code !== "ENOENT" && (error as NodeJS.ErrnoException).code !== "ENOTDIR") {
        throw error;
      }
    }
  }
  throw new Error("Native Windows ROS needs a compatible Python installation. Add its directory to PATH or pass pythonExecutable with the health target.");
}

/** Exported for deterministic command-construction tests; does not activate or change an installation. */
export async function buildHealthCommand(
  target: HealthTarget,
  probePath: string,
  platform: NodeJS.Platform = process.platform,
  inherited: NodeJS.ProcessEnv = process.env
): Promise<HealthCommand> {
  if (!/^[a-z][a-z0-9_-]*$/.test(target.distro)) {
    throw new Error("Invalid ROS distribution name.");
  }
  const targetPath = target.kind === "setup" ? target.setupScript : target.workspace;
  if (!path.isAbsolute(targetPath) || !path.isAbsolute(probePath)) {
    throw new Error("The installation target and probe must use absolute paths.");
  }
  await requireFile(probePath);
  const env = cleanEnvironment(platform, inherited);
  if (target.kind === "pixi") {
    const manifest = path.join(target.workspace, "pixi.toml");
    await requireFile(manifest);
    await requireFile(path.join(target.workspace, "pixi.lock"));
    const prefix = path.join(target.workspace, ".pixi", "envs", target.distro);
    // No fallback to a system Python, even if a damaged Pixi environment leaves it on PATH.
    await requireFile(path.join(prefix, platform === "win32" ? "python.exe" : "bin/python"));
    return {
      command: await findPixi(platform, inherited),
      args: [
        "run", "--frozen", "--no-install", "--executable", "--manifest-path", manifest,
        "-e", target.distro, "python", "-s", probePath,
        "--distro", target.distro, "--prefix", prefix, "--kind", "pixi",
      ],
      env, cwd: target.workspace,
    };
  }
  await requireFile(target.setupScript);
  const prefix = path.dirname(target.setupScript);
  const probeArgs = ["--distro", target.distro, "--prefix", prefix, "--kind", "setup"];
  if (platform === "win32") {
    if (!/\.bat$/i.test(target.setupScript)) {
      throw new Error("Windows health checking requires a .bat setup script.");
    }
    const python = await findWindowsPython(target.pythonExecutable, inherited);
    const args = [target.setupScript, python, probePath, ...probeArgs];
    // CALL expands twice, including inside quotes. Reject all cmd expansion/control characters.
    if (args.some((arg) => /[%!^&|<>()"\r\n\0]/.test(arg))) {
      throw new Error("Unsafe cmd.exe expansion characters in the setup/probe path.");
    }
    const quoted = (arg: string) => `"${arg}"`;
    env.COLCON_PYTHON_EXECUTABLE = python;
    return {
      command: path.win32.join(inherited.SystemRoot || inherited.SYSTEMROOT || "C:\\Windows", "System32", "cmd.exe"),
      args: ["/d", "/v:off", "/s", "/c",
        `"call ${quoted(target.setupScript)} && ${quoted(python)} -s ${[probePath, ...probeArgs].map(quoted).join(" ")}"`],
      env, cwd: prefix, windowsVerbatimArguments: true,
    };
  }
  return {
    command: "/bin/bash",
    args: [
      "--noprofile", "--norc", "-c",
      'source "$1" && shift && exec python3 -s "$@"',
      "rde-ros-health", target.setupScript, probePath, ...probeArgs,
    ],
    env, cwd: prefix,
  };
}

/** Bound both process lifetime and output, including children started by setup scripts. */
export function runHealthProcess(command: HealthCommand, limitMs = timeoutMs): Promise<HealthProcessResult> {
  return new Promise((resolve) => {
    let stdout = "";
    let stderr = "";
    let size = 0;
    let failure: string | undefined;
    let settled = false;
    let killTimer: NodeJS.Timeout | undefined;
    const child = spawn(command.command, command.args, {
      cwd: command.cwd, env: command.env, windowsHide: true,
      windowsVerbatimArguments: command.windowsVerbatimArguments,
      detached: process.platform !== "win32", stdio: ["ignore", "pipe", "pipe"],
    });
    const finish = (exitCode: number | null, signal?: string) => {
      if (settled) {
        return;
      }
      settled = true;
      clearTimeout(timer);
      if (killTimer) {
        clearTimeout(killTimer);
      }
      resolve({ stdout, stderr, exitCode, signal, error: failure });
    };
    const terminate = (reason: string) => {
      if (failure) {
        return;
      }
      failure = reason;
      if (child.pid !== undefined) {
        if (process.platform === "win32") {
          execFile("taskkill.exe", ["/pid", String(child.pid), "/t", "/f"],
            { timeout: 3000, maxBuffer: 4096, windowsHide: true }, (error) => {
              if (error && child.exitCode === null) {
                failure += ` Process-tree termination failed: ${error.message}`;
                child.kill("SIGKILL");
              }
            });
        } else {
          try {
            process.kill(-child.pid, "SIGKILL");
          } catch (error) {
            if ((error as NodeJS.ErrnoException).code !== "ESRCH") {
              failure += ` Process-tree termination failed: ${error}`;
              child.kill("SIGKILL");
            }
          }
        }
      }
      // A descendant holding a pipe must not leave validation awaiting close indefinitely.
      killTimer = setTimeout(() => {
        child.stdout.destroy();
        child.stderr.destroy();
        finish(child.exitCode, "SIGKILL");
      }, 4000);
    };
    const timer = setTimeout(() => terminate(`Health probe timed out after ${limitMs}ms.`), limitMs);
    const collect = (data: Buffer, isError: boolean) => {
      const remaining = Math.max(0, outputLimit - size);
      const text = data.subarray(0, remaining).toString("utf8");
      if (isError) {
        stderr += text;
      } else {
        stdout += text;
      }
      size += data.length;
      if (size > outputLimit) {
        terminate(`Health probe output exceeded ${outputLimit} bytes.`);
      }
    };
    child.stdout.on("data", (data: Buffer) => collect(data, false));
    child.stderr.on("data", (data: Buffer) => collect(data, true));
    child.once("error", (error) => {
      failure = error.message;
      finish(null);
    });
    child.once("close", (code, signal) => finish(code, signal || undefined));
  });
}

export function parseHealthOutput(stdout: string, target: HealthTarget): HealthCheck[] {
  const lines = stdout.split(/\r?\n/).filter((line) => line.startsWith(HEALTH_SENTINEL));
  if (lines.length !== 1) {
    throw new Error("Expected exactly one structured health-probe result.");
  }
  const data = JSON.parse(lines[0].slice(HEALTH_SENTINEL.length));
  if (data.version !== 1 || data.distro !== target.distro || !Array.isArray(data.checks)
    || data.checks.length !== requiredChecks.length) {
    throw new Error("Malformed health result, unsupported version, or distribution mismatch.");
  }
  const seen = new Set<string>();
  for (const check of data.checks) {
    if (!check || !requiredChecks.includes(check.id) || seen.has(check.id)
      || !["passed", "failed"].includes(check.status) || typeof check.detail !== "string"
      || check.detail.length > 8192 || typeof check.durationMs !== "number"
      || !Number.isFinite(check.durationMs) || check.durationMs < 0) {
      throw new Error("Malformed or incomplete health checks.");
    }
    seen.add(check.id);
  }
  return data.checks;
}

export async function validateInstallation(target: HealthTarget, probePath: string): Promise<HealthReport> {
  const started = Date.now();
  const report: HealthReport = { target, healthy: false, checks: [] };
  const fail = (id: string, detail: string) => report.checks.push({
    id, status: "failed", detail, durationMs: Date.now() - started,
  });
  try {
    const command = await buildHealthCommand(target, probePath);
    const result = await runHealthProcess(command);
    try {
      report.checks.push(...parseHealthOutput(result.stdout, target));
    } catch (error) {
      fail("probe-output", String(error));
    }
    if (result.error || result.exitCode !== 0) {
      fail("probe-process", [
        result.error || "",
        `Probe exited with ${result.exitCode === null ? `signal ${result.signal || "unknown"}` : `code ${result.exitCode}`}.`,
        result.stderr.slice(-8192), result.stdout.slice(-2048),
      ].filter(Boolean).join("\n"));
    }
  } catch (error) {
    fail("activation", String(error));
  }
  report.healthy = report.checks.length >= requiredChecks.length
    && report.checks.every((check) => check.status === "passed");
  return report;
}
