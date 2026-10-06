import * as childProcess from "child_process";
import { promises as fs } from "fs";
import * as os from "os";
import * as path from "path";
import { promisify } from "util";

const execFile = promisify(childProcess.execFile);

export interface WindowsBatchOptions {
  cwd?: string;
  onOutput?: (message: string) => void;
  failOnMissingSetup?: boolean;
}

export function quoteBatchPath(value: string): string {
  if (/[\r\n\0"]/.test(value)) {
    throw new Error("Invalid path in Windows ROS environment setup");
  }
  return `"${value.replace(/%/g, "%%")}"`;
}

/** Capture a child CMD environment without mutating the extension host. */
export async function sourceWindowsBatch(
  commands: string[], env: NodeJS.ProcessEnv, options: WindowsBatchOptions = {},
): Promise<NodeJS.ProcessEnv> {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), "rde-ros-env-"));
  try {
    const marker = "__RDE_ROS_ENVIRONMENT__";
    const script = path.join(directory, "setup.bat");
    await fs.writeFile(script, ["@echo off", "setlocal DisableDelayedExpansion", ...commands,
      `echo ${marker}`, "set", ""].join("\r\n"));
    const logDiagnostics = (stdout: string, stderr: string) => {
      // Everything after the marker is the environment dump, not diagnostic output.
      const diagnostic = stdout.split(marker)[0].trim();
      if (diagnostic) { options.onOutput?.(diagnostic); }
      if (stderr.trim()) { options.onOutput?.(stderr.trim()); }
    };
    const { stdout, stderr } = await execFile(process.env.ComSpec || "cmd.exe", [
      "/d", "/s", "/c", `"${script}"`,
    ], { env, cwd: options.cwd, timeout: 60000, maxBuffer: 1024 * 1024,
      windowsHide: true, windowsVerbatimArguments: true }).catch(error => {
        logDiagnostics(String(error.stdout ?? ""), String(error.stderr ?? ""));
        throw error;
      });
    logDiagnostics(stdout, stderr);
    if (options.failOnMissingSetup && /^\s*not found:\s*.+$/im.test(stdout.split(marker)[0] + "\n" + stderr)) {
      throw new Error("ROS setup reported a missing setup hook. See Output > ROS 2 for the missing path; the partial environment was discarded.");
    }
    const lines = stdout.split(/\r?\n/);
    const start = lines.indexOf(marker);
    if (start < 0) { throw new Error("Windows ROS setup did not return an environment"); }
    const result: NodeJS.ProcessEnv = {};
    for (const line of lines.slice(start + 1)) {
      const separator = line.indexOf("=");
      if (separator > 0) { result[line.slice(0, separator)] = line.slice(separator + 1); }
    }
    return result;
  } finally {
    await fs.rm(directory, { recursive: true, force: true });
  }
}