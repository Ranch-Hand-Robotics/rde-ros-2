import * as childProcess from "child_process";
import { promises as fs } from "fs";
import * as os from "os";
import * as path from "path";
import { promisify } from "util";
import { findPixi } from "./installer/pixi";
import { quoteBatchPath, sourceWindowsBatch } from "./windows-batch";
import { activateWindowsToolchain } from "./windows-toolchain";

const execFile = promisify(childProcess.execFile);

export interface WindowsEnvironmentOptions {
  cwd?: string;
  visualStudioSetup?: string[];
  onOutput?: (message: string) => void;
  failOnMissingSetup?: boolean;
}

/** Locate the selected installation, not the parent containing all ROS distros. */
export async function findWindowsPixiEnvironment(filename: string): Promise<{
  manifest: string;
  environment: string;
} | undefined> {
  let directory = path.dirname(path.resolve(filename));
  while (true) {
    const relative = path.relative(directory, filename).split(path.sep);
    const inEnvironment = relative[0]?.toLowerCase() === ".pixi"
      && relative[1]?.toLowerCase() === "envs" && relative.length > 3;
    // Also support the legacy layout: <pixi project>/ros2-windows/setup.bat.
    for (const name of inEnvironment ? ["pixi.toml", "pyproject.toml"] : ["pixi.toml"]) {
      const manifest = path.join(directory, name);
      if (await fs.stat(manifest).then(stat => stat.isFile(), () => false)) {
        return { manifest, environment: inEnvironment ? relative[2] : "default" };
      }
    }
    const parent = path.dirname(directory);
    if (parent === directory) {
      return undefined;
    }
    directory = parent;
  }
}

/** Activate compiler tools before invoking Pixi, then source its hook and ROS. */
export async function sourceWindowsEnvironment(
  filename: string,
  env?: NodeJS.ProcessEnv,
  options: WindowsEnvironmentOptions = {},
): Promise<NodeJS.ProcessEnv> {
  const baseEnv = await activateWindowsToolchain(env ?? process.env, options);
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), "rde-ros-env-"));
  const log = options.onOutput ?? (() => {});
  try {
    const commands: string[] = [];
    const checkedCall = (file: string, args = "") => {
      commands.push(`call ${quoteBatchPath(file)}${args}`, "if errorlevel 1 exit /b %errorlevel%");
    };
    const pixi = await findWindowsPixiEnvironment(filename);
    const selectedEnvironment = pixi && path.relative(path.dirname(pixi.manifest), filename)
      .toLowerCase().startsWith(`.pixi${path.sep}envs${path.sep}`);
    // A supplied ROS environment is an underlay. Do not replace its named Pixi
    // environment with "default" when sourcing a workspace inside that project.
    if (pixi && (selectedEnvironment || env?.ROS_VERSION !== "2")) {
      const executable = await findPixi("win32", os.homedir(), baseEnv);
      if (!executable) {
        throw new Error(`Pixi executable not found for ${pixi.manifest}`);
      }
      log(`Activating Pixi environment '${pixi.environment}' from ${pixi.manifest}`);
      const { stdout, stderr } = await execFile(executable, [
        "shell-hook", "--shell", "cmd", "--manifest-path", pixi.manifest,
        "--environment", pixi.environment, "--frozen", "--no-install",
      ], { env: baseEnv, cwd: path.dirname(pixi.manifest), timeout: 60000, maxBuffer: 1024 * 1024, windowsHide: true });
      if (stderr.trim()) {
        log(stderr.trim());
      }
      if (!stdout.trim()) {
        throw new Error(`Pixi returned an empty activation hook for ${pixi.manifest}`);
      }
      const hook = path.join(directory, "pixi-hook.bat");
      await fs.writeFile(hook, stdout);
      checkedCall(hook);
    }

    checkedCall(filename);
    return await sourceWindowsBatch(commands, baseEnv, options);
  } finally {
    await fs.rm(directory, { recursive: true, force: true });
  }
}