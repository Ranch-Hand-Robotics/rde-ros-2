import { constants, promises as fs } from "fs";
import * as path from "path";

/** Resolve only against the sourced ROS environment, never the editor's PATH.
 * Windows Pixi ships python.exe, not necessarily python3.exe. A bare python3
 * can therefore pick a Store alias with a different ABI from rclpy.
 */
export async function resolveRosPython(
  env: NodeJS.ProcessEnv,
  platform: NodeJS.Platform = process.platform,
): Promise<string> {
  const windows = platform === "win32";
  const paths = windows ? path.win32 : path.posix;
  const value = (name: string): string | undefined => windows
    ? Object.entries(env).find(([key]) => key.toUpperCase() === name)?.[1]
    : env[name];
  const unquote = (entry: string) => entry.replace(/^"|"$/g, "");
  const usable = async (file: string): Promise<boolean> => {
    if (!paths.isAbsolute(file) || (windows && /[\\/]WindowsApps(?:[\\/]|$)/i.test(file))) {
      return false;
    }
    try {
      if (!(await fs.stat(file)).isFile()) { return false; }
      await fs.access(file, windows ? constants.F_OK : constants.X_OK);
      return true;
    } catch { return false; }
  };

  // Pixi/conda activation identifies its interpreter even if PATH was overridden.
  const conda = value("CONDA_PREFIX");
  const virtual = value("VIRTUAL_ENV");
  if (conda || virtual) {
    const prefix = unquote(conda || virtual!);
    const python = paths.join(prefix, windows ? (conda ? "python.exe" : "Scripts/python.exe") : "bin/python");
    if (await usable(python)) { return python; }
    throw new Error(`ROS environment Python is missing or not executable: ${python}. Repair or reselect the ROS/Pixi environment; refusing to fall back to an unrelated Python.`);
  }

  for (const entry of (value("PATH") || "").split(windows ? ";" : ":")) {
    const directory = unquote(entry);
    if (!paths.isAbsolute(directory)) { continue; }
    for (const name of windows ? ["python.exe", "python3.exe"] : ["python3", "python"]) {
      const python = paths.join(directory, name);
      if (await usable(python)) { return python; }
    }
  }
  throw new Error("No Python interpreter found in the sourced ROS environment. Select a working ROS/Pixi installation; Windows Store aliases are not used for ROS launch files.");
}