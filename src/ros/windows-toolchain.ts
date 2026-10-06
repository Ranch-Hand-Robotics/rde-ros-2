import * as childProcess from "child_process";
import { promises as fs } from "fs";
import * as path from "path";
import { promisify } from "util";
import { quoteBatchPath, sourceWindowsBatch, WindowsBatchOptions } from "./windows-batch";

const execFile = promisify(childProcess.execFile);

// Include ATL alongside the C++ toolchain and SDK; Pixi supplies CMake/Ninja.
export const WINDOWS_BUILD_TOOLS_COMMAND = 'winget install --id Microsoft.VisualStudio.2022.BuildTools --exact --source winget --override "--wait --passive --norestart --add Microsoft.VisualStudio.Workload.VCTools --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.VC.ATL --add Microsoft.VisualStudio.Component.Windows11SDK.26100"';

export interface WindowsToolchainOptions extends WindowsBatchOptions {
  visualStudioSetup?: string[];
}

function value(env: NodeJS.ProcessEnv, name: string): string {
  const key = Object.keys(env).find(key => key.toLowerCase() === name.toLowerCase());
  return key ? env[key] ?? "" : "";
}

async function isFile(filename: string): Promise<boolean> {
  return fs.stat(filename).then(stat => stat.isFile(), () => false);
}

/** Verify tools and SDK files, not just VisualStudioVersion left by a partial setup. */
export async function hasWindowsToolchain(env: NodeJS.ProcessEnv): Promise<boolean> {
  if (!value(env, "VisualStudioVersion") || value(env, "VSCMD_ARG_TGT_ARCH").toLowerCase() !== "x64"
    || !value(env, "INCLUDE") || !value(env, "LIB")) { return false; }
  const sdk = value(env, "WindowsSdkDir");
  const version = value(env, "WindowsSDKVersion").replace(/[\\/]+$/, "");
  if (!sdk || !version) { return false; }
  for (const file of [path.join(sdk, "Include", version, "um", "Windows.h"),
    path.join(sdk, "Include", version, "ucrt", "stdio.h"),
    path.join(sdk, "Lib", version, "um", "x64", "kernel32.lib"),
    path.join(sdk, "Lib", version, "ucrt", "x64", "ucrt.lib")]) {
    if (!await isFile(file)) { return false; }
  }
  const directories = value(env, "PATH").split(";").filter(directory => path.isAbsolute(directory));
  for (const tool of ["cl.exe", "link.exe", "rc.exe"]) {
    if (!(await Promise.all(directories.map(directory => isFile(path.join(directory, tool))))).some(Boolean)) {
      return false;
    }
  }
  return true;
}

/** vswhere includes Build Tools (-products *) and excludes IDEs without MSVC. */
export async function findWindowsToolchainSetups(env: NodeJS.ProcessEnv = process.env): Promise<string[]> {
  const roots = [value(env, "ProgramFiles(x86)"), value(env, "ProgramFiles"),
    process.env["ProgramFiles(x86)"], process.env.ProgramFiles].filter((root): root is string => !!root);
  const candidates = roots.map(root => path.join(root, "Microsoft Visual Studio", "Installer", "vswhere.exe"));
  for (const directory of value(env, "PATH").split(";").filter(directory => path.isAbsolute(directory))) {
    candidates.push(path.join(directory, "vswhere.exe"));
  }
  for (const executable of [...new Set(candidates)]) {
    if (!await isFile(executable)) { continue; }
    const { stdout } = await execFile(executable, ["-products", "*", "-version", "[17.0,18.0)",
      "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-sort",
      "-property", "installationPath", "-utf8"], { env, timeout: 15000, windowsHide: true });
    return stdout.split(/\r?\n/).map(line => line.trim()).filter(line => path.isAbsolute(line))
      .map(root => path.join(root, "VC", "Auxiliary", "Build", "vcvarsall.bat"));
  }
  return [];
}

export async function activateWindowsToolchain(
  env: NodeJS.ProcessEnv = process.env, options: WindowsToolchainOptions = {},
): Promise<NodeJS.ProcessEnv> {
  // Reuse a complete inherited environment for workspace overlays.
  if (await hasWindowsToolchain(env)) { return env; }
  const setups = options.visualStudioSetup ?? await findWindowsToolchainSetups(env);
  for (const setup of setups) {
    if (!await isFile(setup)) { continue; }
    try {
      const result = await sourceWindowsBatch([
        `call ${quoteBatchPath(setup)} x64`, "if errorlevel 1 exit /b %errorlevel%",
      ], env, options);
      if (await hasWindowsToolchain(result)) {
        options.onOutput?.(`Activated Visual Studio C++ tools: ${setup}`);
        return result;
      }
      options.onOutput?.(`Visual Studio setup did not provide a complete x64 compiler and Windows SDK: ${setup}`);
    } catch (error) {
      options.onOutput?.(`Visual Studio C++ environment activation failed: ${setup}: ${error instanceof Error ? error.message : String(error)}`);
    }
  }
  throw new Error("Visual Studio 2022 C++ build tools and a Windows SDK are required before starting Pixi/ROS. " +
    "Install Microsoft.VisualStudio.2022.BuildTools with MSVC x64/x86 and Windows SDK, or use Visual Studio Installer > Modify to add them to an existing installation. " +
    "Then retry ROS environment activation. Do not set VisualStudioVersion manually. Install from Administrator PowerShell: " + WINDOWS_BUILD_TOOLS_COMMAND);
}