import * as childProcess from "child_process";
import * as os from "os";
import * as path from "path";
import { promisify } from "util";

const execFile = promisify(childProcess.execFile);

export async function macPrerequisiteIssue(): Promise<string | undefined> {
  if (process.arch === "x64") {
    const translated = await execFile("/usr/sbin/sysctl", ["-n", "sysctl.proc_translated"]).catch(() => undefined);
    if (translated?.stdout.trim() === "1") {
      throw new Error("Use the Apple Silicon build of VS Code/Cursor instead of running under Rosetta before installing ROS 2.");
    }
  }
  try {
    await execFile("/usr/bin/xcrun", ["--find", "clang"], { timeout: 10000 });
    await execFile("/usr/bin/xcrun", ["--show-sdk-path"], { timeout: 10000 });
    return undefined;
  } catch {
    return "Apple Command Line Tools and a working macOS SDK are required for the ROS development environment. Complete Apple's installer, then run Install ROS 2 again. If Xcode is already installed, check xcode-select and accept its license in Xcode.";
  }
}

export async function requestMacCommandLineTools(): Promise<void> {
  await execFile("/usr/bin/xcode-select", ["--install"], { timeout: 10000 });
}

export function pixiExecutableCandidates(
  platform = process.platform,
  home = os.homedir(),
  env = process.env,
  systemCandidates = platform === "darwin" ? ["/opt/homebrew/bin/pixi", "/usr/local/bin/pixi"] : []
): string[] {
  const executable = platform === "win32" ? "pixi.exe" : "pixi";
  return [
    ...(env.PIXI_BIN_DIR ? [path.join(env.PIXI_BIN_DIR, executable)] : []),
    path.join(env.PIXI_HOME || path.join(home, ".pixi"), "bin", executable),
    ...(platform === "win32" && env.LOCALAPPDATA
      ? [
        path.join(env.LOCALAPPDATA, "Microsoft", "WinGet", "Links", executable),
        path.join(env.LOCALAPPDATA, "pixi", "bin", executable),
      ] : []),
    ...(env.PATH || "").split(path.delimiter).filter(Boolean).map(directory => path.join(directory, executable)),
    ...systemCandidates,
  ];
}

export async function findPixi(
  platform = process.platform,
  home = os.homedir(),
  env = process.env,
  systemCandidates = platform === "darwin" ? ["/opt/homebrew/bin/pixi", "/usr/local/bin/pixi"] : []
): Promise<string | undefined> {
  const candidates = pixiExecutableCandidates(platform, home, env, systemCandidates);
  for (const candidate of candidates) {
    try {
      await execFile(candidate, ["--version"], { env, timeout: 10000 });
      return candidate;
    } catch {
      continue;
    }
  }
  return undefined;
}

export function quoteShell(value: string): string {
  return `'${value.replace(/'/g, `'\\''`)}'`;
}

export function pixiPlatform(platform = process.platform, arch = process.arch): string {
  if (platform === "darwin" && (arch === "arm64" || arch === "x64")) {
    return arch === "arm64" ? "osx-arm64" : "osx-64";
  }
  if (platform === "win32" && arch === "x64") {
    return "win-64";
  }
  throw new Error(`Unsupported Pixi host: ${platform}/${arch}. Use a native supported VS Code build.`);
}

export async function macOSVersion(): Promise<string> {
  const { stdout } = await execFile("/usr/bin/sw_vers", ["-productVersion"], { timeout: 10000 });
  const version = stdout.trim();
  if (!/^\d+\.\d+(?:\.\d+)?$/.test(version)) {
    throw new Error(`Could not determine the macOS version: ${version}`);
  }
  return version;
}

export function pixiManifest(distro: string, platform: string, macos?: string): string {
  if (!/^[a-z]+$/.test(distro)) {
    throw new Error("Invalid ROS distribution");
  }
  const isMac = platform === "osx-arm64" || platform === "osx-64";
  if (isMac && (!macos || !/^\d+\.\d+(?:\.\d+)?$/.test(macos))) {
    throw new Error("A valid host macOS version is required for the Pixi manifest");
  }
  const rosPackage = distro === "rolling" ? "ros2-desktop" : `ros-${distro}-desktop`;
  return [
    "[workspace]",
    'name = "ros2-workspace"',
    `channels = ["https://prefix.dev/robostack-${distro}", "https://prefix.dev/conda-forge"]`,
    `platforms = [${isMac ? `{ platform = ${JSON.stringify(platform)}, macos = ${JSON.stringify(macos)} }` : JSON.stringify(platform)}]`,
    "",
    "[dependencies]",
    `${rosPackage} = "*"`,
    'python = "*"',
    'compilers = "*"',
    'cmake = "*"',
    'pkg-config = "*"',
    'make = "*"',
    'ninja = "*"',
    'rosdep = "*"',
    'colcon-common-extensions = "*"',
    "",
    "[environments]",
    `${distro} = { features = [] }`,
    "",
  ].join("\n");
}

export function pixiSetupScript(executable: string, manifest: string, distro: string): string {
  return [
    "shopt -s extglob",
    "unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION",
    `if ! _ros2_pixi_hook="$(${quoteShell(executable)} shell-hook --shell bash --manifest-path ${quoteShell(manifest)} -e ${quoteShell(distro)})"; then`,
    "  return 1",
    "fi",
    'eval "$_ros2_pixi_hook" || { unset _ros2_pixi_hook; return 1; }',
    "unset _ros2_pixi_hook",
    "",
  ].join("\n");
}

export function macInstallScript(executable: string, workspace: string, distro: string, setup: string): string {
  const pixi = quoteShell(executable);
  const smokeTest = "import rclpy; rclpy.init(); node = rclpy.create_node('rde_install_smoke_test'); node.destroy_node(); rclpy.shutdown()";
  return [
    "#!/bin/bash",
    "set -eo pipefail",
    "if ! /usr/bin/xcrun --find clang >/dev/null 2>&1 || ! /usr/bin/xcrun --show-sdk-path >/dev/null 2>&1; then",
    "  printf '%s\\n' 'Apple Command Line Tools and a macOS SDK are required. Run xcode-select --install, finish the Apple installer, then retry Install ROS 2.' >&2",
    "  exit 1",
    "fi",
    `cd ${quoteShell(workspace)}`,
    "unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION",
    `${pixi} --version`,
    `${pixi} install --manifest-path ${quoteShell(path.join(workspace, "pixi.toml"))} -e ${quoteShell(distro)}`,
    `source ${quoteShell(setup)}`,
    `test "$ROS_DISTRO" = ${quoteShell(distro)}`,
    "ros2 --help",
    `python -c ${quoteShell(smokeTest)}`,
    `mv ${quoteShell(setup)} ${quoteShell(path.join(workspace, "setup.bash"))}`,
    `printf '%s\\n' ${quoteShell(`ROS 2 ${distro} setup and smoke test passed.`)}`,
    "",
  ].join("\n");
}

export async function sourceBashEnvironment(filename: string, env = process.env, cwd?: string): Promise<NodeJS.ProcessEnv> {
  const { stdout } = await execFile("/bin/bash", [
    "--noprofile", "--norc", "-c", 'source "$1" >/dev/null && /usr/bin/env -0', "ros2-setup", filename,
  ], { env, cwd, timeout: 60000, maxBuffer: 1024 * 1024 });
  return Object.fromEntries(stdout.split("\0").filter(entry => entry.includes("=")).map(entry => {
    const separator = entry.indexOf("=");
    return [entry.slice(0, separator), entry.slice(separator + 1)];
  }));
}