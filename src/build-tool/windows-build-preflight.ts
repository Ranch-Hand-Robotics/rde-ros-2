import * as vscode from "vscode";
import { activateWindowsToolchain, WINDOWS_BUILD_TOOLS_COMMAND, WindowsToolchainOptions } from "../ros/windows-toolchain";

/** Recheck disk state on every build, even when ROS was activated earlier. */
export async function preflightWindowsBuild(
  env: NodeJS.ProcessEnv, options: WindowsToolchainOptions = {}, isCancelled: () => boolean = () => false,
): Promise<NodeJS.ProcessEnv | undefined> {
  try {
    return await activateWindowsToolchain(env, options);
  } catch (error) {
    if (isCancelled()) { return undefined; }
    const detail = error instanceof Error ? error.message : String(error);
    options.onOutput?.(`Colcon build preflight failed: ${detail}\n\n` +
      "For an existing installation, use Visual Studio Installer > Modify > Desktop development with C++ to add MSVC v143 and the SDK. " +
      "For a new installation, review the copied WinGet command and run it in Administrator PowerShell, accepting the applicable agreements. " +
      "Restart if requested, reload the ROS environment, and rerun the build. Nothing will be installed automatically.");
    const choice = await vscode.window.showWarningMessage(
      "Colcon build stopped: C++ tools need setup.",
      { modal: true, detail: "Visual Studio 2022 C++ tools and a Windows SDK are required.\n" +
        "Copy the command to install, or see Output > ROS 2 for repair steps." },
      "Copy Install Command", "Cancel Build",
    );
    if (choice === "Copy Install Command" && !isCancelled()) {
      await vscode.env.clipboard.writeText(WINDOWS_BUILD_TOOLS_COMMAND);
    }
    return undefined;
  }
}