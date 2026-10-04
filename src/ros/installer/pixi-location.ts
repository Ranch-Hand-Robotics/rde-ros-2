// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as os from "os";
import * as path from "path";
import * as vscode from "vscode";

export const PIXI_INSTALL_LOCATIONS_SETTING = "pixiInstallLocationsByMachine";

export interface PixiLocationOptions {
  machineId: string;
  locations: Record<string, string>;
  machineOverride?: string;
  legacyRoot?: string;
  platform: NodeJS.Platform;
  home: string;
}

function defaultPixiRoot(platform: NodeJS.Platform, home: string): string {
  if (platform === "win32") {
    return "c:\\pixi_ws";
  }
  if (platform === "darwin") {
    return path.join(home, "pixi_ws");
  }
  return "";
}

function absoluteLocation(location: string | undefined): string | undefined {
  const trimmed = location?.trim();
  return trimmed && path.isAbsolute(trimmed) ? trimmed : undefined;
}

/** Resolve the per-machine Pixi root, migrating the legacy setting only before a machine map exists. */
export function resolvePixiInstallRoot(options: PixiLocationOptions): string {
  const machineOverride = absoluteLocation(options.machineOverride);
  if (machineOverride) {
    return machineOverride;
  }

  const machineLocation = absoluteLocation(options.locations[options.machineId]);
  if (machineLocation) {
    return machineLocation;
  }

  const hasMachineEntries = Object.keys(options.locations).length > 0;
  if (!hasMachineEntries) {
    const legacyLocation = absoluteLocation(options.legacyRoot);
    if (legacyLocation) {
      return legacyLocation;
    }
  }

  return defaultPixiRoot(options.platform, options.home);
}

/** Returns the root associated with this VS Code machine, not a cached path from another computer. */
export function getPixiInstallRoot(): string {
  const config = vscode.workspace.getConfiguration("ROS2");
  const machineLocations = config.inspect<Record<string, string>>(PIXI_INSTALL_LOCATIONS_SETTING)?.globalValue ?? {};
  const legacy = config.inspect<string>("pixiRoot");
  return resolvePixiInstallRoot({
    machineId: vscode.env.machineId,
    locations: machineLocations,
    machineOverride: legacy?.globalValue,
    legacyRoot: legacy?.workspaceFolderValue ?? legacy?.workspaceValue ?? legacy?.globalValue,
    platform: process.platform,
    home: os.homedir(),
  });
}

/** Ask the user to choose the root; each ROS distro is installed under a child folder. */
export async function selectPixiInstallRoot(distro?: string): Promise<string | undefined> {
  const currentRoot = getPixiInstallRoot();
  const selected = await vscode.window.showOpenDialog({
    canSelectFiles: false,
    canSelectFolders: true,
    canSelectMany: false,
    defaultUri: path.isAbsolute(currentRoot) ? vscode.Uri.file(currentRoot) : undefined,
    openLabel: "Choose Install Location",
    title: distro ? `Choose Pixi install location for ROS 2 ${distro}` : "Choose Pixi install location",
  });
  const root = selected?.[0]?.fsPath.trim();
  if (!root) {
    return undefined;
  }
  if (!path.isAbsolute(root)) {
    throw new Error(`Pixi install location must be an absolute path: ${root}`);
  }
  return path.normalize(root);
}

/** Persist this machine's chosen root in global settings so Settings Sync can carry a per-machine map. */
export async function cachePixiInstallRoot(
  root: string,
  config = vscode.workspace.getConfiguration("ROS2"),
  machineId = vscode.env.machineId
): Promise<void> {
  if (!machineId) {
    throw new Error("VS Code did not provide a machine ID; the Pixi install location could not be cached.");
  }
  const absoluteRoot = absoluteLocation(root);
  if (!absoluteRoot) {
    throw new Error(`Pixi install root must be an absolute path: ${root}`);
  }

  const locations = config.inspect<Record<string, string>>(PIXI_INSTALL_LOCATIONS_SETTING)?.globalValue ?? {};
  if (locations[machineId] === absoluteRoot) {
    return;
  }
  await config.update(PIXI_INSTALL_LOCATIONS_SETTING, { ...locations, [machineId]: absoluteRoot }, vscode.ConfigurationTarget.Global);
}
