// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

// Worker thread for ROS installation subprocess operations.
// Runs off the extension host main thread to avoid blocking VS Code.
// Communicates via postMessage with the main thread.

import { parentPort } from "worker_threads";
import { findPixi } from "./pixi";
import { runBootstrapProcess } from "./bootstrap-process";

// ---------------------------------------------------------------------------
// Shared message types (imported by install-ros.ts as well)
// ---------------------------------------------------------------------------

/** Messages sent FROM the main thread TO this worker. */
export type WorkerRequest =
  | { type: "check_pixi" }
  | { type: "install_pixi"; platform: string };

/** Messages sent FROM this worker TO the main thread. */
export type WorkerResponse =
  | { type: "pixi_available"; available: boolean; executable?: string }
  | { type: "log"; text: string }
  | { type: "complete" }
  | { type: "error"; message: string };

// ---------------------------------------------------------------------------
// Guard: this file must only run as a worker thread
// ---------------------------------------------------------------------------

if (!parentPort) {
  throw new Error("install-ros-worker.ts must be run as a worker thread");
}

// ---------------------------------------------------------------------------
// Message dispatch
// ---------------------------------------------------------------------------

parentPort.on("message", (msg: WorkerRequest) => {
  switch (msg.type) {
    case "check_pixi":
      handleCheckPixi();
      break;
    case "install_pixi":
      handleInstallPixi(msg.platform);
      break;
    default:
      send({ type: "error", message: `Unknown worker request type: ${(msg as WorkerRequest & { type: string }).type}` });
  }
});

function send(response: WorkerResponse): void {
  parentPort!.postMessage(response);
}

// ---------------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------------

async function handleCheckPixi(): Promise<void> {
  const executable = await findPixi();
  send({ type: "pixi_available", available: !!executable, executable });
}

function handleInstallPixi(platform: string): void {
  let command: string;
  let args: string[];
  if (platform === "win32") {
    command = "winget.exe";
    args = ["install", "--id", "prefix-dev.pixi", "--exact", "--accept-source-agreements", "--accept-package-agreements", "--disable-interactivity"];
  } else if (platform === "darwin") {
    command = "/bin/bash";
    args = ["-o", "pipefail", "-c", "curl --connect-timeout 30 --max-time 300 -fsSL https://pixi.sh/install.sh | sh"];
  } else {
    send({ type: "error", message: "Pixi installation is only supported on Windows and macOS" });
    return;
  }

  runBootstrapProcess(command, args, (text) => send({ type: "log", text })).then(
    () => {
      send({ type: "complete" });
    },
    (error: Error) => {
      send({ type: "error", message: error.message });
    }
  );
}
