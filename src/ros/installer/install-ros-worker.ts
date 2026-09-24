// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

// Worker thread for ROS installation subprocess operations.
// Runs off the extension host main thread to avoid blocking VS Code.
// Communicates via postMessage with the main thread.

import { parentPort } from "worker_threads";
import * as child_process from "child_process";
import { findPixi } from "./pixi";

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
  let installCommand: string;
  if (platform === "win32") {
    installCommand = "winget install prefix-dev.pixi";
  } else if (platform === "darwin") {
    installCommand = "set -o pipefail; curl --fail --show-error --silent --location --connect-timeout 30 --max-time 300 https://pixi.sh/install.sh | sh";
  } else {
    send({ type: "error", message: "Pixi installation is only supported on Windows and macOS" });
    return;
  }

  const proc = child_process.exec(installCommand, {
    shell: platform === "darwin" ? "/bin/bash" : undefined,
    timeout: 600000,
    maxBuffer: 10 * 1024 * 1024,
  }, (error) => {
    if (error) {
      send({ type: "error", message: error.message });
    } else {
      send({ type: "complete" });
    }
  });

  proc.stdout?.on("data", (data: Buffer) => {
    send({ type: "log", text: data.toString() });
  });

  proc.stderr?.on("data", (data: Buffer) => {
    send({ type: "log", text: data.toString() });
  });
}
