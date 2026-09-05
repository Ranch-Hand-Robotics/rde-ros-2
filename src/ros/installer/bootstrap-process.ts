// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import { execFile, spawn } from "child_process";

/** Runs Pixi bootstrap without an output buffer limit, killing its process tree on timeout. */
export function runBootstrapProcess(
  command: string,
  args: string[],
  onLog: (text: string) => void,
  timeoutMs = 600000
): Promise<void> {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, {
      stdio: ["ignore", "pipe", "pipe"],
      windowsHide: true,
      // A separate Unix process group lets the timeout kill the shell AND its installer children.
      detached: process.platform !== "win32",
    });
    let timedOut = false;
    let cleanup: Promise<void> = Promise.resolve();
    const timer = setTimeout(() => {
      timedOut = true;
      if (child.pid === undefined) {
        return;
      }
      if (process.platform === "win32") {
        cleanup = new Promise((done, failed) => {
          execFile("taskkill.exe", ["/pid", String(child.pid), "/t", "/f"], { timeout: 15000 }, (error) => {
            if (error && child.exitCode === null) {
              failed(new Error(`Bootstrap timed out and process-tree termination failed: ${error.message}`));
            } else {
              done();
            }
          });
        });
      } else {
        try {
          process.kill(-child.pid, "SIGKILL");
        } catch (error) {
          if ((error as NodeJS.ErrnoException).code !== "ESRCH") {
            cleanup = Promise.reject(error);
          }
        }
      }
      // Attach the rejection handler immediately; close may arrive in a later event-loop turn.
      cleanup.catch(reject);
    }, timeoutMs);
    child.stdout.on("data", (data: Buffer) => onLog(data.toString()));
    child.stderr.on("data", (data: Buffer) => onLog(data.toString()));
    child.once("error", (error) => {
      clearTimeout(timer);
      reject(error);
    });
    child.once("close", (code, signal) => {
      clearTimeout(timer);
      cleanup.then(() => {
        if (timedOut) {
          reject(new Error(`Pixi bootstrap exceeded ${timeoutMs}ms; its process tree was terminated. Partial installation changes may remain.`));
        } else if (code !== 0) {
          reject(new Error(`Pixi bootstrap exited with ${code === null ? `signal ${signal}` : `code ${code}`}.`));
        } else {
          resolve();
        }
      }, reject);
    });
  });
}
