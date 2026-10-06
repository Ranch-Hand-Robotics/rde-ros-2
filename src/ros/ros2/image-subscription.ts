// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as childProcess from "child_process";
import * as path from "path";
import * as extension from "../../extension";
import { resolveRosPython } from "../python";
import { TopicMessage } from "./topic-types";

const maxFrameLength = 64 * 1024 * 1024;

/** Assemble complete NDJSON frames without rescanning a growing multi-MB buffer. */
export class ImageFrameDecoder {
  private parts: string[] = [];
  private length = 0;

  public push(chunk: string, onFrame: (data: unknown) => void): void {
    let start = 0;
    while (start < chunk.length) {
      const end = chunk.indexOf("\n", start);
      const part = chunk.slice(start, end < 0 ? chunk.length : end);
      this.length += part.length;
      if (this.length > maxFrameLength) {
        this.parts = [];
        this.length = 0;
        throw new Error("Image frame exceeds the 64 MiB transport limit");
      }
      this.parts.push(part);
      if (end < 0) { return; }
      const line = this.parts.join("");
      this.parts = [];
      this.length = 0;
      if (line.trim()) {
        const data = JSON.parse(line);
        if (!data || typeof data !== "object" || typeof data.data !== "string") {
          throw new Error("Invalid image subscriber frame");
        }
        onFrame(data);
      }
      start = end + 1;
    }
  }
}

interface ImageSession {
  rateHz: number;
  process?: childProcess.ChildProcess;
}

export class ImageSubscriptionManager {
  private sessions = new Map<string, ImageSession>();

  public start(topic: string, type: string, onMessage: (message: TopicMessage) => void, rateHz = 5): void {
    this.stop(topic);
    const session: ImageSession = { rateHz: this.clampRate(rateHz) };
    this.sessions.set(topic, session);
    void this.launch(topic, type, session, onMessage).catch(error => {
      if (this.sessions.get(topic) === session) {
        extension.outputChannel.appendLine(`Unable to subscribe to image ${topic}: ${error.message}`);
        this.stop(topic);
      }
    });
  }

  private async launch(topic: string, type: string, session: ImageSession, onMessage: (message: TopicMessage) => void): Promise<void> {
    const env = await extension.resolvedEnv();
    if (this.sessions.get(topic) !== session) { return; }
    const python = await resolveRosPython(env);
    if (this.sessions.get(topic) !== session) { return; }
    const child = childProcess.spawn(python, [
      "-u", path.join(extension.extPath, "assets", "scripts", "image_subscriber.py"),
      "--topic", topic, "--type", type, "--rate", String(session.rateHz),
    ], { env, stdio: ["pipe", "pipe", "pipe"], windowsHide: true });
    session.process = child;
    extension.outputChannel.appendLine(`Image subscription ${topic}: ${python} (${session.rateHz} Hz preview)`);
    const current = () => this.sessions.get(topic) === session;
    const fail = (message: string) => {
      if (current()) {
        extension.outputChannel.appendLine(`Image subscription ${topic}: ${message}`);
        this.stop(topic);
      }
    };
    const decoder = new ImageFrameDecoder();
    child.stdout?.setEncoding("utf8");
    child.stdout?.on("data", (chunk: string) => {
      if (!current()) { return; }
      try {
        decoder.push(chunk, data => { if (current()) { onMessage({ timestamp: Date.now(), data }); } });
      } catch {
        // Never dump camera payloads or a JSON parser's input into the log.
        fail("Invalid or oversized image frame received; subscription stopped.");
      }
    });
    child.stderr?.setEncoding("utf8");
    child.stderr?.on("data", (text: string) => {
      if (current()) { extension.outputChannel.appendLine(`Image subscriber ${topic}: ${text.trim()}`); }
    });
    child.stdin?.on("error", error => fail(`Control pipe failed: ${error.message}`));
    child.on("error", error => fail(`Process failed: ${error.message}`));
    child.on("close", (code, signal) => {
      if (current()) {
        this.sessions.delete(topic);
        extension.outputChannel.appendLine(`Image subscription ${topic} exited (${signal || code}).`);
      }
    });
  }

  public setRate(topic: string, rateHz: number): void {
    const session = this.sessions.get(topic);
    if (!session) { return; }
    session.rateHz = this.clampRate(rateHz);
    const input = session.process?.stdin;
    // Ignore refresh updates after the helper's control pipe has closed.
    if (input && !input.destroyed && input.writable) {
      input.write(JSON.stringify({ rateHz: session.rateHz }) + "\n");
    }
  }

  private clampRate(rate: number): number {
    return Number.isFinite(rate) ? Math.min(30, Math.max(1, rate)) : 5;
  }

  public stop(topic: string): void {
    const session = this.sessions.get(topic);
    this.sessions.delete(topic); // Invalidate pending activation and late callbacks first.
    session?.process?.stdin?.end();
    session?.process?.kill(); // Direct Python child: no ros2 launcher/process tree to orphan.
  }

  public has(topic: string): boolean { return this.sessions.has(topic); }

  public dispose(): void {
    for (const topic of this.sessions.keys()) { this.stop(topic); }
  }
}