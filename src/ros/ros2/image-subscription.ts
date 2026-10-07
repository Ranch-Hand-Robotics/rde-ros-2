// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as childProcess from "child_process";
import * as path from "path";
import { TextDecoder } from "util";
import * as extension from "../../extension";
import { resolveRosPython } from "../python";
import { TopicMessage } from "./topic-types";

const maxMetadataLength = 64 * 1024;
const maxPayloadLength = 32 * 1024 * 1024;
const frameMagic = Buffer.from("RDEB", "ascii");

function isRecord(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function isUint32(value: unknown): boolean {
  return typeof value === "number" && Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
}

/** Validate transport metadata, not the sensor's opaque encoding or field layout. */
function validMetadata(value: unknown, type?: string): value is Record<string, unknown> {
  if (!isRecord(value) || Object.prototype.hasOwnProperty.call(value, "data")) { return false; }
  const header = value.header;
  if (!isRecord(header) || typeof header.frame_id !== "string" || !isRecord(header.stamp)) { return false; }
  const { sec, nanosec } = header.stamp;
  if (typeof sec !== "number" || !Number.isInteger(sec) || sec < -0x80000000 || sec > 0x7fffffff ||
      !isUint32(nanosec)) { return false; }
  if ("format" in value) {
    return (!type || type === "sensor_msgs/msg/CompressedImage") && typeof value.format === "string";
  }
  if (!isUint32(value.width) || !isUint32(value.height)) { return false; }
  if ("fields" in value) {
    return (!type || type === "sensor_msgs/msg/PointCloud2") && Array.isArray(value.fields) &&
      value.fields.every(field => isRecord(field) && typeof field.name === "string" &&
        isUint32(field.offset) && isUint32(field.count) && isUint32(field.datatype) && Number(field.datatype) <= 255) &&
      typeof value.is_bigendian === "boolean" && typeof value.is_dense === "boolean" &&
      isUint32(value.point_step) && isUint32(value.row_step);
  }
  return (!type || type === "sensor_msgs/msg/Image") && typeof value.encoding === "string" &&
    isUint32(value.step) && isUint32(value.is_bigendian) && Number(value.is_bigendian) <= 255;
}

/** RDEB + uint32 LE metadata/payload lengths, JSON metadata, then raw bytes.
 * Each incoming byte is copied once, with no rescanning or growing concatenation.
 */
export class ImageFrameDecoder {
  private readonly header = Buffer.alloc(12);
  private readonly utf8 = new TextDecoder("utf-8", { fatal: true });
  private stage: "header" | "metadata" | "payload" = "header";
  private target = this.header;
  private offset = 0;
  private payloadLength = 0;
  private metadata?: Record<string, unknown>;
  private failed = false;

  constructor(private readonly type?: string) {}

  public push(chunk: Buffer, onFrame: (data: unknown) => void): void {
    try {
      if (this.failed || !Buffer.isBuffer(chunk)) { throw new Error("Invalid binary stream"); }
      let start = 0;
      while (start < chunk.length) {
        const count = Math.min(chunk.length - start, this.target.length - this.offset);
        chunk.copy(this.target, this.offset, start, start + count);
        start += count;
        this.offset += count;
        if (this.offset < this.target.length) { return; }
        this.offset = 0;
        if (this.stage === "header") {
          const metadataLength = this.header.readUInt32LE(4);
          this.payloadLength = this.header.readUInt32LE(8);
          if (!this.header.subarray(0, 4).equals(frameMagic) || metadataLength === 0 ||
              metadataLength > maxMetadataLength || this.payloadLength > maxPayloadLength) {
            throw new Error("Invalid frame header");
          }
          this.target = Buffer.allocUnsafe(metadataLength);
          this.stage = "metadata";
          continue;
        } else if (this.stage === "metadata") {
          const metadata: unknown = JSON.parse(this.utf8.decode(this.target));
          if (!validMetadata(metadata, this.type)) { throw new Error("Invalid frame metadata"); }
          this.metadata = metadata;
          this.target = Buffer.allocUnsafe(this.payloadLength);
          this.stage = "payload";
          if (this.payloadLength > 0) { continue; }
        }
        this.emit(onFrame);
      }
    } catch {
      this.failed = true;
      this.target = this.header;
      this.metadata = undefined;
      this.offset = 0;
      // JSON.parse errors may contain sensor data; expose only a fixed diagnostic.
      throw new Error("Invalid or oversized binary subscriber frame (transport limit)");
    }
  }

  private emit(onFrame: (data: unknown) => void): void {
    const data = { ...this.metadata, data: this.target };
    this.metadata = undefined;
    this.target = this.header;
    this.stage = "header";
    onFrame(data);
  }
}

interface BinarySession {
  rateHz: number;
  process?: childProcess.ChildProcess;
}

interface StreamOptions {
  label: string;
  kind: string;
  defaultRate: number;
  minRate: number;
  maxRate: number;
}

/** Shared direct-Python lifecycle and bounded binary transport. */
class BinarySubscriptionManager {
  private sessions = new Map<string, BinarySession>();

  constructor(private readonly options: StreamOptions) {}

  public start(topic: string, type: string, onMessage: (message: TopicMessage) => void, rateHz = this.options.defaultRate): void {
    this.stop(topic);
    const session: BinarySession = { rateHz: this.clampRate(rateHz) };
    this.sessions.set(topic, session);
    void this.launch(topic, type, session, onMessage).catch(error => {
      if (this.sessions.get(topic) === session) {
        extension.outputChannel.appendLine(`Unable to subscribe to ${this.options.kind} ${topic}: ${error.message}`);
        this.stop(topic);
      }
    });
  }

  private async launch(topic: string, type: string, session: BinarySession, onMessage: (message: TopicMessage) => void): Promise<void> {
    const env = await extension.resolvedEnv();
    if (this.sessions.get(topic) !== session) { return; }
    const python = await resolveRosPython(env);
    if (this.sessions.get(topic) !== session) { return; }
    const child = childProcess.spawn(python, [
      "-u", path.join(extension.extPath, "assets", "scripts", "image_subscriber.py"),
      "--topic", topic, "--type", type, "--rate", String(session.rateHz),
    ], { env, stdio: ["pipe", "pipe", "pipe"], windowsHide: true });
    session.process = child;
    extension.outputChannel.appendLine(`${this.options.label} subscription ${topic}: ${python} (${session.rateHz} Hz preview)`);
    const current = () => this.sessions.get(topic) === session;
    const fail = (message: string) => {
      if (current()) {
        extension.outputChannel.appendLine(`${this.options.label} subscription ${topic}: ${message}`);
        this.stop(topic);
      }
    };
    const decoder = new ImageFrameDecoder(type);
    child.stdout?.on("data", (chunk: Buffer) => {
      if (!current()) { return; }
      try {
        decoder.push(chunk, data => { if (current()) { onMessage({ timestamp: Date.now(), data }); } });
      } catch {
        // Never dump binary payloads or a JSON parser's input into the log.
        fail(`Invalid or oversized ${this.options.kind} frame received; subscription stopped.`);
      }
    });
    child.stderr?.setEncoding("utf8");
    child.stderr?.on("data", (text: string) => {
      if (current()) { extension.outputChannel.appendLine(`${this.options.label} subscriber ${topic}: ${text.trim()}`); }
    });
    child.stdin?.on("error", error => fail(`Control pipe failed: ${error.message}`));
    child.on("error", error => fail(`Process failed: ${error.message}`));
    child.on("close", (code, signal) => {
      if (current()) {
        this.sessions.delete(topic);
        extension.outputChannel.appendLine(`${this.options.label} subscription ${topic} exited (${signal || code}).`);
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
    return Number.isFinite(rate) ? Math.min(this.options.maxRate, Math.max(this.options.minRate, rate)) : this.options.defaultRate;
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

export class ImageSubscriptionManager extends BinarySubscriptionManager {
  constructor() {
    super({ label: "Image", kind: "image", defaultRate: 5, minRate: 1, maxRate: 30 });
  }
}

export class PointCloudSubscriptionManager extends BinarySubscriptionManager {
  constructor() {
    super({ label: "PointCloud2", kind: "point cloud", defaultRate: 1, minRate: 0.2, maxRate: 5 });
  }
}