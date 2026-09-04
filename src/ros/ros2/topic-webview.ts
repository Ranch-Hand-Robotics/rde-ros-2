// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as crypto from "crypto";
import * as vscode from "vscode";

import * as topicMonitor from "./topic-monitor";
import { isImageType, TopicMessage } from "./topic-types";

interface TopicWebviewMessage {
  command?: string;
  rateHz?: number;
  maxMessages?: number;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function imageDataToBase64(value: unknown): string | undefined {
  if (typeof value === "string") {
    return value;
  }
  if (Array.isArray(value)) {
    return Buffer.from(value).toString("base64");
  }
  if (Buffer.isBuffer(value) || value instanceof Uint8Array) {
    return Buffer.from(value).toString("base64");
  }
  if (isRecord(value) && Array.isArray(value.data)) {
    return Buffer.from(value.data).toString("base64");
  }
  return undefined;
}

export function isConsoleTopic(topicType: string): boolean {
  return topicType === "std_msgs/msg/String" || topicType === "rcl_interfaces/msg/Log" || topicType === "rosgraph_msgs/msg/Log";
}

function formatRosLogMessage(data: Record<string, unknown>): string | undefined {
  if (typeof data.msg !== "string") return undefined;
  const level = new Map([[10, "DEBUG"], [20, "INFO"], [30, "WARN"], [40, "ERROR"], [50, "FATAL"]]).get(Number(data.level)) ?? "LOG";
  const source = typeof data.name === "string" && data.name ? data.name : "rosout";
  return `[${level}] ${source}: ${data.msg}`;
}

function escapeHtml(value: string): string {
  return value
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");
}

export function prepareTopicMessage(message: TopicMessage, topicType: string): TopicMessage {
  if (topicType === "std_msgs/msg/String" && isRecord(message.data) && typeof message.data.data === "string") {
    return { ...message, data: message.data.data };
  }

  if ((topicType === "rcl_interfaces/msg/Log" || topicType === "rosgraph_msgs/msg/Log") && isRecord(message.data)) {
    const logMessage = formatRosLogMessage(message.data);
    if (logMessage) return { ...message, data: logMessage };
  }

  if (!isRecord(message.data)) {
    return message;
  }

  const encodedData = imageDataToBase64(message.data.data);
  if (!encodedData) return message;

  if (topicType === "sensor_msgs/msg/CompressedImage") {
    return {
      ...message,
      data: {
        ...message.data,
        data: encodedData
      }
    };
  }

  if (topicType === "sensor_msgs/msg/Image") {
    return {
      ...message,
      data: {
        ...message.data,
        data: encodedData
      }
    };
  }

  return message;
}

export class TopicMessageRingBuffer {
  private readonly messages: TopicMessage[] = [];
  private nextIndex = 0;

  constructor(private capacity: number) {}

  public push(message: TopicMessage): void {
    if (this.capacity <= 0) {
      return;
    }

    if (this.messages.length < this.capacity) {
      this.messages.push(message);
      return;
    }

    this.messages[this.nextIndex] = message;
    this.nextIndex = (this.nextIndex + 1) % this.capacity;
  }

  public clear(): void {
    this.messages.length = 0;
    this.nextIndex = 0;
  }

  public resize(capacity: number): void {
    const retainedMessages = this.values().slice(-Math.max(0, capacity));
    this.capacity = capacity;
    this.messages.length = 0;
    this.messages.push(...retainedMessages);
    this.nextIndex = 0;
  }

  public values(): TopicMessage[] {
    if (this.messages.length < this.capacity || this.nextIndex === 0) {
      return [...this.messages];
    }

    return [...this.messages.slice(this.nextIndex), ...this.messages.slice(0, this.nextIndex)];
  }
}

export function createTopicMonitorHtml(
  cspSource: string,
  topicName: string,
  topicType: string,
  maxMessages: number = 100,
  nonce: string = crypto.randomBytes(16).toString("base64")
): string {
  const escapedTopicName = escapeHtml(topicName);
  const escapedTopicType = escapeHtml(topicType);
  const isImageTopic = isImageType(topicType);
  const isConsoleStream = isConsoleTopic(topicType);
  const isCompressedImage = topicType === "sensor_msgs/msg/CompressedImage";

  return `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta http-equiv="Content-Security-Policy" content="default-src 'none'; img-src data:; style-src ${cspSource} 'unsafe-inline'; script-src 'nonce-${nonce}';">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Topic Monitor: ${escapedTopicName}</title>
  <style>
    :root {
      color-scheme: light dark;
      --surface: color-mix(in srgb, var(--vscode-editor-background) 92%, var(--vscode-foreground) 8%);
      --surface-raised: color-mix(in srgb, var(--vscode-editor-background) 96%, var(--vscode-foreground) 4%);
      --outline: color-mix(in srgb, var(--vscode-panel-border) 75%, transparent);
      --accent: var(--vscode-focusBorder, #007acc);
      --success: var(--vscode-testing-iconPassed, #3fb950);
      --muted: var(--vscode-descriptionForeground);
      --shadow: 0 12px 32px color-mix(in srgb, #000 22%, transparent);
    }

    * {
      box-sizing: border-box;
    }

    body {
      min-width: 300px;
      margin: 0;
      color: var(--vscode-foreground);
      background:
        radial-gradient(circle at 12% -10%, color-mix(in srgb, var(--accent) 18%, transparent), transparent 34rem),
        var(--vscode-editor-background);
      font-family: var(--vscode-font-family);
    }

    button {
      font: inherit;
    }

    .shell {
      width: min(1100px, 100%);
      min-height: 100vh;
      margin: 0 auto;
      padding: 14px clamp(12px, 3vw, 28px) 24px;
    }

    .hero {
      position: relative;
      overflow: hidden;
      display: flex;
      align-items: center;
      gap: 12px;
      min-height: 64px;
      padding: 10px 14px;
      border: 1px solid var(--outline);
      border-radius: 16px;
      background: linear-gradient(135deg, var(--surface-raised), var(--surface));
      box-shadow: var(--shadow);
    }

    .hero::after {
      position: absolute;
      top: -70px;
      right: -50px;
      width: 160px;
      height: 160px;
      border: 1px solid color-mix(in srgb, var(--accent) 24%, transparent);
      border-radius: 50%;
      content: "";
      box-shadow: 0 0 0 22px color-mix(in srgb, var(--accent) 4%, transparent);
      pointer-events: none;
    }

    .eyebrow {
      display: flex;
      align-items: center;
      gap: 8px;
      flex: 0 0 auto;
      margin: 0;
      color: var(--muted);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.12em;
      text-transform: uppercase;
    }

    .status-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--success);
      box-shadow: 0 0 0 4px color-mix(in srgb, var(--success) 15%, transparent);
      animation: pulse 2.4s ease-out infinite;
    }

    .paused .status-dot {
      background: var(--vscode-charts-yellow, #cca700);
      box-shadow: none;
      animation: none;
    }

    .topic-name {
      position: relative;
      z-index: 1;
      margin: 0;
      font-family: var(--vscode-editor-font-family);
      flex: 1 1 auto;
      min-width: 0;
      font-size: clamp(18px, 3vw, 24px);
      font-weight: 650;
      letter-spacing: -0.04em;
      line-height: 1.1;
      overflow-wrap: anywhere;
    }

    .topic-type {
      position: relative;
      z-index: 1;
      display: inline-flex;
      flex: 0 1 auto;
      margin: 0 28px 0 0;
      padding: 4px 7px;
      border: 1px solid var(--outline);
      border-radius: 999px;
      color: var(--muted);
      background: color-mix(in srgb, var(--vscode-editor-background) 72%, transparent);
      font-family: var(--vscode-editor-font-family);
      font-size: 12px;
    }

    .dashboard {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin: 10px 0;
    }

    .metric {
      padding: 10px 12px;
      border: 1px solid var(--outline);
      border-radius: 12px;
      background: var(--surface-raised);
    }

    .metric-label {
      display: block;
      margin-bottom: 4px;
      color: var(--muted);
      font-size: 11px;
      font-weight: 650;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .metric-value {
      font-family: var(--vscode-editor-font-family);
      font-size: 16px;
      font-variant-numeric: tabular-nums;
    }

    .toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin: 12px 0 8px;
    }

    .stream-title {
      margin: 0;
      font-size: 16px;
      font-weight: 650;
    }

    .controls {
      display: flex;
      gap: 8px;
    }

    .control {
      display: inline-flex;
      align-items: center;
      gap: 7px;
      min-height: 34px;
      padding: 7px 12px;
      border: 1px solid transparent;
      border-radius: 7px;
      color: var(--vscode-button-foreground);
      background: var(--vscode-button-background);
      cursor: pointer;
      transition: transform 120ms ease, background 120ms ease, border-color 120ms ease;
    }

    .control:hover {
      background: var(--vscode-button-hoverBackground);
      transform: translateY(-1px);
    }

    .control:focus-visible {
      outline: 2px solid var(--accent);
      outline-offset: 2px;
    }

    .control.secondary {
      border-color: var(--outline);
      color: var(--vscode-button-secondaryForeground);
      background: var(--vscode-button-secondaryBackground);
    }

    .control.secondary:hover {
      background: var(--vscode-button-secondaryHoverBackground);
    }

    .icon {
      width: 14px;
      height: 14px;
      fill: currentColor;
    }

    .messages {
      display: grid;
      gap: 10px;
    }

    .console-message {
      margin: 0;
      padding: 2px 4px;
      overflow: auto;
      color: var(--vscode-terminal-foreground, var(--vscode-foreground));
      background: var(--vscode-terminal-background, var(--vscode-editor-background));
      font-family: var(--vscode-editor-font-family), monospace;
      font-size: var(--vscode-editor-font-size, 12px);
      line-height: 1.4;
      tab-size: 2;
      white-space: pre-wrap;
      word-break: break-word;
    }

    .ansi-bold { font-weight: 700; }
    .ansi-fg-30 { color: var(--vscode-terminal-ansiBlack, #000000); }
    .ansi-fg-31 { color: var(--vscode-terminal-ansiRed, #cd3131); }
    .ansi-fg-32 { color: var(--vscode-terminal-ansiGreen, #0dbc79); }
    .ansi-fg-33 { color: var(--vscode-terminal-ansiYellow, #e5e510); }
    .ansi-fg-34 { color: var(--vscode-terminal-ansiBlue, #2472c8); }
    .ansi-fg-35 { color: var(--vscode-terminal-ansiMagenta, #bc3fbc); }
    .ansi-fg-36 { color: var(--vscode-terminal-ansiCyan, #11a8cd); }
    .ansi-fg-37 { color: var(--vscode-terminal-ansiWhite, #e5e5e5); }
    .ansi-fg-90 { color: var(--vscode-terminal-ansiBrightBlack, #666666); }
    .ansi-fg-91 { color: var(--vscode-terminal-ansiBrightRed, #cd3131); }
    .ansi-fg-92 { color: var(--vscode-terminal-ansiBrightGreen, #23d18b); }
    .ansi-fg-93 { color: var(--vscode-terminal-ansiBrightYellow, #f5f543); }
    .ansi-fg-94 { color: var(--vscode-terminal-ansiBrightBlue, #3b8eea); }
    .ansi-fg-95 { color: var(--vscode-terminal-ansiBrightMagenta, #d670d6); }
    .ansi-fg-96 { color: var(--vscode-terminal-ansiBrightCyan, #29b8db); }
    .ansi-fg-97 { color: var(--vscode-terminal-ansiBrightWhite, #e5e5e5); }

    .latest-image {
      overflow: hidden;
      border: 1px solid var(--outline);
      border-radius: 12px;
      background: var(--surface-raised);
    }

    .image-metadata {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      gap: 1px;
      border-bottom: 1px solid var(--outline);
      background: var(--outline);
    }

    .image-meta-item {
      min-width: 0;
      padding: 8px 10px;
      background: var(--surface);
    }

    .image-meta-label {
      display: block;
      margin-bottom: 3px;
      color: var(--muted);
      font-size: 10px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .image-meta-value {
      display: block;
      overflow: hidden;
      color: var(--vscode-foreground);
      font-family: var(--vscode-editor-font-family), monospace;
      font-size: 12px;
      text-overflow: ellipsis;
      white-space: nowrap;
    }

    .stream-rate, .buffer-length {
      display: none;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 12px;
    }

    .stream-rate input, .buffer-length input { accent-color: var(--accent); }
    .stream-rate output, .buffer-length output { min-width: 3.5em; color: var(--vscode-foreground); font-family: var(--vscode-editor-font-family), monospace; }
    body[data-image-topic="true"] .stream-rate, body[data-console-stream="true"] .stream-rate, .buffer-length { display: inline-flex; }

    .message {
      overflow: hidden;
      border: 1px solid var(--outline);
      border-radius: 12px;
      background: var(--surface-raised);
      animation: enter 160ms ease-out;
    }

    .message-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 7px 10px;
      border-bottom: 1px solid var(--outline);
      color: var(--muted);
      background: var(--surface);
      font-size: 11px;
      font-variant-numeric: tabular-nums;
    }

    .message-index {
      color: var(--vscode-foreground);
      font-family: var(--vscode-editor-font-family);
      font-weight: 650;
    }

    .message-content {
      margin: 0;
      padding: 10px;
      overflow: auto;
      font-family: var(--vscode-editor-font-family), monospace;
      font-size: var(--vscode-editor-font-size, 12px);
      line-height: 1.55;
      tab-size: 2;
      white-space: pre-wrap;
      word-break: break-word;
    }

    .json-key { color: var(--vscode-symbolIcon-propertyForeground, #9cdcfe); }
    .json-string { color: var(--vscode-symbolIcon-stringForeground, #ce9178); }
    .json-number { color: var(--vscode-symbolIcon-numberForeground, #b5cea8); }
    .json-boolean, .json-null { color: var(--vscode-symbolIcon-booleanForeground, #569cd6); }

    .image-wrap {
      padding: 14px;
      text-align: center;
      background-color: color-mix(in srgb, var(--vscode-editor-background) 94%, var(--accent) 6%);
      background-image:
        linear-gradient(90deg, color-mix(in srgb, var(--accent) 18%, transparent) 1px, transparent 1px),
        linear-gradient(color-mix(in srgb, var(--accent) 18%, transparent) 1px, transparent 1px);
      background-size: 24px 24px;
    }

    .image-wrap img, .image-wrap canvas {
      display: block;
      max-width: 100%;
      max-height: 70vh;
      margin: auto;
      border-radius: 8px;
      box-shadow: var(--shadow);
    }

    .image-wrap [hidden] {
      display: none !important;
    }

    .empty {
      display: grid;
      min-height: 260px;
      place-items: center;
      border: 1px dashed var(--outline);
      border-radius: 14px;
      color: var(--muted);
      text-align: center;
      background: color-mix(in srgb, var(--surface-raised) 72%, transparent);
    }

    .empty-orbit {
      position: relative;
      width: 58px;
      height: 58px;
      margin: 0 auto 18px;
      border: 1px solid color-mix(in srgb, var(--accent) 35%, transparent);
      border-radius: 50%;
    }

    .empty-orbit::before {
      position: absolute;
      inset: 16px;
      border-radius: 50%;
      background: var(--accent);
      content: "";
      box-shadow: 0 0 24px color-mix(in srgb, var(--accent) 55%, transparent);
    }

    .empty-title {
      margin-bottom: 6px;
      color: var(--vscode-foreground);
      font-weight: 650;
    }

    @keyframes pulse {
      0% { box-shadow: 0 0 0 0 color-mix(in srgb, var(--success) 30%, transparent); }
      70%, 100% { box-shadow: 0 0 0 8px transparent; }
    }

    @keyframes enter {
      from { opacity: 0; transform: translateY(5px); }
      to { opacity: 1; transform: translateY(0); }
    }

    @media (max-width: 600px) {
      .hero {
        align-items: flex-start;
        flex-wrap: wrap;
      }

      .topic-name {
        flex-basis: calc(100% - 110px);
      }

      .topic-type {
        margin-left: 20px;
      }

      .dashboard {
        grid-template-columns: 1fr;
      }

      .toolbar {
        align-items: stretch;
        flex-direction: column;
      }

      .controls {
        display: grid;
        grid-template-columns: 1fr 1fr;
      }

      .control {
        justify-content: center;
      }
    }

    @media (prefers-reduced-motion: reduce) {
      *, *::before, *::after {
        scroll-behavior: auto !important;
        animation-duration: 0.01ms !important;
        animation-iteration-count: 1 !important;
        transition-duration: 0.01ms !important;
      }
    }
  </style>
</head>
<body data-image-topic="${isImageTopic}" data-console-stream="${isConsoleStream}" data-compressed-image="${isCompressedImage}">
  <main class="shell">
    <header class="hero" id="hero">
      <div class="eyebrow"><span class="status-dot" aria-hidden="true"></span><span id="statusText">Live stream</span></div>
      <h1 class="topic-name">${escapedTopicName}</h1>
      <div class="topic-type">${escapedTopicType}</div>
    </header>

    <section class="dashboard" aria-label="Topic statistics">
      <div class="metric"><span class="metric-label">Messages</span><span class="metric-value" id="messageCount">0</span></div>
      <div class="metric"><span class="metric-label">Last update</span><span class="metric-value" id="lastUpdate">Waiting</span></div>
      <div class="metric"><span class="metric-label">Buffer</span><span class="metric-value" id="bufferUsage">0 / ${maxMessages}</span></div>
    </section>

    <div class="toolbar">
      <h2 class="stream-title">Message stream</h2>
      <div class="controls">
        <label class="stream-rate" for="streamRate">Refresh <input id="streamRate" type="range" min="1" max="30" value="5" step="1"><output id="streamRateValue">5 Hz</output></label>
        <label class="buffer-length" for="bufferLength">Buffer <input id="bufferLength" type="range" min="1" max="500" value="${maxMessages}" step="1"><output id="bufferLengthValue">${maxMessages}</output></label>
        <button class="control" id="pauseButton" type="button">
          <svg class="icon" viewBox="0 0 16 16" aria-hidden="true"><path d="M3 2.5h3.5v11H3v-11Zm6.5 0H13v11H9.5v-11Z"/></svg>
          <span id="pauseLabel">Pause</span>
        </button>
        <button class="control secondary" id="clearButton" type="button">
          <svg class="icon" viewBox="0 0 16 16" aria-hidden="true"><path d="M6 1h4l1 2h3v1H2V3h3l1-2Zm-2 4h8l-.7 9H4.7L4 5Zm2 2v5h1V7H6Zm3 0v5h1V7H9Z"/></svg>
          <span>Clear</span>
        </button>
      </div>
    </div>

    <section class="messages" id="messageContainer" aria-live="polite" aria-label="Topic messages"></section>
  </main>

  <script nonce="${nonce}">
    const vscode = acquireVsCodeApi();
    let maxMessages = ${maxMessages};
    const isImageTopic = document.body.dataset.imageTopic === "true";
    const isCompressedImage = document.body.dataset.compressedImage === "true";
    const hero = document.getElementById("hero");
    const statusText = document.getElementById("statusText");
    const pauseButton = document.getElementById("pauseButton");
    const pauseLabel = document.getElementById("pauseLabel");
    const clearButton = document.getElementById("clearButton");
    const streamRate = document.getElementById("streamRate");
    const streamRateValue = document.getElementById("streamRateValue");
    const bufferLength = document.getElementById("bufferLength");
    const bufferLengthValue = document.getElementById("bufferLengthValue");
    const messageCount = document.getElementById("messageCount");
    const lastUpdate = document.getElementById("lastUpdate");
    const bufferUsage = document.getElementById("bufferUsage");
    const messageContainer = document.getElementById("messageContainer");
    let isPaused = false;
    let messages = [];
    let nextMessageIndex = 0;

    function addMessage(message) {
      if (messages.length < maxMessages) {
        messages.push(message);
        return;
      }

      messages[nextMessageIndex] = message;
      nextMessageIndex = (nextMessageIndex + 1) % maxMessages;
    }

    function orderedMessages() {
      if (messages.length < maxMessages || nextMessageIndex === 0) {
        return messages;
      }

      return messages.slice(nextMessageIndex).concat(messages.slice(0, nextMessageIndex));
    }

    function resizeMessageBuffer(capacity) {
      messages = orderedMessages().slice(-capacity);
      maxMessages = capacity;
      nextMessageIndex = 0;
    }

    function formatTimestamp(timestamp) {
      const date = new Date(timestamp);
      return date.toLocaleTimeString([], {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
        fractionalSecondDigits: 3
      });
    }

    function imageSource(message) {
      const encodedData = message && message.data && message.data.data;
      if (!isCompressedImage || typeof encodedData !== "string") return undefined;

      const format = String(message.data.format || "").toLowerCase();
      const mime = format.includes("png") ? "image/png" : format.includes("webp") ? "image/webp" : "image/jpeg";
      return "data:" + mime + ";base64," + encodedData;
    }

    function imageMetadata(message) {
      const data = message.data || {};
      const stamp = data.header && data.header.stamp;
      return [
        ["Size", Number(data.width) + " × " + Number(data.height)],
        ["Encoding", String(data.encoding || "Unknown")],
        ["Step", String(data.step || "Unknown") + " bytes"],
        ["Frame", String(data.header && data.header.frame_id || "None")],
        ["Timestamp", stamp ? String(stamp.sec || 0) + "." + String(stamp.nanosec || 0).padStart(9, "0") : "Unavailable"]
      ];
    }

    function imageBytes(value) {
      if (typeof value === "string") {
        try {
          return Uint8Array.from(atob(value), character => character.charCodeAt(0));
        } catch {
          return undefined;
        }
      }
      if (Array.isArray(value)) return Uint8Array.from(value);
      return undefined;
    }

    function rawImageFormat(encoding) {
      const normalized = encoding.toLowerCase();
      const colorMatch = normalized.match(/^(rgb|bgr|rgba|bgra)(8|16)$/);
      if (colorMatch) return { order: colorMatch[1], channels: colorMatch[1].length, depth: Number(colorMatch[2]) };
      const monoMatch = normalized.match(/^mono(8|16)$/);
      if (monoMatch) return { order: "mono", channels: 1, depth: Number(monoMatch[1]) };
      const cvMatch = normalized.match(/^(8|16)uc([1-4])$/);
      if (cvMatch) return { order: "generic", channels: Number(cvMatch[2]), depth: Number(cvMatch[1]) };
      if (/^(yuv422|yuyv|yuy2|uyvy)$/.test(normalized)) return { order: normalized, channels: 2, depth: 8 };
      if (/^bayer_(rggb|bggr|gbrg|grbg)(8|16)$/.test(normalized)) return { order: normalized, channels: 1, depth: Number(normalized.slice(-2)) };
      return undefined;
    }

    function clampColor(value) {
      return Math.max(0, Math.min(255, value));
    }

    function yuvToRgb(y, u, v) {
      const red = y + 1.402 * (v - 128);
      const green = y - 0.344136 * (u - 128) - 0.714136 * (v - 128);
      const blue = y + 1.772 * (u - 128);
      return [clampColor(red), clampColor(green), clampColor(blue)];
    }

    function drawRawImage(canvas, message) {
      const data = message.data;
      if (!data || !Number.isInteger(data.width) || !Number.isInteger(data.height)) return "Image dimensions are unavailable.";
      const format = rawImageFormat(String(data.encoding || ""));
      if (!format) return "Unsupported image encoding: " + String(data.encoding || "unknown");
      const bytes = imageBytes(data.data);
      if (!bytes) return "Image data is unavailable.";
      const bytesPerChannel = format.depth / 8;
      const minimumStep = data.width * format.channels * bytesPerChannel;
      const step = Number.isInteger(data.step) ? data.step : minimumStep;
      if (step < minimumStep || bytes.length < step * data.height) return "Image data is incomplete.";

      canvas.width = data.width;
      canvas.height = data.height;
      const context = canvas.getContext("2d");
      if (!context) return "Canvas rendering is unavailable.";
      const output = context.createImageData(data.width, data.height);
      const littleEndian = Number(data.is_bigendian) !== 1;
      const sample = (offset) => format.depth === 8
        ? bytes[offset]
        : (littleEndian ? bytes[offset + 1] : bytes[offset]);
      for (let y = 0; y < data.height; y += 1) {
        for (let x = 0; x < data.width; x += 1) {
          const sourceOffset = y * step + x * format.channels * bytesPerChannel;
          const targetOffset = (y * data.width + x) * 4;
          if (format.order === "yuv422" || format.order === "yuyv" || format.order === "yuy2" || format.order === "uyvy") {
            const pairOffset = y * step + Math.floor(x / 2) * 4;
            const isUyvy = format.order === "uyvy";
            const yValue = bytes[pairOffset + (isUyvy ? (x % 2 === 0 ? 1 : 3) : x % 2 * 2)];
            const u = bytes[pairOffset + (isUyvy ? 0 : 1)];
            const v = bytes[pairOffset + (isUyvy ? 2 : 3)];
            const [red, green, blue] = yuvToRgb(yValue, u, v);
            output.data[targetOffset] = red;
            output.data[targetOffset + 1] = green;
            output.data[targetOffset + 2] = blue;
          } else if (format.order === "mono" || format.order.startsWith("bayer_") || format.order === "generic" && format.channels <= 2) {
            const intensity = sample(sourceOffset);
            output.data[targetOffset] = intensity;
            output.data[targetOffset + 1] = intensity;
            output.data[targetOffset + 2] = intensity;
            output.data[targetOffset + 3] = format.channels === 2 ? sample(sourceOffset + bytesPerChannel) : 255;
          } else {
            const isBgr = format.order.startsWith("bgr") || format.order === "generic";
            output.data[targetOffset] = sample(sourceOffset + (isBgr ? 2 * bytesPerChannel : 0));
            output.data[targetOffset + 1] = sample(sourceOffset + bytesPerChannel);
            output.data[targetOffset + 2] = sample(sourceOffset + (isBgr ? 0 : 2 * bytesPerChannel));
            output.data[targetOffset + 3] = format.channels === 4 ? sample(sourceOffset + 3 * bytesPerChannel) : 255;
          }
        }
      }
      context.putImageData(output, 0, 0);
      return undefined;
    }

    let latestImage;

    function createLatestImage() {
      const latest = document.createElement("article");
      latest.className = "latest-image";
      const metadata = document.createElement("div");
      metadata.className = "image-metadata";
      const wrap = document.createElement("div");
      wrap.className = "image-wrap";
      const image = document.createElement("img");
      image.alt = "Latest image from the topic";
      const canvas = document.createElement("canvas");
      const notice = document.createElement("div");
      notice.hidden = true;
      image.hidden = true;
      canvas.hidden = true;
      wrap.append(image, canvas, notice);
      latest.append(metadata, wrap);
      return { element: latest, metadata, image, canvas, notice };
    }

    function updateLatestImage(message) {
      if (!latestImage) {
        latestImage = createLatestImage();
        messageContainer.replaceChildren(latestImage.element);
      }
      latestImage.metadata.replaceChildren(...imageMetadata(message).map(([label, value]) => {
        const item = document.createElement("div");
        item.className = "image-meta-item";
        const name = document.createElement("span");
        name.className = "image-meta-label";
        name.textContent = label;
        const content = document.createElement("span");
        content.className = "image-meta-value";
        content.textContent = value;
        item.append(name, content);
        return item;
      }));
      const source = imageSource(message);
      latestImage.notice.hidden = true;
      if (source) {
        latestImage.image.hidden = false;
        latestImage.canvas.hidden = true;
        latestImage.image.src = source;
        return;
      }
      latestImage.image.hidden = true;
      latestImage.canvas.hidden = false;
      const error = drawRawImage(latestImage.canvas, message);
      latestImage.notice.hidden = !error;
      latestImage.notice.textContent = error || "";
    }

    function appendAnsiText(element, text) {
      let foreground;
      let bold = false;
      let position = 0;
      const sgr = /\\x1b\\[([0-9;]*)m/g;

      function appendText(value) {
        if (!value) return;
        const span = document.createElement("span");
        if (foreground !== undefined) span.classList.add("ansi-fg-" + String(foreground));
        if (bold) span.classList.add("ansi-bold");
        span.textContent = value;
        element.appendChild(span);
      }

      for (let match; (match = sgr.exec(text)) !== null;) {
        appendText(text.slice(position, match.index));
        const codes = match[1] === "" ? [0] : match[1].split(";").map(Number);
        for (const code of codes) {
          if (code === 0) {
            foreground = undefined;
            bold = false;
          } else if (code === 1) {
            bold = true;
          } else if (code === 22) {
            bold = false;
          } else if ((code >= 30 && code <= 37) || (code >= 90 && code <= 97)) {
            foreground = code;
          } else if (code === 39) {
            foreground = undefined;
          }
        }
        position = sgr.lastIndex;
      }
      appendText(text.slice(position).replace(/\\x1b\\[[^m]*m/g, ""));
    }

    function createMessageCard(message, index) {
      if (typeof message.data === "string") {
        const consoleMessage = document.createElement("pre");
        consoleMessage.className = "console-message";
        appendAnsiText(consoleMessage, message.data);
        return consoleMessage;
      }

      const article = document.createElement("article");
      article.className = "message";

      const header = document.createElement("div");
      header.className = "message-header";
      const number = document.createElement("span");
      number.className = "message-index";
      number.textContent = "#" + String(index);
      const timestamp = document.createElement("time");
      timestamp.dateTime = new Date(message.timestamp).toISOString();
      timestamp.textContent = formatTimestamp(message.timestamp);
      header.append(number, timestamp);
      article.appendChild(header);

      const source = imageSource(message);
      if (source) {
        const wrap = document.createElement("div");
        wrap.className = "image-wrap";
        const image = document.createElement("img");
        image.src = source;
        image.alt = "Latest compressed image from the topic";
        wrap.appendChild(image);
        article.appendChild(wrap);
      } else {
        const pre = document.createElement("pre");
        pre.className = "message-content";
        const serialized = JSON.stringify(message.data, null, 2);
        pre.textContent = serialized === undefined ? String(message.data) : serialized;
        article.appendChild(pre);
      }

      return article;
    }

    function render() {
      messageCount.textContent = String(messages.length);
      bufferUsage.textContent = String(messages.length) + " / " + String(maxMessages);

      if (messages.length === 0) {
        latestImage = undefined;
        messageContainer.replaceChildren();
        const empty = document.createElement("div");
        empty.className = "empty";
        const content = document.createElement("div");
        const orbit = document.createElement("div");
        orbit.className = "empty-orbit";
        orbit.setAttribute("aria-hidden", "true");
        const title = document.createElement("div");
        title.className = "empty-title";
        title.textContent = "Listening for messages";
        const detail = document.createElement("div");
        detail.textContent = "The stream will appear here as soon as data arrives.";
        content.append(orbit, title, detail);
        empty.appendChild(content);
        messageContainer.appendChild(empty);
        lastUpdate.textContent = "Waiting";
        return;
      }

      const ordered = orderedMessages();
      lastUpdate.textContent = formatTimestamp(ordered[ordered.length - 1].timestamp);
      if (isImageTopic) {
        updateLatestImage(ordered[ordered.length - 1]);
        return;
      }
      messageContainer.replaceChildren();
      const fragment = document.createDocumentFragment();
      for (let index = ordered.length - 1; index >= 0; index -= 1) {
        fragment.appendChild(createMessageCard(ordered[index], index + 1));
      }
      messageContainer.appendChild(fragment);
    }

    function setPaused(paused) {
      isPaused = paused;
      hero.classList.toggle("paused", isPaused);
      pauseLabel.textContent = isPaused ? "Resume" : "Pause";
      statusText.textContent = isPaused ? "Stream paused" : "Live stream";
      pauseButton.setAttribute("aria-pressed", String(isPaused));
    }

    pauseButton.addEventListener("click", () => {
      setPaused(!isPaused);
      vscode.postMessage({ command: isPaused ? "pause" : "resume" });
    });

    clearButton.addEventListener("click", () => {
      messages = [];
      nextMessageIndex = 0;
      render();
      vscode.postMessage({ command: "clear" });
    });

    streamRate.addEventListener("input", () => {
      streamRateValue.textContent = streamRate.value + " Hz";
      vscode.postMessage({ command: "setRefreshRate", rateHz: Number(streamRate.value) });
    });

    bufferLength.addEventListener("input", () => {
      const capacity = Number(bufferLength.value);
      bufferLengthValue.textContent = String(capacity);
      resizeMessageBuffer(capacity);
      render();
      vscode.postMessage({ command: "setBufferLength", maxMessages: capacity });
    });

    window.addEventListener("message", event => {
      const message = event.data;
      if (message.command === "newMessage") {
        addMessage(message.message);
        render();
      } else if (message.command === "history") {
        messages = Array.isArray(message.messages) ? message.messages.slice(-maxMessages) : [];
        nextMessageIndex = messages.length % maxMessages;
        render();
      } else if (message.command === "setPaused") {
        setPaused(message.paused === true);
      }
    });

    render();
    vscode.postMessage({ command: "getHistory" });
  </script>
</body>
</html>`;
}

/**
 * Manages webview panels for topic monitoring.
 */
export class TopicWebviewManager implements vscode.Disposable {
  private readonly panels = new Map<string, vscode.WebviewPanel>();
  private readonly echoManager = new topicMonitor.TopicEchoManager();
  private readonly messageBuffers = new Map<string, TopicMessageRingBuffer>();
  private readonly topicTypes = new Map<string, string>();
  private readonly lastImageDelivery = new Map<string, number>();
  private readonly refreshIntervals = new Map<string, number>();
  private readonly pausedTopics = new Set<string>();
  private monitoringEnabled = false;
  private readonly maxMessagesPerTopic = 100;
  private readonly maxMessagesPerImageTopic = 1;
  private readonly defaultImagePreviewIntervalMs = 200;

  constructor(
    private readonly context: vscode.ExtensionContext,
    private readonly onTopicMonitorClosed?: (topicName: string) => void
  ) {}

  public openTopicMonitor(topicName: string, topicType: string): void {
    const existingPanel = this.panels.get(topicName);
    if (existingPanel) {
      existingPanel.reveal(vscode.ViewColumn.Beside);
      return;
    }

    const panel = vscode.window.createWebviewPanel(
      `topicMonitor_${topicName}`,
      `Topic: ${topicName}`,
      vscode.ViewColumn.Beside,
      {
        enableScripts: true,
        retainContextWhenHidden: true,
        localResourceRoots: []
      }
    );

    this.panels.set(topicName, panel);
    const messageLimit = isImageType(topicType)
      ? this.maxMessagesPerImageTopic
      : this.maxMessagesPerTopic;
    this.messageBuffers.set(topicName, new TopicMessageRingBuffer(messageLimit));
    this.topicTypes.set(topicName, topicType);
    if (isImageType(topicType) || isConsoleTopic(topicType)) {
      this.refreshIntervals.set(topicName, this.defaultImagePreviewIntervalMs);
    }
    panel.webview.html = createTopicMonitorHtml(
      panel.webview.cspSource,
      topicName,
      topicType,
      messageLimit
    );

    panel.webview.onDidReceiveMessage(
      (message: TopicWebviewMessage) => this.handleWebviewMessage(topicName, message),
      undefined,
      this.context.subscriptions
    );

    panel.onDidDispose(
      () => this.releaseTopicMonitor(topicName),
      undefined,
      this.context.subscriptions
    );

    this.startMonitoring(topicName);
  }

  /** Synchronizes global topic watcher play/pause state with open monitor panes. */
  public setMonitoringEnabled(enabled: boolean): void {
    if (this.monitoringEnabled === enabled) {
      return;
    }

    this.monitoringEnabled = enabled;
    for (const topicName of this.panels.keys()) {
      if (enabled) {
        this.pausedTopics.delete(topicName);
        this.startMonitoring(topicName);
      } else {
        this.echoManager.stopEcho(topicName);
        this.pausedTopics.add(topicName);
      }
      void this.panels.get(topicName)?.webview.postMessage({ command: "setPaused", paused: !enabled });
    }
  }

  public closeTopicMonitor(topicName: string): void {
    const panel = this.panels.get(topicName);
    this.releaseTopicMonitor(topicName);
    panel?.dispose();
  }

  public closeAll(): void {
    for (const topicName of Array.from(this.panels.keys())) {
      this.closeTopicMonitor(topicName);
    }
  }

  private releaseTopicMonitor(topicName: string): void {
    if (!this.panels.has(topicName)) {
      return;
    }

    this.echoManager.stopEcho(topicName);
    this.panels.delete(topicName);
    this.messageBuffers.delete(topicName);
    this.topicTypes.delete(topicName);
    this.lastImageDelivery.delete(topicName);
    this.refreshIntervals.delete(topicName);
    this.pausedTopics.delete(topicName);
    this.onTopicMonitorClosed?.(topicName);
  }

  private startMonitoring(topicName: string): void {
    if (this.pausedTopics.has(topicName)) {
      return;
    }

    this.echoManager.startEcho(topicName, (message: TopicMessage) => {
      if (this.pausedTopics.has(topicName)) {
        return;
      }

      const topicType = this.topicTypes.get(topicName) ?? "";
      if (isImageType(topicType) || isConsoleTopic(topicType)) {
        const now = Date.now();
        const lastDelivery = this.lastImageDelivery.get(topicName) ?? 0;
        const interval = this.refreshIntervals.get(topicName) ?? this.defaultImagePreviewIntervalMs;
        if (now - lastDelivery < interval) {
          return;
        }
        this.lastImageDelivery.set(topicName, now);
      }

      const preparedMessage = prepareTopicMessage(message, topicType);
      const buffer = this.messageBuffers.get(topicName);
      buffer?.push(preparedMessage);

      void this.panels.get(topicName)?.webview.postMessage({
        command: "newMessage",
        message: preparedMessage
      });
    });
  }

  private handleWebviewMessage(topicName: string, message: TopicWebviewMessage): void {
    switch (message.command) {
      case "pause":
        this.pausedTopics.add(topicName);
        this.echoManager.stopEcho(topicName);
        break;
      case "resume":
        this.pausedTopics.delete(topicName);
        this.startMonitoring(topicName);
        break;
      case "clear":
        this.messageBuffers.get(topicName)?.clear();
        break;
      case "setRefreshRate":
        if (typeof message.rateHz === "number" && Number.isFinite(message.rateHz)) {
          const rateHz = Math.min(30, Math.max(1, message.rateHz));
          this.refreshIntervals.set(topicName, 1000 / rateHz);
        }
        break;
      case "setBufferLength":
        if (typeof message.maxMessages === "number" && Number.isInteger(message.maxMessages)) {
          this.messageBuffers.get(topicName)?.resize(Math.min(500, Math.max(1, message.maxMessages)));
        }
        break;
      case "getHistory":
        void this.panels.get(topicName)?.webview.postMessage({
          command: "history",
          messages: this.messageBuffers.get(topicName)?.values() ?? []
        });
        break;
    }
  }

  public dispose(): void {
    this.closeAll();
    this.echoManager.dispose();
  }
}
