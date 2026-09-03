// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as crypto from "crypto";
import * as vscode from "vscode";

import * as topicMonitor from "./topic-monitor";
import { isImageType, TopicMessage } from "./topic-types";

interface TopicWebviewMessage {
  command?: string;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
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
  if (!isRecord(message.data) || !Array.isArray(message.data.data)) {
    return message;
  }

  const byteLength = message.data.data.length;
  if (topicType === "sensor_msgs/msg/CompressedImage") {
    return {
      ...message,
      data: {
        ...message.data,
        data: Buffer.from(message.data.data).toString("base64")
      }
    };
  }

  if (topicType === "sensor_msgs/msg/Image") {
    return {
      ...message,
      data: {
        ...message.data,
        data: `<${byteLength} image bytes omitted from preview>`
      }
    };
  }

  return message;
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
      padding: 28px clamp(16px, 4vw, 48px) 48px;
    }

    .hero {
      position: relative;
      overflow: hidden;
      padding: clamp(20px, 4vw, 34px);
      border: 1px solid var(--outline);
      border-radius: 16px;
      background: linear-gradient(135deg, var(--surface-raised), var(--surface));
      box-shadow: var(--shadow);
    }

    .hero::after {
      position: absolute;
      top: -90px;
      right: -70px;
      width: 230px;
      height: 230px;
      border: 1px solid color-mix(in srgb, var(--accent) 24%, transparent);
      border-radius: 50%;
      content: "";
      box-shadow: 0 0 0 34px color-mix(in srgb, var(--accent) 4%, transparent);
      pointer-events: none;
    }

    .eyebrow {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 12px;
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
      font-size: clamp(24px, 5vw, 42px);
      font-weight: 650;
      letter-spacing: -0.04em;
      line-height: 1.1;
      overflow-wrap: anywhere;
    }

    .topic-type {
      position: relative;
      z-index: 1;
      display: inline-flex;
      margin-top: 14px;
      padding: 5px 9px;
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
      margin: 16px 0;
    }

    .metric {
      padding: 16px 18px;
      border: 1px solid var(--outline);
      border-radius: 12px;
      background: var(--surface-raised);
    }

    .metric-label {
      display: block;
      margin-bottom: 7px;
      color: var(--muted);
      font-size: 11px;
      font-weight: 650;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .metric-value {
      font-family: var(--vscode-editor-font-family);
      font-size: 20px;
      font-variant-numeric: tabular-nums;
    }

    .toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin: 20px 0 12px;
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
      padding: 11px 14px;
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
      padding: 16px;
      overflow: auto;
      font-family: var(--vscode-editor-font-family);
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
      background:
        linear-gradient(45deg, var(--surface) 25%, transparent 25%),
        linear-gradient(-45deg, var(--surface) 25%, transparent 25%);
      background-size: 18px 18px;
    }

    .image-wrap img {
      display: block;
      max-width: 100%;
      max-height: 70vh;
      margin: auto;
      border-radius: 8px;
      box-shadow: var(--shadow);
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
<body data-compressed-image="${isCompressedImage}">
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
    const maxMessages = ${maxMessages};
    const isCompressedImage = document.body.dataset.compressedImage === "true";
    const hero = document.getElementById("hero");
    const statusText = document.getElementById("statusText");
    const pauseButton = document.getElementById("pauseButton");
    const pauseLabel = document.getElementById("pauseLabel");
    const clearButton = document.getElementById("clearButton");
    const messageCount = document.getElementById("messageCount");
    const lastUpdate = document.getElementById("lastUpdate");
    const bufferUsage = document.getElementById("bufferUsage");
    const messageContainer = document.getElementById("messageContainer");
    let isPaused = false;
    let messages = [];

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

    function createMessageCard(message, index) {
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
      messageContainer.replaceChildren();
      messageCount.textContent = String(messages.length);
      bufferUsage.textContent = String(messages.length) + " / " + String(maxMessages);

      if (messages.length === 0) {
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

      lastUpdate.textContent = formatTimestamp(messages[messages.length - 1].timestamp);
      const fragment = document.createDocumentFragment();
      for (let index = messages.length - 1; index >= 0; index -= 1) {
        fragment.appendChild(createMessageCard(messages[index], index + 1));
      }
      messageContainer.appendChild(fragment);
    }

    pauseButton.addEventListener("click", () => {
      isPaused = !isPaused;
      hero.classList.toggle("paused", isPaused);
      pauseLabel.textContent = isPaused ? "Resume" : "Pause";
      statusText.textContent = isPaused ? "Stream paused" : "Live stream";
      pauseButton.setAttribute("aria-pressed", String(isPaused));
      vscode.postMessage({ command: isPaused ? "pause" : "resume" });
    });

    clearButton.addEventListener("click", () => {
      messages = [];
      render();
      vscode.postMessage({ command: "clear" });
    });

    window.addEventListener("message", event => {
      const message = event.data;
      if (message.command === "newMessage") {
        messages.push(message.message);
        if (messages.length > maxMessages) messages.shift();
        render();
      } else if (message.command === "history") {
        messages = Array.isArray(message.messages) ? message.messages.slice(-maxMessages) : [];
        render();
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
  private readonly messageBuffers = new Map<string, TopicMessage[]>();
  private readonly messageLimits = new Map<string, number>();
  private readonly topicTypes = new Map<string, string>();
  private readonly lastImageDelivery = new Map<string, number>();
  private readonly pausedTopics = new Set<string>();
  private readonly maxMessagesPerTopic = 100;
  private readonly maxMessagesPerImageTopic = 1;
  private readonly imagePreviewIntervalMs = 200;

  constructor(private readonly context: vscode.ExtensionContext) {}

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
    this.messageBuffers.set(topicName, []);
    const messageLimit = isImageType(topicType)
      ? this.maxMessagesPerImageTopic
      : this.maxMessagesPerTopic;
    this.messageLimits.set(topicName, messageLimit);
    this.topicTypes.set(topicName, topicType);
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
    this.echoManager.stopEcho(topicName);
    this.panels.delete(topicName);
    this.messageBuffers.delete(topicName);
    this.messageLimits.delete(topicName);
    this.topicTypes.delete(topicName);
    this.lastImageDelivery.delete(topicName);
    this.pausedTopics.delete(topicName);
  }

  private startMonitoring(topicName: string): void {
    this.echoManager.startEcho(topicName, (message: TopicMessage) => {
      if (this.pausedTopics.has(topicName)) {
        return;
      }

      const topicType = this.topicTypes.get(topicName) ?? "";
      if (isImageType(topicType)) {
        const now = Date.now();
        const lastDelivery = this.lastImageDelivery.get(topicName) ?? 0;
        if (now - lastDelivery < this.imagePreviewIntervalMs) {
          return;
        }
        this.lastImageDelivery.set(topicName, now);
      }

      const preparedMessage = prepareTopicMessage(message, topicType);
      const buffer = this.messageBuffers.get(topicName) ?? [];
      buffer.push(preparedMessage);
      const messageLimit = this.messageLimits.get(topicName) ?? this.maxMessagesPerTopic;
      if (buffer.length > messageLimit) {
        buffer.shift();
      }
      this.messageBuffers.set(topicName, buffer);

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
        break;
      case "resume":
        this.pausedTopics.delete(topicName);
        break;
      case "clear":
        this.messageBuffers.set(topicName, []);
        break;
      case "getHistory":
        void this.panels.get(topicName)?.webview.postMessage({
          command: "history",
          messages: this.messageBuffers.get(topicName) ?? []
        });
        break;
    }
  }

  public dispose(): void {
    this.closeAll();
    this.echoManager.dispose();
  }
}
