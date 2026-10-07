// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as path from "path";
import * as vscode from "vscode";
import { prepareTopicMessage, TopicWebviewManager } from "../../src/ros/ros2/topic-webview";
import { TopicEchoManager } from "../../src/ros/ros2/topic-monitor";
import { TopicMessage, isPointCloudType } from "../../src/ros/ros2/topic-types";

describe("PointCloud2 topic panel", () => {
  let manager: TopicWebviewManager;
  let restore: (() => void)[];
  let received: (message: { command: string; maxMessages?: number; rateHz?: number }) => void;
  let emit: (message: TopicMessage) => void;
  let deliveries: any[];
  let options: vscode.WebviewOptions;
  let panel: vscode.WebviewPanel;
  let stops: number;
  let starts: number;
  let now: number;
  let sourceRates: number[];
  let startRates: (number | undefined)[];

  function replace(object: object, key: string, value: unknown): void {
    const descriptor = Object.getOwnPropertyDescriptor(object, key)!;
    Object.defineProperty(object, key, { configurable: true, value });
    restore.push(() => Object.defineProperty(object, key, descriptor));
  }

  beforeEach(() => {
    restore = []; deliveries = []; stops = 0; starts = 0; now = 1000;
    sourceRates = []; startRates = [];
    panel = {
      webview: {
        cspSource: "https://webview.local", html: "",
        asWebviewUri: (uri: vscode.Uri) => uri,
        postMessage: async (message: unknown) => { deliveries.push(message); return true; },
        onDidReceiveMessage: (callback: typeof received) => { received = callback; return { dispose() {} }; }
      },
      onDidDispose: () => ({ dispose() {} }), dispose() {}, reveal() {}
    } as unknown as vscode.WebviewPanel;
    replace(vscode.window, "createWebviewPanel", (_view: string, _title: string, _column: unknown, opts: vscode.WebviewOptions) => {
      options = opts; return panel;
    });
    replace(TopicEchoManager.prototype, "startEcho", (_name: string, callback: typeof emit, type: string, rate?: number) => {
      assert.strictEqual(type, "sensor_msgs/msg/PointCloud2");
      emit = callback; starts++; startRates.push(rate);
    });
    replace(TopicEchoManager.prototype, "setPointCloudRefreshRate", (_name: string, rate: number) => { sourceRates.push(rate); });
    replace(TopicEchoManager.prototype, "stopEcho", () => { stops++; });
    replace(Date, "now", () => now);
    manager = new TopicWebviewManager({ extensionUri: vscode.Uri.file(path.resolve(__dirname, "../../..")), subscriptions: [] } as unknown as vscode.ExtensionContext);
    manager.openTopicMonitor("/points", "sensor_msgs/msg/PointCloud2");
  });

  afterEach(() => { manager.dispose(); restore.reverse().forEach(undo => undo()); });

  it("recognizes only PointCloud2 and limits local asset roots", () => {
    assert.strictEqual(isPointCloudType("sensor_msgs/msg/PointCloud2"), true);
    assert.strictEqual(isPointCloudType("sensor_msgs/msg/PointCloud"), false);
    assert.strictEqual(isPointCloudType("custom/PointCloud2"), false);
    assert.strictEqual(options.localResourceRoots?.length, 2);
    assert.ok(options.localResourceRoots[0].path.endsWith("/dist"));
    assert.ok(options.localResourceRoots[1].path.endsWith("/assets/ros/topic-monitor"));
    assert.ok(panel.webview.html.includes("let maxMessages = 1"));
    assert.ok(panel.webview.html.includes('min="0.2" max="5" value="1" step="0.2"'));
    assert.deepStrictEqual(startRates, [1]);
  });

  it("delivers source-throttled frames without double throttling and retains only the latest cloud", () => {
    emit({ timestamp: 1, data: { data: [1] } });
    now += 50;
    emit({ timestamp: 2, data: { data: [2] } });
    now += 150;
    emit({ timestamp: 3, data: { data: [3] } });
    assert.deepStrictEqual(deliveries.map(message => message.message.timestamp), [1, 2, 3]);
    received({ command: "setBufferLength", maxMessages: 500 });
    now += 200;
    emit({ timestamp: 4, data: { data: [4] } });
    received({ command: "getHistory" });
    assert.deepStrictEqual(deliveries[4].messages, [{ timestamp: 4, data: { data: [4] } }]);
  });

  it("supports refresh rate, pause, resume, clear and closing", () => {
    received({ command: "setRefreshRate", rateHz: 10 });
    assert.deepStrictEqual(sourceRates, [5]);
    emit({ timestamp: 1, data: {} });
    now += 100;
    emit({ timestamp: 2, data: {} });
    assert.strictEqual(deliveries.length, 2);
    received({ command: "pause" });
    now += 100;
    emit({ timestamp: 3, data: {} });
    assert.strictEqual(deliveries.length, 2);
    assert.strictEqual(stops, 1);
    received({ command: "resume" });
    assert.strictEqual(starts, 2);
    assert.deepStrictEqual(startRates, [1, 5]);
    emit({ timestamp: 4, data: {} });
    received({ command: "clear" });
    received({ command: "getHistory" });
    assert.deepStrictEqual(deliveries[3].messages, []);
    manager.closeTopicMonitor("/points");
    assert.strictEqual(stops, 2);
  });

  it("keeps fractional rates across global pause/resume and rejects invalid updates", () => {
    manager.setMonitoringEnabled(true);
    manager.setMonitoringEnabled(false);
    const count = starts;
    for (const rateHz of [NaN, Infinity]) { received({ command: "setRefreshRate", rateHz }); }
    assert.deepStrictEqual(sourceRates, []);
    received({ command: "setRefreshRate", rateHz: 0 });
    assert.deepStrictEqual(sourceRates, [0.2]);
    assert.strictEqual(starts, count, "Changing a paused rate must not start the subscriber");
    manager.setMonitoringEnabled(true);
    assert.strictEqual(startRates[startRates.length - 1], 0.2);
  });
});

describe("PointCloud2 binary webview transport", () => {
  it("receives typed bytes through VS Code postMessage without base64 or buffer pooling", async () => {
    const panel = vscode.window.createWebviewPanel("cloudBinaryTest", "Binary transport test",
      { viewColumn: vscode.ViewColumn.Beside, preserveFocus: true }, { enableScripts: true });
    const bytes = Buffer.from([99, 0, 127, 255, 88]).subarray(1, 4);
    const message = prepareTopicMessage({ timestamp: 42, data: { data: bytes } }, "sensor_msgs/msg/PointCloud2");
    let timeout: ReturnType<typeof setTimeout>;
    try {
      await new Promise<void>((resolve, reject) => {
        timeout = setTimeout(() => reject(new Error("Binary webview round trip timed out")), 10000);
        panel.webview.onDidReceiveMessage(result => {
          if (result.ready) {
            panel.webview.postMessage(message).then(undefined, reject);
            return;
          }
          try {
            assert.deepStrictEqual(result, {
              type: "Uint8Array", bytes: [0, 127, 255], bufferLength: 3, timestamp: 42
            });
            resolve();
          } catch (error) { reject(error); }
        });
        panel.webview.html = `<!DOCTYPE html><html><body><script>
          const vscode = acquireVsCodeApi();
          window.addEventListener("message", ({ data: message }) => {
            const bytes = message.data.data;
            vscode.postMessage({ type: bytes.constructor.name, bytes: Array.from(bytes),
              bufferLength: bytes.buffer?.byteLength, timestamp: message.timestamp });
          });
          vscode.postMessage({ ready: true });
        </script></body></html>`;
      });
    } finally {
      clearTimeout(timeout);
      panel.dispose();
    }
  });
});