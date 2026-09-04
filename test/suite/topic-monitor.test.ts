// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";

import {
  createTopicEchoArguments,
  parseTopicEchoMessage,
  parseTopicFrequency
} from "../../src/ros/ros2/topic-monitor";
import {
  createTopicMonitorHtml,
  isConsoleTopic,
  prepareTopicMessage,
  TopicMessageRingBuffer
} from "../../src/ros/ros2/topic-webview";

describe("ROS 2 Topic Monitor", () => {
  it("parses nested topic messages without flattening fields", () => {
    const message = parseTopicEchoMessage(`
header:
  stamp:
    sec: 42
    nanosec: 100
  frame_id: map
values:
  - 1
  - 2
enabled: true
`) as {
      header: { stamp: { sec: number; nanosec: number }; frame_id: string };
      values: number[];
      enabled: boolean;
    };

    assert.deepStrictEqual(message.header.stamp, { sec: 42, nanosec: 100 });
    assert.deepStrictEqual(message.values, [1, 2]);
    assert.strictEqual(message.enabled, true);
  });

  it("parses topic frequency summaries", () => {
    const metrics = parseTopicFrequency("average rate: 29.970\nmin: 0.032s max: 0.035s");

    assert.strictEqual(metrics?.averageRate, 29.97);
    assert.strictEqual(metrics?.minRate, 0.032);
    assert.strictEqual(metrics?.maxRate, 0.035);
  });

  it("requests full-length topic echo output for image payloads", () => {
    assert.deepStrictEqual(createTopicEchoArguments("/image"), [
      "topic",
      "echo",
      "/image",
      "--full-length"
    ]);
  });

  it("creates a CSP-protected webview and escapes topic metadata", () => {
    const html = createTopicMonitorHtml(
      "vscode-webview:",
      "</title><script>alert('topic')</script>",
      'sensor_msgs/msg/String"><img src=x onerror=alert(1)>',
      100,
      "test-nonce"
    );

    assert.ok(html.includes("default-src 'none'"));
    assert.ok(html.includes("script-src 'nonce-test-nonce'"));
    assert.ok(html.includes('nonce="test-nonce"'));
    assert.ok(!html.includes("</title><script>alert('topic')</script>"));
    assert.ok(!html.includes('<img src=x onerror=alert(1)>'));
    assert.ok(!html.includes("onclick="));
  });

  it("enables image rendering only for compressed image topics", () => {
    const compressed = createTopicMonitorHtml(
      "vscode-webview:",
      "/camera/compressed",
      "sensor_msgs/msg/CompressedImage",
      1,
      "test-nonce"
    );
    const raw = createTopicMonitorHtml(
      "vscode-webview:",
      "/camera/raw",
      "sensor_msgs/msg/Image",
      1,
      "test-nonce"
    );

    assert.ok(compressed.includes('data-compressed-image="true"'));
    assert.ok(compressed.includes('data-image-topic="true"'));
    assert.ok(compressed.includes("let maxMessages = 1"));
    assert.ok(raw.includes('data-compressed-image="false"'));
    assert.ok(raw.includes("function updateLatestImage(message)"));
    assert.ok(raw.includes("messageContainer.replaceChildren(latestImage.element);"));
    assert.ok(!raw.includes("fragment.appendChild(createLatestImage("));
    assert.ok(raw.includes('className = "image-meta-item"'));
    assert.ok(raw.includes(".image-wrap [hidden]"));
    assert.ok(raw.includes('id="streamRate"'));
    assert.ok(raw.includes('id="bufferLength"'));
    assert.ok(raw.includes("linear-gradient(90deg, color-mix(in srgb, var(--accent) 18%, transparent) 1px, transparent 1px)"));
    assert.ok(raw.includes('message.command === "setPaused"'));
    assert.ok(raw.includes("function setPaused(paused)"));
    assert.ok(raw.includes("display: flex;\n      align-items: center;\n      gap: 12px;\n      min-height: 64px;"));
  });

  it("unwraps ROS string messages to their raw payload", () => {
    const message = prepareTopicMessage(
      { timestamp: 1, data: { data: "plain console text" } },
      "std_msgs/msg/String"
    );

    assert.strictEqual(message.data, "plain console text");
  });

  it("renders string messages as ANSI-aware console output", () => {
    const html = createTopicMonitorHtml(
      "vscode-webview:",
      "/chatter",
      "std_msgs/msg/String",
      100,
      "test-nonce"
    );

    assert.ok(html.includes("consoleMessage.className = \"console-message\";"));
    assert.ok(html.includes("function appendAnsiText(element, text)"));
    assert.ok(html.includes('const sgr = /\\x1b\\[([0-9;]*)m/g;'));
    assert.ok(html.includes("ansi-fg-31"));
    assert.ok(html.includes('data-console-stream="true"'));
  });

  it("retains the latest messages in chronological order when its buffer wraps", () => {
    const buffer = new TopicMessageRingBuffer(2);
    buffer.push({ timestamp: 1, data: "first" });
    buffer.push({ timestamp: 2, data: "second" });
    buffer.push({ timestamp: 3, data: "third" });

    assert.deepStrictEqual(buffer.values(), [
      { timestamp: 2, data: "second" },
      { timestamp: 3, data: "third" }
    ]);
  });

  it("formats ROS log topics as console output", () => {
    const message = prepareTopicMessage(
      { timestamp: 1, data: { level: 30, name: "camera", msg: "Frame dropped" } },
      "rcl_interfaces/msg/Log"
    );

    assert.strictEqual(message.data, "[WARN] camera: Frame dropped");
    assert.strictEqual(isConsoleTopic("rcl_interfaces/msg/Log"), true);
  });

  it("resizes a message buffer while retaining its newest messages", () => {
    const buffer = new TopicMessageRingBuffer(3);
    buffer.push({ timestamp: 1, data: "first" });
    buffer.push({ timestamp: 2, data: "second" });
    buffer.push({ timestamp: 3, data: "third" });
    buffer.resize(2);

    assert.deepStrictEqual(buffer.values(), [
      { timestamp: 2, data: "second" },
      { timestamp: 3, data: "third" }
    ]);
  });

  it("encodes compressed images before sending them to the webview", () => {
    const message = prepareTopicMessage(
      {
        timestamp: 1,
        data: {
          format: "jpeg",
          data: [0, 1, 2, 255]
        }
      },
      "sensor_msgs/msg/CompressedImage"
    );

    assert.deepStrictEqual(message.data, {
      format: "jpeg",
      data: Buffer.from([0, 1, 2, 255]).toString("base64")
    });
  });

  it("encodes raw image bytes for the webview preview", () => {
    const message = prepareTopicMessage(
      {
        timestamp: 1,
        data: {
          width: 2,
          height: 1,
          data: [10, 20, 30, 40, 50, 60]
        }
      },
      "sensor_msgs/msg/Image"
    );

    assert.deepStrictEqual(message.data, {
      width: 2,
      height: 1,
      data: Buffer.from([10, 20, 30, 40, 50, 60]).toString("base64")
    });
  });

  it("preserves already encoded raw image data", () => {
    const encoded = Buffer.from([10, 20, 30]).toString("base64");
    const message = prepareTopicMessage(
      { timestamp: 1, data: { width: 1, height: 1, encoding: "bgr8", data: encoded } },
      "sensor_msgs/msg/Image"
    );

    assert.strictEqual((message.data as { data: string }).data, encoded);
  });
});
