// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";

import { parseTopicEchoMessage, parseTopicFrequency } from "../../src/ros/ros2/topic-monitor";
import { createTopicMonitorHtml, prepareTopicMessage } from "../../src/ros/ros2/topic-webview";

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
    assert.ok(compressed.includes("const maxMessages = 1"));
    assert.ok(raw.includes('data-compressed-image="false"'));
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

  it("omits raw image bytes from the preview", () => {
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
      data: "<6 image bytes omitted from preview>"
    });
  });
});
