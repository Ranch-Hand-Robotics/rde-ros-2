// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as vscode from "vscode";

import { TopicTreeItem, TopicTreeItemType } from "../../src/ros/topic-tree/topic-tree-item";

describe("ROS 2 Topic Tree", () => {
  it("creates a checked topic item with type metadata", () => {
    const item = TopicTreeItem.createTopicItem(
      { name: "/camera/image", type: "sensor_msgs/msg/Image" },
      true
    );

    assert.strictEqual(item.type, TopicTreeItemType.Topic);
    assert.strictEqual(item.label, "/camera/image");
    assert.strictEqual(item.description, "sensor_msgs/msg/Image");
    assert.strictEqual(item.checkboxState, vscode.TreeItemCheckboxState.Checked);
    assert.strictEqual(item.contextValue, "topicSubscribed");
  });

  it("includes QoS details in the topic tooltip", () => {
    const item = TopicTreeItem.createTopicItem({
      name: "/status",
      type: "std_msgs/msg/String"
    });

    item.setMetrics(10, 2, 3, {
      reliability: "RELIABLE",
      durability: "VOLATILE",
      deadline: "",
      lifespan: "",
      liveliness: "AUTOMATIC",
      livelinessLeaseDuration: ""
    });

    const tooltip = item.tooltip as vscode.MarkdownString;
    assert.ok(tooltip.value.includes("Frequency: 10.00 Hz"));
    assert.ok(tooltip.value.includes("Publishers: 2"));
    assert.ok(tooltip.value.includes("Reliability: RELIABLE"));
  });
});
