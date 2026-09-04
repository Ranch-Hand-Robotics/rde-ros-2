// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as fs from "fs";
import * as path from "path";

import { TopicWatchState } from "../../src/ros/topic-tree/topic-watch-state";

describe("ROS 2 Topic Watch State", () => {
  it("starts paused and denies hidden topic queries", () => {
    const state = new TopicWatchState();

    assert.strictEqual(state.isWatcherEnabled(), false);
    assert.strictEqual(state.consumeQueryPermission(), false);
    assert.strictEqual(state.requestManualRefresh(), false);
  });

  it("allows automatic refresh only while visible, enabled, and focused", () => {
    const state = new TopicWatchState();
    state.setWatcherEnabled(true);

    assert.strictEqual(state.shouldAutoRefresh(true), false);

    state.setViewVisible(true);
    assert.strictEqual(state.shouldAutoRefresh(false), false);
    assert.strictEqual(state.shouldAutoRefresh(true), true);
    assert.strictEqual(state.consumeQueryPermission(), true);

    state.setViewVisible(false);
    assert.strictEqual(state.shouldAutoRefresh(true), false);
    assert.strictEqual(state.consumeQueryPermission(), false);
  });

  it("consumes a visible manual refresh exactly once while paused", () => {
    const state = new TopicWatchState();
    state.setViewVisible(true);

    assert.strictEqual(state.requestManualRefresh(), true);
    assert.strictEqual(state.consumeQueryPermission(), true);
    assert.strictEqual(state.consumeQueryPermission(), false);
  });

  it("cancels a pending manual refresh when the view is hidden", () => {
    const state = new TopicWatchState();
    state.setViewVisible(true);
    state.requestManualRefresh();
    state.setViewVisible(false);
    state.setViewVisible(true);

    assert.strictEqual(state.consumeQueryPermission(), false);
  });

  it("contributes mutually exclusive Play and Pause view buttons", () => {
    const packageJson = JSON.parse(fs.readFileSync(
      path.resolve(__dirname, "../../../package.json"),
      "utf8"
    ));
    const commands = packageJson.contributes.commands as Array<{ command: string; icon?: string }>;
    const titleMenu = packageJson.contributes.menus["view/title"] as Array<{
      command: string;
      when: string;
    }>;

    assert.strictEqual(
      commands.find(command => command.command === "ROS2.topicTree.startWatcher")?.icon,
      "$(play)"
    );
    assert.strictEqual(
      commands.find(command => command.command === "ROS2.topicTree.pauseWatcher")?.icon,
      "$(debug-pause)"
    );
    assert.strictEqual(
      titleMenu.find(item => item.command === "ROS2.topicTree.startWatcher")?.when,
      "view == ranchhandrobotics.rde-ros-2.topicTree && !ros2.topicWatcherEnabled"
    );
    assert.strictEqual(
      titleMenu.find(item => item.command === "ROS2.topicTree.pauseWatcher")?.when,
      "view == ranchhandrobotics.rde-ros-2.topicTree && ros2.topicWatcherEnabled"
    );
  });
});
