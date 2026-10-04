// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as fs from "fs";
import * as path from "path";

describe("ROS view registrations", () => {
  const packagePath = path.resolve(__dirname, "../../../package.json");
  const packageText = fs.readFileSync(packagePath, "utf8");
  const manifest = JSON.parse(packageText);
  const contributes = manifest.contributes;
  const viewIds = Object.values(contributes.views).flatMap((views: unknown) =>
    (views as Array<{ id: string }>).map(view => view.id)
  );

  it("contributes every view registered by the extension", () => {
    for (const id of [
      "ranchhandrobotics.rde-ros-2.launchTree",
      "ranchhandrobotics.rde-ros-2.topicTree",
      "ros2Distributions",
    ]) {
      assert.ok(viewIds.includes(id), `Missing contributes.views registration for ${id}`);
    }
  });

  it("does not keep stale view activation IDs or duplicate views sections", () => {
    assert.ok(!manifest.activationEvents.includes("onView:ros2LaunchTree"));
    assert.strictEqual((packageText.match(/"views"\s*:/g) ?? []).length, 1);
  });
});
