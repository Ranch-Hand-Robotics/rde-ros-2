// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import * as path from "path";
import * as vscode from "vscode";
import { cachePixiInstallRoot, PIXI_INSTALL_LOCATIONS_SETTING, resolvePixiInstallRoot } from "../../src/ros/installer/pixi-location";

describe("per-machine Pixi install locations", () => {
  const home = "C:\\Users\\tester";

  it("uses this machine's cached location", () => {
    assert.strictEqual(resolvePixiInstallRoot({
      machineId: "machine-a",
      locations: { "machine-a": "D:\\ros\\pixi", "machine-b": "E:\\ros\\pixi" },
      legacyRoot: "C:\\old-pixi",
      platform: "win32",
      home,
    }), "D:\\ros\\pixi");
  });

  it("does not reuse another machine's synced path or the synced legacy root", () => {
    assert.strictEqual(resolvePixiInstallRoot({
      machineId: "machine-b",
      locations: { "machine-a": "D:\\ros\\pixi" },
      legacyRoot: "D:\\ros\\pixi",
      platform: "win32",
      home,
    }), "c:\\pixi_ws");
  });

  it("uses the legacy root only before any machine-specific path has been cached", () => {
    assert.strictEqual(resolvePixiInstallRoot({
      machineId: "machine-a",
      locations: {},
      legacyRoot: "D:\\legacy\\pixi",
      platform: "win32",
      home,
    }), "D:\\legacy\\pixi");
  });

  it("supports an explicit local override and platform-specific defaults", () => {
    assert.strictEqual(resolvePixiInstallRoot({
      machineId: "machine-a", locations: { "machine-a": "C:\\cached" },
      machineOverride: "E:\\local", platform: "win32", home,
    }), "E:\\local");
    assert.strictEqual(resolvePixiInstallRoot({
      machineId: "machine-a", locations: {}, platform: "darwin", home: "/Users/tester",
    }), "/Users/tester/pixi_ws");
    assert.strictEqual(resolvePixiInstallRoot({
      machineId: "machine-a", locations: {}, platform: "linux", home,
    }), "");
  });

  it("persists a chosen root in global settings under only the current machine ID", async () => {
    let persisted: Record<string, string> | undefined;
    let target: vscode.ConfigurationTarget | undefined;
    const config = {
      inspect: (name: string) => name === PIXI_INSTALL_LOCATIONS_SETTING
        ? { globalValue: { "machine-a": "D:\\existing", "machine-b": "E:\\existing" } }
        : undefined,
      update: async (name: string, value: Record<string, string>, scope: vscode.ConfigurationTarget) => {
        assert.strictEqual(name, PIXI_INSTALL_LOCATIONS_SETTING);
        persisted = value;
        target = scope;
      },
    } as unknown as vscode.WorkspaceConfiguration;

    const root = path.resolve("pixi-location-fixture");
    await cachePixiInstallRoot(root, config, "machine-a");
    assert.deepStrictEqual(persisted, { "machine-a": root, "machine-b": "E:\\existing" });
    assert.strictEqual(target, vscode.ConfigurationTarget.Global);
  });

  it("rejects relative roots on every target platform and Windows roots on POSIX", () => {
    for (const platform of ["win32", "darwin", "linux"] as const) {
      const options = { machineId: "machine-a", locations: {}, platform, home: "/Users/tester" };
      const fallback = resolvePixiInstallRoot(options);
      assert.strictEqual(resolvePixiInstallRoot({
        ...options, machineOverride: "relative/pixi", legacyRoot: "relative/legacy",
        locations: { "machine-a": "relative/cache" },
      }), fallback);
      if (platform !== "win32") {
        assert.strictEqual(resolvePixiInstallRoot({
          ...options, machineOverride: "D:\\pixi", legacyRoot: "D:\\legacy",
          locations: { "machine-a": "D:\\cache" },
        }), fallback);
      }
    }
  });

  it("does not persist a relative install root", async () => {
    const config = {
      update: async () => assert.fail("Invalid roots must not be persisted"),
    } as unknown as vscode.WorkspaceConfiguration;
    await assert.rejects(cachePixiInstallRoot("relative/pixi", config, "machine-a"), /absolute path/);
  });
});
