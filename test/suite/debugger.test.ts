import * as assert from "assert";
import * as os from "os";
import * as vscode from "vscode";
import * as vscodeUtils from "../../src/vscode-utils";
import { AttachResolver } from "../../src/debugger/configuration/resolvers/attach";
import { LaunchResolver } from "../../src/debugger/configuration/resolvers/ros2/launch";
import { LocalProcessPicker } from "../../src/debugger/process-picker/process-picker";

describe("Native debugger configuration", () => {
  const originalPlatform = os.platform;
  const originalCppToolsInstalled = vscodeUtils.isCppToolsExtensionInstalled;
  const originalLldbInstalled = vscodeUtils.isLldbExtensionInstalled;
  const originalStartDebugging = vscode.debug.startDebugging;
  const originalPick = LocalProcessPicker.prototype.pick;
  let launchedConfig: vscode.DebugConfiguration;

  const request = {
    nodeName: "talker",
    executable: "/workspace/install/talker",
    arguments: ["--ros-args", "-r", "__node:=talker"],
    cwd: ".",
    env: { ROS_DISTRO: "jazzy", PATH: "/workspace/bin" },
    sourceFileMap: { "/build/src": "/workspace/src" }
  };

  beforeEach(() => {
    (os as any).platform = () => "darwin";
    (vscodeUtils as any).isCppToolsExtensionInstalled = () => true;
    (vscodeUtils as any).isLldbExtensionInstalled = () => true;
    launchedConfig = undefined;
    vscode.debug.startDebugging = async (_folder, config) => {
      launchedConfig = config as vscode.DebugConfiguration;
      return true;
    };
  });

  afterEach(() => {
    (os as any).platform = originalPlatform;
    (vscodeUtils as any).isCppToolsExtensionInstalled = originalCppToolsInstalled;
    (vscodeUtils as any).isLldbExtensionInstalled = originalLldbInstalled;
    vscode.debug.startDebugging = originalStartDebugging;
    LocalProcessPicker.prototype.pick = originalPick;
  });

  it("prefers CodeLLDB on macOS and preserves launch options using its schema", () => {
    const config = new LaunchResolver()["createCppLaunchConfig"](request, true);
    assert.deepStrictEqual(config, {
      name: request.nodeName,
      type: "lldb",
      request: "launch",
      program: request.executable,
      args: request.arguments,
      cwd: request.cwd,
      env: request.env,
      stopOnEntry: true,
      sourceMap: request.sourceFileMap
    });
    assert.strictEqual(new LaunchResolver()["createCppLaunchConfig"](request, false)["stopOnEntry"], false);
  });

  for (const platform of ["darwin", "linux", "win32"]) {
    it(`uses CodeLLDB as the only installed adapter on ${platform}`, async () => {
      (os as any).platform = () => platform;
      (vscodeUtils as any).isCppToolsExtensionInstalled = () => false;
      assert.strictEqual(new LaunchResolver()["createCppLaunchConfig"](request, false).type, "lldb");
      const resolver = new AttachResolver();
      resolver["resolveCommandLineIfNeeded"] = async () => assert.fail("CodeLLDB does not need an executable lookup");
      await resolver.resolveDebugConfigurationWithSubstitutedVariables(undefined, {
        name: "attach", type: "ros2", request: "attach", runtime: "C++", processId: "12345"
      });
      assert.deepStrictEqual(launchedConfig, {
        name: "C++: 12345", type: "lldb", request: "attach", pid: "12345"
      });
    });
  }

  it("attaches with the picked PID on macOS without resolving an executable", async () => {
    LocalProcessPicker.prototype.pick = async () => ({ pid: "12345", name: "talker", commandLine: "/workspace/talker" });
    const resolver = new AttachResolver();
    resolver["resolveCommandLineIfNeeded"] = async () => assert.fail("CodeLLDB does not need an executable lookup");
    const result = await resolver.resolveDebugConfigurationWithSubstitutedVariables(undefined, {
      name: "attach", type: "ros2", request: "attach", runtime: "C++", processId: "${action:pick}"
    });
    assert.strictEqual(result, null);
    assert.deepStrictEqual(launchedConfig, {
      name: "C++: 12345", type: "lldb", request: "attach", pid: "12345"
    });
  });

  it("propagates a failed attach startup to the resolver caller", async () => {
    vscode.debug.startDebugging = async () => false;
    await assert.rejects(new AttachResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined, {
      name: "attach", type: "ros2", request: "attach", runtime: "C++", processId: 12345
    }), /Failed to start debug session/);
  });

  for (const [platform, expectedType] of [["linux", "cppdbg"], ["win32", "cppvsdbg"]]) {
    it(`keeps the C/C++ adapter preference on ${platform}`, async () => {
      (os as any).platform = () => platform;
      const config = new LaunchResolver()["createCppLaunchConfig"](request, true);
      assert.strictEqual(config.type, expectedType);
      assert.strictEqual(config["stopAtEntry"], true);
      assert.deepStrictEqual(config["sourceFileMap"], request.sourceFileMap);
      await new AttachResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined, {
        name: "attach", type: "ros2", request: "attach", runtime: "C++", processId: 12345,
        commandLine: request.executable
      });
      assert.strictEqual(launchedConfig.type, expectedType);
      assert.strictEqual(launchedConfig.processId, 12345);
    });
  }

  it("falls back to C/C++ on macOS when CodeLLDB is absent", async () => {
    (vscodeUtils as any).isLldbExtensionInstalled = () => false;
    assert.strictEqual(new LaunchResolver()["createCppLaunchConfig"](request, false).type, "cppdbg");
    await new AttachResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined, {
      name: "attach", type: "ros2", request: "attach", runtime: "C++", processId: 12345,
      commandLine: request.executable
    });
    assert.strictEqual(launchedConfig.type, "cppdbg");
    assert.strictEqual(launchedConfig.program, request.executable);
  });

  it("resolves the C/C++ executable with ps on macOS", async function () {
    if (originalPlatform() !== "darwin") {
      this.skip();
    }
    (vscodeUtils as any).isLldbExtensionInstalled = () => false;
    await new AttachResolver().resolveDebugConfigurationWithSubstitutedVariables(undefined, {
      name: "attach", type: "ros2", request: "attach", runtime: "C++", processId: process.pid
    });
    assert.strictEqual(launchedConfig.type, "cppdbg");
    assert.ok(launchedConfig.program.startsWith("/"), launchedConfig.program);
  });
});