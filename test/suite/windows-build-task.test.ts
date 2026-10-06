import * as assert from "assert";
import { promises as fs } from "fs";
import * as os from "os";
import * as path from "path";
import * as vscode from "vscode";
import * as extension from "../../src/extension";
import { COLCON_TASK_TYPE } from "../../src/build-tool/colcon";
import { make, resolve } from "../../src/build-tool/ros-shell";

// Use VS Code's actual task service and a harmless PowerShell fixture instead of
// colcon/MSVC. Compiler/SDK files are readiness fixtures, not real toolchains.
describe("Windows build task execution", function() {
  this.timeout(30000);

  it("discovers both colcon builds through the actual VS Code task service without a ROS environment", async function() {
    if (process.platform !== "win32") { this.skip(); }
    assert.ok(vscode.workspace.workspaceFolders?.length, "Discovery needs an open folder");
    // Let VS Code activate the packaged extension and discover its own provider.
    // Registering a test provider here would mask activation failures.
    const discovered = await vscode.tasks.fetchTasks({ type: COLCON_TASK_TYPE });
    const builds = discovered.filter(task => task.group?.id === "build");
    assert.deepStrictEqual(builds.map(task => task.name).sort(), ["Colcon Build Debug", "Colcon Build Release"]);
  });

  for (const resolved of [false, true]) {
    it(`runs ${resolved ? "resolved" : "generated"} tasks with substituted variables and compiler environment`, async function() {
      if (process.platform !== "win32") { this.skip(); }
      const folder = vscode.workspace.workspaceFolders?.[0];
      assert.ok(folder, "Integration tests need a workspace");
      const root = await fs.mkdtemp(path.join(os.tmpdir(), "RDE build user's & fixture!-"));
      const hostEnv = { ...process.env };
      const previousEnv = extension.env;
      const previousPrepare = extension.prepareRosBuildEnvironment;
      let execution: vscode.TaskExecution | undefined;
      let listener: vscode.Disposable | undefined;
      let timer: NodeJS.Timeout | undefined;
      try {
        const bin = path.join(root, "tools");
        const sdk = path.join(root, "SDK");
        for (const file of ["tools/cl.exe", "tools/link.exe", "tools/rc.exe",
          "SDK/Include/10.0/um/Windows.h", "SDK/Include/10.0/ucrt/stdio.h",
          "SDK/Lib/10.0/um/x64/kernel32.lib", "SDK/Lib/10.0/ucrt/x64/ucrt.lib"]) {
          const filename = path.join(root, file);
          await fs.mkdir(path.dirname(filename), { recursive: true });
          await fs.writeFile(filename, "");
        }
        const compilerEnv = { Path: bin, VisualStudioVersion: "17.0", VSCMD_ARG_TGT_ARCH: "x64",
          WindowsSdkDir: sdk, WindowsSDKVersion: "10.0\\", INCLUDE: sdk, LIB: sdk };
        // ROS sourcing/CLI failures are tested separately; this test validates
        // the real VS Code task callback and subprocess environment transport.
        let prepared = false;
        let preparationError: unknown;
        Object.assign(extension, { prepareRosBuildEnvironment: async (
          activated: NodeJS.ProcessEnv,
          options: Parameters<typeof extension.prepareRosBuildEnvironment>[1],
        ) => {
          try {
            assert.strictEqual(activated.VisualStudioVersion, "17.0");
            assert.strictEqual(activated.INCLUDE, sdk);
            assert.strictEqual(activated.LIB, sdk);
            const paths = Object.entries(activated).filter(([key]) => key.toLowerCase() === "path");
            assert.strictEqual(paths.length, 1);
            assert.strictEqual(paths[0][1], bin);
            const expectedFolder = path.normalize(folder!.uri.fsPath).toLowerCase();
            assert.strictEqual(path.normalize(options?.cwd ?? "").toLowerCase(), expectedFolder);
            assert.strictEqual(path.normalize(options?.args?.[options.args.length - 1] ?? "").toLowerCase(), expectedFolder);
            assert.strictEqual(typeof options?.onOutput, "function");
            prepared = true;
            return activated;
          } catch (error) {
            preparationError = error;
            throw error;
          }
        } });
        const script = path.join(root, "probe.ps1");
        await fs.writeFile(script, [
          "param([string]$verb, [string]$folder)",
          "$result = @{ cwd = (Get-Location).Path; folder = $folder; verb = $verb; vs = $env:VisualStudioVersion; marker = $env:RDE_BUILD_FIXTURE } | ConvertTo-Json -Compress",
          "[System.IO.File]::WriteAllText((Join-Path $PSScriptRoot 'result.json'), $result)",
          "exit 0",
        ].join("\r\n"));
        const command = path.join(process.env.SystemRoot!, "System32", "WindowsPowerShell", "v1.0", "powershell.exe");
        const definition = { type: "colcon", command,
          args: ["-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", script, "build", "${workspaceFolder}"],
          options: { env: { ...compilerEnv, RDE_BUILD_FIXTURE: "${workspaceFolderBasename}" } },
        };
        const task = resolved
          ? resolve(new vscode.Task(definition, folder!, "RDE resolved build fixture", "colcon"))
          : make("RDE generated build fixture", definition);
        if (resolved) { assert.strictEqual(task.definition, definition); }
        assert.ok(task.execution instanceof vscode.CustomExecution);
        task.problemMatchers = [];
        task.presentationOptions = { reveal: vscode.TaskRevealKind.Never, close: true };
        const completed = new Promise<void>((yes, no) => {
          timer = setTimeout(() => no(new Error("Build fixture did not finish")), 20000);
          listener = vscode.tasks.onDidEndTask(event => {
            if (event.execution.task.name === task.name) { yes(); }
          });
        });
        execution = await vscode.tasks.executeTask(task);
        await completed;
        if (preparationError) { throw preparationError; }
        assert.ok(prepared, "ROS preflight must run before launching the task process");
        const result = JSON.parse(await fs.readFile(path.join(root, "result.json"), "utf8"));
        assert.strictEqual(path.normalize(result.cwd).toLowerCase(), path.normalize(folder!.uri.fsPath).toLowerCase());
        assert.strictEqual(path.normalize(result.folder).toLowerCase(), path.normalize(folder!.uri.fsPath).toLowerCase());
        assert.strictEqual(result.marker, path.basename(folder!.uri.fsPath));
        assert.strictEqual(result.verb, "build");
        assert.strictEqual(result.vs, "17.0");
        assert.strictEqual(extension.env, previousEnv, "Compiler fixtures must come from task overrides");
        assert.deepStrictEqual({ ...process.env }, hostEnv, "The real host environment must remain unchanged");
      } finally {
        if (timer) { clearTimeout(timer); }
        listener?.dispose();
        execution?.terminate();
        Object.assign(extension, { prepareRosBuildEnvironment: previousPrepare });
        await fs.rm(root, { recursive: true, force: true });
      }
    });
  }
});