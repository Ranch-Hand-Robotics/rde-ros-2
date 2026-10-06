import * as assert from "assert";
import { constants, promises as fs } from "fs";
import * as os from "os";
import * as path from "path";
import * as vscode from "vscode";
import * as extension from "../../src/extension";
import { make, resolve, RosShellTaskProvider, RosTaskDefinition } from "../../src/build-tool/ros-shell";

async function within<T>(promise: PromiseLike<T>, message: string): Promise<T> {
  let timer: NodeJS.Timeout | undefined;
  try {
    return await Promise.race([
      promise,
      new Promise<never>((_, reject) => {
        timer = setTimeout(() => reject(new Error(message)), 10000);
      }),
    ]);
  } finally {
    if (timer) { clearTimeout(timer); }
  }
}

// Exercise VS Code's substitution and CustomExecution lifecycle, not a mock PTY.
// No ROS installation, Electron-as-Node subprocess, or host env changes needed.
describe("Deferred POSIX ROS task execution", function() {
  this.timeout(70000);

  for (const resolved of [false, true]) {
    it(`runs ${resolved ? "resolved" : "generated"} tasks only after fresh activation`, async function() {
      if (process.platform !== "linux" && process.platform !== "darwin") { this.skip(); }
      await fs.access("/bin/sh", constants.X_OK);
      const folder = vscode.workspace.workspaceFolders?.[0];
      assert.ok(folder, "Integration tests need an actual workspace folder");
      const root = await fs.mkdtemp(path.join(os.tmpdir(), "RDE task user's & fixture-"));
      const output = path.join(root, "result.txt");
      const hostEnv = { ...process.env };
      const previousResolvedEnv = extension.resolvedEnv;
      let reads = 0;
      let release: (env: NodeJS.ProcessEnv | undefined) => void = () => {};
      let opened: () => void = () => {};
      let gate = new Promise<NodeJS.ProcessEnv | undefined>(yes => { release = yes; });
      try {
        Object.assign(extension, { resolvedEnv: () => { reads++; opened(); return gate; } });
        const discovered = await within(Promise.resolve(new RosShellTaskProvider().provideTasks()),
          "Discovery waited for activation instead of returning tasks");
        assert.ok(discovered?.length, "Discovery should work without activation");
        assert.strictEqual(reads, 0, "Discovery must not request the ROS environment");

        // Ensure both '*' and '?' would expand if literal argument quoting broke.
        await fs.writeFile(path.join(root, "x"), "");
        const options = {
          cwd: "${workspaceFolder}/" + path.relative(folder!.uri.fsPath, root),
          env: { RDE_TASK_MARKER: "${workspaceFolderBasename}", RDE_REMOVED: null },
          shell: { executable: "/bin/sh", args: ["-c"] },
        };
        const literals = ["*", "?", "${workspaceFolder}", "a'b", "$HOME", ""];
        const definition: RosTaskDefinition = {
          type: resolved ? "colcon" : "ROS2",
          command: "/bin/sh",
          args: ["-c", [
            "output=$1; shift",
            'printf \'%s\\n\' "$PWD" "$RDE_TASK_MARKER" "$RDE_ACTIVATED" "$RDE_REMOVED" "$#" "$@" > "$output"',
          ].join("\n"), "rde-task-fixture", output, ...literals],
          // Also verify legacy options survive VS Code stripping reserved fields.
          ...(resolved ? { options } : { taskOptions: options }),
        };
        const name = `RDE deferred ${resolved ? "resolved" : "generated"} ${path.basename(root)}`;
        const original = resolved ? new vscode.Task(definition, folder!, name, "colcon") : undefined;
        const task = original ? resolve(original) : make(name, definition, undefined, folder!);
        if (original) { assert.strictEqual(task, original); }
        assert.strictEqual(task.definition, definition);
        assert.strictEqual(task.scope, folder);
        assert.ok(task.execution instanceof vscode.CustomExecution);
        assert.strictEqual(reads, 0, "Creating/resolving tasks must not read the environment");
        task.problemMatchers = [];
        task.presentationOptions = { reveal: vscode.TaskRevealKind.Never, close: true };

        for (let run = 1; run <= 2; run++) {
          gate = new Promise<NodeJS.ProcessEnv | undefined>(yes => { release = yes; });
          const opening = new Promise<void>(yes => { opened = yes; });
          let execution: vscode.TaskExecution | undefined;
          let listener: vscode.Disposable | undefined;
          let cleaningUp = false;
          let ended = false;
          try {
            const completed = new Promise<void>(yes => {
              listener = vscode.tasks.onDidEndTask(event => {
                if (event.execution.task.name === name) { ended = true; yes(); }
              });
            });
            const starting = vscode.tasks.executeTask(task).then(value => {
              execution = value;
              // A timed-out CustomExecution callback can return after cleanup.
              if (cleaningUp) { value.terminate(); }
              return value;
            });
            await within(starting, "executeTask waited for activation instead of returning a cancellable task");
            await within(opening, "The real task terminal never opened/requested its environment");
            assert.strictEqual(reads, run, "Each open must request a fresh environment exactly once");
            assert.strictEqual(ended, false, "The task must remain pending while activation is blocked");
            await assert.rejects(fs.access(output), { code: "ENOENT" }, "No command may run before activation");

            const activated = { RDE_ACTIVATED: `fresh-${run}`, RDE_TASK_MARKER: "old", RDE_REMOVED: "old" };
            release(activated);
            await within(completed, "The POSIX fixture task did not finish");
            const lines = (await fs.readFile(output, "utf8")).split("\n");
            assert.strictEqual(await fs.realpath(lines[0]), await fs.realpath(root), "Substitute task cwd");
            assert.deepStrictEqual(lines.slice(1), [
              path.basename(folder!.uri.fsPath), `fresh-${run}`, "", String(literals.length),
              "*", "?", folder!.uri.fsPath, "a'b", "$HOME", "", "",
            ], "Preserve literal arguments, substitute task variables, and apply fresh env plus overrides");
            assert.deepStrictEqual(activated, {
              RDE_ACTIVATED: `fresh-${run}`, RDE_TASK_MARKER: "old", RDE_REMOVED: "old",
            }, "Task overrides must not mutate the activated environment");
            await fs.unlink(output);
          } finally {
            cleaningUp = true;
            listener?.dispose();
            try {
              if (!ended) { execution?.terminate(); }
            } finally {
              // Undefined prevents a late open/callback from spawning a child.
              release(undefined);
            }
          }
        }
        assert.deepStrictEqual({ ...process.env }, hostEnv, "The real host environment must remain unchanged");
      } finally {
        release(undefined);
        Object.assign(extension, { resolvedEnv: previousResolvedEnv });
        await fs.rm(root, { recursive: true, force: true });
      }
    });
  }
});