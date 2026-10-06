import * as cp from "child_process";
import * as path from "path";
import * as vscode from "vscode";
import type { RosTaskDefinition } from "./ros-shell";
import { preflightWindowsBuild } from "./windows-build-preflight";
import { buildInstallPrefixes, cleanBuildEnvironment, RosBuildOptions } from "../ros/build-environment";

/** CustomExecution runs the check at execution time, not during task discovery. */
export function windowsColconExecution(
  getEnv: () => NodeJS.ProcessEnv, onOutput: (message: string) => void, defaultCwd?: string,
  prepareRos?: (env: NodeJS.ProcessEnv, options: RosBuildOptions) => Promise<NodeJS.ProcessEnv>,
): vscode.CustomExecution {
  return new vscode.CustomExecution(async definition =>
    new WindowsColconTerminal(definition as RosTaskDefinition, getEnv, onOutput, defaultCwd, prepareRos));
}

class WindowsColconTerminal implements vscode.Pseudoterminal {
  private readonly writes = new vscode.EventEmitter<string>();
  private readonly closes = new vscode.EventEmitter<number>();
  readonly onDidWrite = this.writes.event;
  readonly onDidClose = this.closes.event;
  private child?: cp.ChildProcess;
  private cancelled = false;
  private finished = false;

  constructor(
    private readonly definition: RosTaskDefinition,
    private readonly getEnv: () => NodeJS.ProcessEnv,
    private readonly onOutput: (message: string) => void,
    private readonly defaultCwd?: string,
    private readonly prepareRos?: (env: NodeJS.ProcessEnv, options: RosBuildOptions) => Promise<NodeJS.ProcessEnv>,
  ) {}

  open(): void { void this.run(); }

  close(): void {
    this.cancelled = true;
    const child = this.child;
    if (child?.pid && !this.finished) {
      // Killing only colcon leaves CMake/MSBuild/compiler subprocesses running.
      const taskkill = path.join(process.env.SystemRoot || "C:\\Windows", "System32", "taskkill.exe");
      cp.execFile(taskkill, ["/pid", String(child.pid), "/T", "/F"], { windowsHide: true }, error => {
        if (error) { child.kill(); }
      });
    } else {
      this.finish(130);
    }
  }

  handleInput(data: string): void {
    if (data.includes("\x03")) { this.close(); }
    else { this.child?.stdin?.write(data.replace(/\r/g, "\n")); }
  }

  private write(text: string): void {
    if (!this.finished) { this.writes.fire(text.replace(/\r?\n/g, "\r\n")); }
  }

  private finish(code: number): void {
    if (this.finished) { return; }
    this.finished = true;
    this.closes.fire(code);
    this.writes.dispose();
    this.closes.dispose();
  }

  private async run(): Promise<void> {
    if (this.cancelled) { return; }
    try {
      // Use the host baseline plus overrides, never the runtime overlay snapshot.
      const env = { ...this.getEnv() };
      const options = this.definition.buildOptions ?? this.definition.options;
      for (const [key, value] of Object.entries(options?.env ?? {})) {
        for (const existing of Object.keys(env)) {
          if (existing.toLowerCase() === key.toLowerCase()) { delete env[existing]; }
        }
        if (value !== null && value !== undefined) { env[key] = value; }
      }
      const cwd = options?.cwd ?? this.defaultCwd;
      const buildOptions = { cwd, args: this.definition.args,
        onOutput: (message: string) => { this.onOutput(message); this.write(message + "\n"); } };
      const clean = cleanBuildEnvironment(env, buildInstallPrefixes(this.defaultCwd, buildOptions));
      this.write("Checking Windows C++ build environment...\n");
      let activated = await preflightWindowsBuild(clean, {
        cwd, onOutput: message => { this.onOutput(message); this.write(message + "\n"); },
      }, () => this.cancelled);
      if (this.cancelled) { return; }
      if (!activated) {
        this.write("Build stopped before launching colcon. Install or repair the toolchain and rerun the build.\n");
        this.finish(1);
        return;
      }
      if (this.prepareRos) {
        this.write("Checking ROS underlays, ros2, and colcon (without the workspace install overlay)...\n");
        activated = await this.prepareRos(activated, buildOptions);
        if (this.cancelled) { return; }
      }
      const command = this.definition.command === "colcon" ? "colcon.exe" : this.definition.command;
      const args = this.definition.args ?? [];
      // No shell interpolation: spaces and metacharacters in package paths stay arguments.
      this.child = cp.spawn(command, args, { env: activated, cwd, shell: false, windowsHide: true, stdio: "pipe" });
      this.child.stdout?.setEncoding("utf8");
      this.child.stderr?.setEncoding("utf8");
      this.child.stdout?.on("data", text => this.write(text));
      this.child.stderr?.on("data", text => this.write(text));
      this.child.once("error", error => {
        this.write(`Failed to start colcon build: ${error.message}\n`);
        this.finish(1);
      });
      this.child.once("close", code => {
        this.write(`Colcon build exited with code ${code ?? 1}.\n`);
        this.finish(this.cancelled ? 130 : code ?? 1);
      });
    } catch (error) {
      const message = `Colcon build preflight/start failed: ${error instanceof Error ? error.message : String(error)}`;
      this.onOutput(message);
      this.write(message + "\n");
      if (!this.cancelled) {
        void vscode.window.showErrorMessage(`${message}\nSee Output > ROS 2 for details. No build was started.`);
      }
      this.finish(this.cancelled ? 130 : 1);
    }
  }
}