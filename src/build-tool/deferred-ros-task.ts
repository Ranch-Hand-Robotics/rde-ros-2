// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as cp from "child_process";
import * as path from "path";
import { quote } from "shell-quote";
import * as vscode from "vscode";
import type { RosTaskDefinition } from "./ros-shell";

export interface RosTaskOptions {
  cwd?: string;
  env?: { [key: string]: string | null };
  shell?: { executable: string; args?: string[] };
}

/** Do not await activation in either discovery or the CustomExecution callback.
 * Waiting in open() lets VS Code attach listeners and cancel pending activation.
 */
export function deferredRosExecution(
  getEnv: () => Promise<NodeJS.ProcessEnv | undefined>, defaultCwd?: string,
): vscode.CustomExecution {
  return new vscode.CustomExecution(async definition =>
    new DeferredRosTerminal(definition as RosTaskDefinition, getEnv, defaultCwd));
}

class DeferredRosTerminal implements vscode.Pseudoterminal {
  private readonly writes = new vscode.EventEmitter<string>();
  private readonly closes = new vscode.EventEmitter<number>();
  readonly onDidWrite = this.writes.event;
  readonly onDidClose = this.closes.event;
  private child?: cp.ChildProcess;
  private started = false;
  private finished = false;

  constructor(
    private readonly definition: RosTaskDefinition,
    private readonly getEnv: () => Promise<NodeJS.ProcessEnv | undefined>,
    private readonly defaultCwd?: string,
  ) {}

  open(): void {
    if (this.started || this.finished) { return; }
    this.started = true;
    void this.run();
  }

  close(): void {
    if (this.finished) { return; }
    this.stopChild();
    this.finish(130);
  }

  handleInput(data: string): void {
    if (this.finished) { return; }
    if (data.includes("\x03")) { this.close(); }
    else {
      try { this.child?.stdin?.write(data.replace(/\r/g, "\n")); }
      catch (error) { this.fail(error); }
    }
  }

  private stopChild(): void {
    if (!this.child?.pid) { return; }
    // Each Unix task owns a process group. Stop shells, colcon and their children,
    // not just the immediate shell. No external kill executable is needed.
    try { process.kill(-this.child.pid, "SIGKILL"); }
    catch (error) {
      if ((error as NodeJS.ErrnoException).code === "ESRCH") { return; }
      try { this.child.kill("SIGKILL"); }
      catch (failure) { this.write(`Failed to stop ROS task: ${String(failure)}\r\n`); }
    }
  }

  private write(text: string): void {
    if (!this.finished) { this.writes.fire(text); }
  }

  private finish(code: number): void {
    if (this.finished) { return; }
    this.finished = true;
    this.closes.fire(code);
    this.writes.dispose();
    this.closes.dispose();
  }

  private fail(error: unknown): void {
    if (this.finished) { return; }
    this.write(`ROS task failed: ${error instanceof Error ? error.message : String(error)}\r\n`);
    this.stopChild();
    this.finish(1);
  }

  private pipe(stream: NodeJS.ReadableStream & { setEncoding(encoding: BufferEncoding): unknown }): void {
    stream.setEncoding("utf8");
    let lastWasCR = false;
    stream.on("data", (text: string) => {
      // Keep CRLF intact even when a stream splits it across chunks.
      const output = text.replace(/\n/g, (_, offset: number) =>
        (offset === 0 ? lastWasCR : text[offset - 1] === "\r") ? "\n" : "\r\n");
      if (text.length) { lastWasCR = text.endsWith("\r"); }
      this.write(output);
    });
    stream.on("error", error => this.fail(error));
  }

  private async run(): Promise<void> {
    try {
      this.write("Waiting for the ROS environment...\r\n");
      const resolvedEnv = await this.getEnv();
      if (this.finished) { return; }
      if (!resolvedEnv) {
        throw new Error("No ROS environment is available. Select a ROS installation and rerun the task.");
      }
      const options = this.definition.taskOptions ?? this.definition.options;
      const env = { ...resolvedEnv };
      for (const [key, value] of Object.entries(options?.env ?? {})) {
        if (value === null) { delete env[key]; }
        else { env[key] = value; }
      }
      const shell = options?.shell?.executable ?? "/bin/sh";
      // shell-quote uses POSIX quoting. Do not silently corrupt literal arguments
      // in fish, PowerShell, or other shells with different quoting rules.
      if (!["sh", "bash", "dash", "zsh", "ksh"].includes(path.posix.basename(shell))) {
        throw new Error(`Unsupported ROS task shell '${shell}'. Use a POSIX shell in taskOptions.shell.`);
      }
      const command = this.definition.command;
      const args = this.definition.args ?? [];
      if (typeof command !== "string" || !command || args.some(arg => typeof arg !== "string")) {
        throw new Error("ROS tasks require a command and string arguments.");
      }
      // Keep the structured ShellExecution(command, args) contract: command and
      // args are literal words, while shell built-ins still work. For an explicit
      // pipeline/expansion use command 'sh' and args ['-c', '<shell script>'].
      const commandLine = quote([command, ...args]);
      this.child = cp.spawn(shell, [...(options?.shell?.args ?? ["-c"]), commandLine], {
        cwd: options?.cwd ?? this.defaultCwd, env, shell: false, detached: true, stdio: "pipe",
      });
      if (this.child.stdout) { this.pipe(this.child.stdout); }
      if (this.child.stderr) { this.pipe(this.child.stderr); }
      this.child.stdin?.on("error", error => this.fail(error));
      this.child.once("error", error => this.fail(error));
      // 'exit' precedes drained stdio; use 'close' to keep final diagnostics.
      this.child.once("close", (code, signal) => {
        this.finish(code ?? (signal === "SIGINT" ? 130 : signal === "SIGTERM" ? 143 : 1));
      });
    } catch (error) {
      this.fail(error);
    }
  }
}