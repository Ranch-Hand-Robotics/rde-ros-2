// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

export interface PreflightCheck {
  id: string;
  status: "passed" | "warning" | "blocked";
  detail: string;
  remediation?: string;
}

export interface PreflightReport {
  ready: boolean;
  checks: PreflightCheck[];
}

export interface PreflightCommandResult {
  exitCode: number | null;
  stdout: string;
  stderr: string;
  error?: string;
}

export type PreflightRunner = (command: string, args: string[], timeoutMs?: number) => Promise<PreflightCommandResult>;
