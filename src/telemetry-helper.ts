// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import { TelemetryReporter } from '@vscode/extension-telemetry';

import * as vscode_utils from "./vscode-utils";

let reporterSingleton: TelemetryReporter | undefined = undefined;

function getTelemetryReporter(): TelemetryReporter | undefined {
    if (reporterSingleton) {
        return reporterSingleton;
    }

    const extensionId = "Ranch-Hand-Robotics.rde-ros-2";
    const packageInfo = vscode_utils.getPackageInfo(extensionId);
    if (packageInfo) {
        reporterSingleton = new TelemetryReporter(packageInfo.aiKey);
    }
    return reporterSingleton;
}

enum TelemetryEvent {
    activate = "activate",
    command = "command",
}

export interface ITelemetryReporter {
    sendTelemetryActivate(): void;
    sendTelemetryCommand(commandName: string): void;
}

class SimpleReporter implements ITelemetryReporter {
    private telemetryReporter: TelemetryReporter;

    constructor() {
        this.telemetryReporter = getTelemetryReporter();
    }

    public sendTelemetryActivate(): void {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.activate);
    }

    public sendTelemetryCommand(commandName: string): void {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.command, {
            name: commandName,
        });
    }
}

export function getReporter(): ITelemetryReporter {
    return (new SimpleReporter());
}

export async function clearReporter(): Promise<void> {
    await reporterSingleton.dispose();
    reporterSingleton = undefined;
}
