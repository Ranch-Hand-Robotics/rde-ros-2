import * as assert from "assert";
import * as vscode from "vscode";
import * as http from "http";
import * as fs from "fs/promises";
import * as os from "os";
import * as path from "path";
import { parse } from "jsonc-parser";

async function runDebugLaunch(): Promise<void> {
    const root = path.resolve(__dirname, "../..");
    const launch = parse(await fs.readFile(path.join(root, ".vscode/launch.json"), "utf8"));
    const enableNetworkView = process.env.RDE_TEST_DEBUG_NETWORK_VIEW === "1";
    const debugConfig = vscode.workspace.getConfiguration("debug.javascript");
    const originalNetworkView = debugConfig.inspect<boolean>("enableNetworkView")?.globalValue;
    const profile = await fs.mkdtemp(path.join(os.tmpdir(), "ros-debug-startup-"));
    const developmentPath = path.join(profile, "extension");
    await fs.symlink(root, developmentPath, "dir");
    const output: string[] = [];
    const tracker = vscode.debug.registerDebugAdapterTrackerFactory("*", {
        createDebugAdapterTracker: () => ({ onDidSendMessage: message => {
            if (message.event === "output") { output.push(message.body.output); }
        } }),
    });
    let session: vscode.DebugSession | undefined;
    const started = vscode.debug.onDidStartDebugSession(active => { if (active.name === "ROS Startup Regression") { session = active; } });
    let terminated: vscode.Disposable | undefined;
    let timer: NodeJS.Timeout | undefined;
    try {
        await debugConfig.update("enableNetworkView", enableNetworkView, vscode.ConfigurationTarget.Global);
        const finished = new Promise<void>((resolve, reject) => {
            timer = setTimeout(() => reject(new Error("Debug startup timed out")), 45000);
            terminated = vscode.debug.onDidTerminateDebugSession(active => {
                if (active.name === "ROS Startup Regression") { clearTimeout(timer); resolve(); }
            });
        });
        const accepted = await vscode.debug.startDebugging(undefined, {
            ...launch.configurations.find((config: any) => config.name === "Extension"),
            name: "ROS Startup Regression",
            preLaunchTask: undefined,
            runtimeExecutable: "${execPath}",
            trace: { logFile: path.join(profile, "debug-trace.json"), stdio: true },
            outFiles: [path.join(root, "dist/**/*.js")],
            args: [`--extensionDevelopmentPath=${developmentPath}`, `--extensionTestsPath=${path.join(root, "out/test/emptyWindow")}`, `--user-data-dir=${profile}`, "--disable-extensions", "--disable-workspace-trust", "--new-window"],
            env: { RDE_TEST_DEBUG_LAUNCH: "", RDE_TEST_ROS_DAEMON_SETUP: process.env.RDE_TEST_ROS_DAEMON_SETUP },
        });
        assert.ok(accepted, "Debugger must accept the extension launch");
        await finished;
        console.log(output.join(""));
        assert.ok(output.join("").includes("Empty-window ROS activation and registered commands passed"), `Debug host exited before completing startup; trace: ${profile}`);
        const trace = (await fs.readFile(path.join(profile, "debug-trace.json"), "utf8")).trim().split("\n").map(line => JSON.parse(line));
        const requests = trace.filter(entry => entry.tag === "cdp.send").map(entry => entry.metadata.message.method);
        assert.ok(requests.includes("Debugger.enable"), "The test must run with a debugger attached");
        if (!enableNetworkView) {
            assert.ok(!requests.includes("Network.enable"), "The workaround must avoid enabling network inspection");
        }
    } finally {
        if (timer) { clearTimeout(timer); }
        if (session) { await vscode.debug.stopDebugging(session); }
        tracker.dispose();
        started.dispose();
        terminated?.dispose();
        await debugConfig.update("enableNetworkView", originalNetworkView, vscode.ConfigurationTarget.Global);
        console.log(`Debug startup trace: ${profile}`);
    }
}

export async function run(): Promise<void> {
    if (process.env.RDE_TEST_DEBUG_LAUNCH === "1") { return runDebugLaunch(); }
    assert.ok(!vscode.workspace.workspaceFolders?.length, "Run this test without opening a folder");
    const setup = process.env.RDE_TEST_ROS_DAEMON_SETUP;
    assert.ok(setup, "RDE_TEST_ROS_DAEMON_SETUP must point to an installed ROS setup script");
    const config = vscode.workspace.getConfiguration("ROS2");
    const original = config.inspect<string>("rosSetupScript")?.globalValue;
    const requestDescriptor = Object.getOwnPropertyDescriptor(http, "request")!;
    const originalRequest = http.request;
    let responses = 0;
    let invalidChunks = 0;
    let onResponse = () => {};
    Object.defineProperty(http, "request", { ...requestDescriptor, value: (...args: any[]) => {
        const request = (originalRequest as any)(...args);
        request.on("response", (response: http.IncomingMessage) => {
            response.on("data", chunk => {
                if (!Buffer.isBuffer(chunk)) { invalidChunks++; }
                responses++;
                onResponse();
            });
        });
        return request;
    } });
    try {
        await config.update("rosSetupScript", undefined, vscode.ConfigurationTarget.Global);
        const extension = vscode.extensions.getExtension("Ranch-Hand-Robotics.rde-ros-2");
        assert.ok(extension, "Development extension must be available");
        const api = await extension.activate();
        const environmentChanged = new Promise<void>((resolve, reject) => {
            const timer = setTimeout(() => { listener.dispose(); reject(new Error("Global ROS selection did not reload the environment")); }, 15000);
            const listener = api.onDidChangeEnv(() => {
                if (api.getEnv()?.ROS_VERSION === "2") {
                    clearTimeout(timer);
                    listener.dispose();
                    resolve();
                }
            });
        });
        await config.update("rosSetupScript", setup, vscode.ConfigurationTarget.Global);
        await environmentChanged;
        assert.strictEqual(api.getEnv()?.ROS_VERSION, "2", "Empty-window activation must load the global ROS environment");
        await vscode.commands.executeCommand("ROS2.startCore");
        const hasMonitor = () => vscode.window.tabGroups.all.some(group => group.tabs.some(tab => tab.label === "ROS 2 Status"));
        const monitorOpened = new Promise<void>((resolve, reject) => {
            if (hasMonitor()) { resolve(); return; }
            const timer = setTimeout(() => { listener.dispose(); reject(new Error("Show Status did not open the monitor without a workspace")); }, 10000);
            const listener = vscode.window.tabGroups.onDidChangeTabs(() => {
                if (hasMonitor()) {
                    clearTimeout(timer);
                    listener.dispose();
                    resolve();
                }
            });
        });
        await vscode.commands.executeCommand("ROS2.showCoreStatus");
        await monitorOpened;
        const terminalCount = vscode.window.terminals.length;
        await vscode.commands.executeCommand("ROS2.createTerminal");
        assert.strictEqual(vscode.window.terminals.length, terminalCount + 1, "Create Terminal must work without a workspace");
        vscode.window.terminals[vscode.window.terminals.length - 1].dispose();
        if (responses < 5) {
            await new Promise<void>((resolve, reject) => {
                const timer = setTimeout(() => reject(new Error("No sustained daemon polling after startup")), 10000);
                onResponse = () => {
                    if (responses >= 5) {
                        clearTimeout(timer);
                        resolve();
                    }
                };
            });
        }
        assert.strictEqual(invalidChunks, 0, "HTTP chunks must retain byte lengths for debugger network inspection");
        console.log(`Validated ${responses} buffered HTTP response chunks during startup`);
        console.log("Empty-window ROS activation and registered commands passed");
    } finally {
        Object.defineProperty(http, "request", requestDescriptor);
        await config.update("rosSetupScript", original, vscode.ConfigurationTarget.Global);
    }
}