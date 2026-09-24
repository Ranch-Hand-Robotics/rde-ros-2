import * as assert from "assert";
import * as xmlrpc from "xmlrpc";
import * as http from "http";
import { AddressInfo } from "net";
import * as extension from "../../src/extension";
import { getDaemonUri, XmlRpcApi } from "../../src/ros/ros2/ros2-monitor";
import { startDaemon, StatusBarItem } from "../../src/ros/ros2/daemon";
import { sourceBashEnvironment } from "../../src/ros/installer/pixi";

describe("ROS daemon connection", () => {
    let originalEnv: PropertyDescriptor;
    let originalDomain: string | undefined;

    beforeEach(() => {
        originalEnv = Object.getOwnPropertyDescriptor(extension, "env")!;
        originalDomain = process.env.ROS_DOMAIN_ID;
    });

    afterEach(() => {
        Object.defineProperty(extension, "env", originalEnv);
        if (originalDomain === undefined) {
            delete process.env.ROS_DOMAIN_ID;
        } else {
            process.env.ROS_DOMAIN_ID = originalDomain;
        }
    });

    function setEnvironment(env: NodeJS.ProcessEnv | undefined): void {
        Object.defineProperty(extension, "env", { ...originalEnv, value: env });
    }

    it("uses IPv4 and the activated environment instead of the editor's domain", () => {
        process.env.ROS_DOMAIN_ID = "42";
        setEnvironment({ ROS_DOMAIN_ID: "7" });
        assert.strictEqual(getDaemonUri(), "http://127.0.0.1:11518/ros2cli/");
        setEnvironment({});
        assert.strictEqual(getDaemonUri(), "http://127.0.0.1:11511/ros2cli/");
    });

    it("survives an environment cleared while a status poll is pending", async () => {
        const monitor = new StatusBarItem();
        const internal = monitor as unknown as { update(): Promise<void>; ros2cli: { getNodeNamesAndNamespaces(): Promise<any> } };
        internal.ros2cli = { getNodeNamesAndNamespaces: async () => { setEnvironment(undefined); return []; } };
        try {
            await internal.update();
        } finally {
            monitor.dispose();
        }
    });

    it("does not restart polling after disposal during a pending request", async () => {
        const monitor = new StatusBarItem();
        const internal = monitor as unknown as { update(): Promise<void>; timeout?: NodeJS.Timeout; ros2cli: { getNodeNamesAndNamespaces(): Promise<any> } };
        let complete!: (nodes: any[]) => void;
        internal.ros2cli = { getNodeNamesAndNamespaces: () => new Promise(resolve => { complete = resolve; }) };
        const pending = internal.update();
        monitor.dispose();
        complete([]);
        await pending;
        assert.strictEqual(internal.timeout, undefined);
    });

    it("connects to an IPv4 daemon and follows environment changes after construction", async () => {
        setEnvironment({ ROS_DOMAIN_ID: "0" });
        const api = new XmlRpcApi();
        const server = await new Promise<xmlrpc.Server>((resolve) => {
            const listening = xmlrpc.createServer({ host: "127.0.0.1", port: 0, path: "/ros2cli/" }, () => resolve(listening));
        });
        const nodes = [["test_node", "/"]];
        const requestDescriptor = Object.getOwnPropertyDescriptor(http, "request")!;
        const originalRequest = http.request;
        let responseEncoding: BufferEncoding | null | undefined;
        Object.defineProperty(http, "request", { ...requestDescriptor, value: (...args: any[]) => {
            const request = (originalRequest as any)(...args);
            request.on("response", (response: http.IncomingMessage) => {
                response.on("end", () => { responseEncoding = response.readableEncoding; });
            });
            return request;
        } });
        server.on("get_node_names_and_namespaces", (_error, _params, callback) => callback(null, nodes));
        try {
            const port = (server.httpServer.address() as AddressInfo).port;
            setEnvironment({ ROS_DOMAIN_ID: String(port - 11511) });
            assert.deepStrictEqual(await api.getNodeNamesAndNamespaces(), nodes);
            assert.strictEqual(await api.check(), true);
            assert.strictEqual(responseEncoding, null, "HTTP responses must remain buffers for Node inspector network events");
        } finally {
            Object.defineProperty(http, "request", requestDescriptor);
            await new Promise<void>((resolve, reject) => server.httpServer.close(error => error ? reject(error) : resolve()));
        }
    });

    it("starts and queries an installed Pixi daemon from the extension host", async function () {
        const setup = process.env.RDE_TEST_ROS_DAEMON_SETUP;
        if (!setup) { this.skip(); }
        this.timeout(60000);
        const output = Object.getOwnPropertyDescriptor(extension, "outputChannel")!;
        const messages: string[] = [];
        try {
            setEnvironment(await sourceBashEnvironment(setup));
            Object.defineProperty(extension, "outputChannel", {
                ...output, value: { appendLine: (message: string) => { messages.push(message); console.log(message); } },
            });
            await startDaemon();
            const api = new XmlRpcApi();
            assert.strictEqual(await api.check(), true);
            assert.ok(Array.isArray(await api.getTopicNamesAndTypes()));
            assert.ok(Array.isArray(await api.getServiceNamesAndTypes()));
            assert.ok(messages.some(message => message.includes("daemon is already running") || message.includes("daemon has been started")));
        } finally {
            Object.defineProperty(extension, "outputChannel", output);
        }
    });
});