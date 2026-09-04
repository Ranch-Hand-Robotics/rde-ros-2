import * as assert from "assert";
import * as http from "http";
import { AddressInfo } from "net";

import { isMcpServerReady, waitForMcpServerReady } from "../../src/mcp";

async function reserveUnusedPort(): Promise<number> {
    return new Promise<number>((resolve, reject) => {
        const server = http.createServer();

        server.on("error", reject);
        server.listen(0, "127.0.0.1", () => {
            const address = server.address() as AddressInfo;
            const port = address.port;
            server.close((error) => {
                if (error) {
                    reject(error);
                    return;
                }
                resolve(port);
            });
        });
    });
}

async function closeServer(server?: http.Server): Promise<void> {
    if (!server) {
        return;
    }

    await new Promise<void>((resolve, reject) => {
        server.close((error) => {
            if (error) {
                reject(error);
                return;
            }
            resolve();
        });
    });
}

describe("MCP startup readiness", () => {
    it("isMcpServerReady returns false when server is not running", async () => {
        const port = await reserveUnusedPort();
        const ready = await isMcpServerReady(port, 100);

        assert.strictEqual(ready, false);
    });

    it("waitForMcpServerReady returns true once /sse endpoint comes up", async () => {
        const port = await reserveUnusedPort();
        let server: http.Server | undefined;

        try {
            setTimeout(() => {
                server = http.createServer((request, response) => {
                    if (request.url === "/sse") {
                        response.writeHead(200, { "Content-Type": "text/event-stream" });
                        response.end();
                        return;
                    }

                    response.writeHead(404);
                    response.end();
                });

                server.listen(port, "127.0.0.1");
            }, 150);

            const ready = await waitForMcpServerReady(port, 3000, 100, 200);
            assert.strictEqual(ready, true);
        } finally {
            await closeServer(server);
        }
    });

    it("waitForMcpServerReady returns false after timeout", async () => {
        const port = await reserveUnusedPort();
        const ready = await waitForMcpServerReady(port, 500, 100, 100);

        assert.strictEqual(ready, false);
    });
});
