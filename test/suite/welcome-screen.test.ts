import * as assert from "assert";

import { resolveWelcomePromptSelectionWithTimeout, shouldShowWelcome } from "../../src/extension";

describe("Welcome screen startup behavior", () => {
    describe("shouldShowWelcome", () => {
        it("shows welcome on first install", () => {
            assert.strictEqual(shouldShowWelcome("", "1.0.0"), true);
        });

        it("shows welcome on major/minor upgrades", () => {
            assert.strictEqual(shouldShowWelcome("1.2.3", "1.3.0"), true);
            assert.strictEqual(shouldShowWelcome("1.2.3", "2.0.0"), true);
        });

        it("does not show welcome on patch-only updates", () => {
            assert.strictEqual(shouldShowWelcome("1.2.3", "1.2.4"), false);
        });
    });

    describe("resolveWelcomePromptSelectionWithTimeout", () => {
        it("returns the selection when user responds before timeout", async () => {
            const result = await resolveWelcomePromptSelectionWithTimeout(Promise.resolve("Yes"), 1000);
            assert.strictEqual(result, "Yes");
        });

        it("returns undefined when prompt times out", async () => {
            const neverResolves = new Promise<string | undefined>(() => undefined);
            const result = await resolveWelcomePromptSelectionWithTimeout(neverResolves, 20);
            assert.strictEqual(result, undefined);
        });

        it("returns undefined immediately for non-positive timeout", async () => {
            const result = await resolveWelcomePromptSelectionWithTimeout(Promise.resolve("Yes"), 0);
            assert.strictEqual(result, undefined);
        });
    });
});
