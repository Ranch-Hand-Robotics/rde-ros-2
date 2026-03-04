import * as assert from "assert";

import { compareVersions } from "../../src/vscode-utils";

describe("VS Code Utils - Version Comparison", () => {
    describe("compareVersions", () => {
        it("should detect patch-level upgrades by default", () => {
            assert.strictEqual(compareVersions("1.2.3", "1.2.4"), -1);
            assert.strictEqual(compareVersions("1.2.4", "1.2.3"), 1);
        });

        it("should ignore patch differences when ignorePatch is true", () => {
            assert.strictEqual(compareVersions("1.2.3", "1.2.4", true), 0);
            assert.strictEqual(compareVersions("1.2.9", "1.2.0", true), 0);
        });

        it("should still detect major/minor upgrades when ignorePatch is true", () => {
            assert.strictEqual(compareVersions("1.2.3", "1.3.0", true), -1);
            assert.strictEqual(compareVersions("1.2.3", "2.0.0", true), -1);
            assert.strictEqual(compareVersions("2.1.0", "1.9.9", true), 1);
        });

        it("should treat missing components as zero", () => {
            assert.strictEqual(compareVersions("1", "1.0.0"), 0);
            assert.strictEqual(compareVersions("1.2", "1.2.0"), 0);
            assert.strictEqual(compareVersions("1.2", "1.3"), -1);
        });
    });
});
