import * as assert from "assert";
import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import { spawnSync } from "child_process";

describe("Test script preparation", () => {
  const root = path.resolve(__dirname, "../../..");
  const manifest = JSON.parse(fs.readFileSync(path.join(root, "package.json"), "utf8"));
  let fixture: string;

  beforeEach(() => {
    fixture = fs.mkdtempSync(path.join(os.tmpdir(), "rde-test-scripts-"));
    fs.writeFileSync(path.join(fixture, "package.json"), JSON.stringify({
      scripts: {
        test: manifest.scripts.test,
        pretest: "node step.js unexpected-pretest",
        build: "node step.js build",
        "test-compile": "node step.js compile",
      },
    }));
    fs.writeFileSync(path.join(fixture, ".npmrc"), "ignore-scripts=true\n");
    fs.writeFileSync(path.join(fixture, "step.js"), `
const fs = require("fs");
const step = process.argv[2];
fs.appendFileSync("steps.txt", step + "\\n");
if (process.env.RDE_TEST_FAIL_STEP === step) {
  process.exit(1);
}
if (step === "compile") {
  fs.mkdirSync("out/test", { recursive: true });
  fs.writeFileSync("out/test/runTest.js",
    'require("fs").appendFileSync("steps.txt", "run\\\\n");');
}
`);
  });

  afterEach(() => {
    fs.rmSync(fixture, { recursive: true, force: true });
  });

  function runTest(failStep = "") {
    return spawnSync("npm test --ignore-scripts", {
      cwd: fixture,
      shell: true,
      encoding: "utf8",
      timeout: 15000,
      env: { ...process.env, RDE_TEST_FAIL_STEP: failStep },
    });
  }

  it("builds and compiles before launching tests with lifecycle hooks disabled", () => {
    const result = runTest();
    assert.ifError(result.error);
    assert.strictEqual(result.status, 0, result.stdout + result.stderr);
    assert.strictEqual(fs.readFileSync(path.join(fixture, "steps.txt"), "utf8"),
      "build\ncompile\nrun\n");
  });

  for (const step of ["build", "compile"]) {
    it(`does not launch tests after ${step} fails`, () => {
      const result = runTest(step);
      assert.ifError(result.error);
      assert.notStrictEqual(result.status, 0, result.stdout + result.stderr);
      assert.strictEqual(fs.readFileSync(path.join(fixture, "steps.txt"), "utf8"),
        step === "build" ? "build\n" : "build\ncompile\n");
    });
  }

  it("uses the same prepared test command for headless CI", () => {
    assert.strictEqual(manifest.scripts["headless-test"], "xvfb-run -a npm test");
  });

  it("runs Mocha with the CI JUnit reporter and writes passing results", () => {
    const result = spawnSync(process.execPath, ["--eval", `
const Mocha = require(${JSON.stringify(require.resolve("mocha"))});
const mocha = new Mocha({
  reporter: ${JSON.stringify(require.resolve("mocha-junit-reporter"))},
  reporterOptions: { mochaFile: "junit.xml" },
});
mocha.suite.addTest(new Mocha.Test("reporter smoke test", () => {}));
mocha.run(failures => { process.exitCode = failures ? 1 : 0; });
`], {
      cwd: fixture,
      encoding: "utf8",
      timeout: 15000,
      env: { ...process.env, ELECTRON_RUN_AS_NODE: "1" },
    });
    assert.ifError(result.error);
    assert.strictEqual(result.status, 0, result.stdout + result.stderr);
    const report = fs.readFileSync(path.join(fixture, "junit.xml"), "utf8");
    assert.match(report, /tests="1"/);
    assert.match(report, /failures="0"/);
    assert.match(report, /name="reporter smoke test"/);
  });
});
