const assert = require("node:assert/strict");
const fs = require("node:fs/promises");
const os = require("node:os");
const path = require("node:path");
const { test } = require("node:test");
const { parse } = require("jsonc-parser");
const { cleanSettings, cleanProfile, validateRemoval, createPlan, executePlan } = require("../scripts/reset-ros-macos");

test("reset removes only ROS settings and retains JSONC comments", () => {
  const original = '{\n// Keep this comment\n"editor.fontSize": 14, "ROS2.distro": "jazzy", "ROS2.neverInstallRos": true,\n}';
  const updated = cleanSettings(original);
  assert.deepEqual(parse(updated), { "editor.fontSize": 14 });
  assert.match(updated, /Keep this comment/);
  assert.deepEqual(parse(cleanSettings('{"folders": [], "settings": {"ROS2.pixiRoot": "/custom", "other": true}}', true)), { folders: [], settings: { other: true } });
  assert.throws(() => cleanSettings('{"broken": }'));
});

test("profile cleanup preserves unrelated PATH entries", () => {
  const profile = 'export PATH="/other/bin:$HOME/.pixi/bin:$PATH"\nexport KEEP=yes\neval "$(pixi completion --shell zsh)"\nexport PIXI_HOME="/home/user/.pixi"\n';
  assert.equal(cleanProfile(profile, "/home/user", ["/home/user/.pixi/bin"]), 'export PATH="/other/bin:$PATH"\nexport KEEP=yes\n');
});

test("reset requires exact confirmation and deletes only planned sandbox paths", async () => {
  const home = await fs.mkdtemp(path.join(os.tmpdir(), "ros-reset-"));
  const cwd = path.join(home, "project");
  try {
    await fs.mkdir(path.join(cwd, ".vscode"), { recursive: true });
    await fs.mkdir(path.join(home, ".pixi/bin"), { recursive: true });
    await fs.writeFile(path.join(home, ".pixi/bin/pixi"), "fixture");
    await fs.writeFile(path.join(cwd, ".vscode/settings.json"), '{"ROS2.distro":"jazzy", "keep":true}');
    await fs.writeFile(path.join(home, ".zshrc"), 'export PATH="$HOME/.pixi/bin:$PATH"\nexport KEEP=yes\n');
    const plan = await createPlan({ home, cwd, env: {}, brewPrefixes: [] });
    assert.equal(await executePlan(plan, "yes"), false);
    assert.equal(await fs.readFile(path.join(home, ".pixi/bin/pixi"), "utf8"), "fixture");
    assert.equal(await executePlan(plan, "RESET"), true);
    await assert.rejects(fs.access(path.join(home, ".pixi")));
    assert.deepEqual(parse(await fs.readFile(path.join(cwd, ".vscode/settings.json"), "utf8")), { keep: true });
    assert.match(await fs.readFile(path.join(home, ".zshrc"), "utf8"), /KEEP=yes/);
    assert.ok((await fs.readdir(path.join(cwd, ".vscode"))).some(name => name.includes("before-ros-reset")));
    await assert.rejects(validateRemoval(home, home, cwd));
    await assert.rejects(validateRemoval(cwd, home, cwd));
    await assert.rejects(validateRemoval("/", home, cwd));
    await assert.rejects(validateRemoval("relative", home, cwd));
    await assert.rejects(validateRemoval(path.join(home, "Library"), home, cwd));
    await fs.symlink(os.tmpdir(), path.join(home, "linked"));
    await assert.rejects(validateRemoval(path.join(home, "linked/child"), home, cwd));
  } finally {
    await fs.rm(home, { recursive: true, force: true });
  }
});