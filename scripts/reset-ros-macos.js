const fs = require("node:fs/promises");
const os = require("node:os");
const path = require("node:path");
const readline = require("node:readline/promises");
const { execFileSync } = require("node:child_process");
const { parse, modify, applyEdits } = require("jsonc-parser");

const settingsKeys = ["ROS2.rosSetupScript", "ROS2.distro", "ROS2.pixiRoot", "ROS2.pixiInstallLocationsByMachine", "ROS2.neverInstallRos"];

async function readOptional(filename) {
  try {
    return await fs.readFile(filename, "utf8");
  } catch (error) {
    if (error.code === "ENOENT") { return undefined; }
    throw error;
  }
}

function cleanSettings(content, workspaceFile = false) {
  const errors = [];
  parse(content, errors, { allowTrailingComma: true });
  if (errors.length) { throw new Error("Invalid JSONC settings; fix syntax before resetting."); }
  for (const key of settingsKeys) {
    content = applyEdits(content, modify(content, workspaceFile ? ["settings", key] : [key], undefined, {}));
  }
  return content;
}

function cleanProfile(content, home, binDirectories) {
  const tokens = new Set(["$HOME/.pixi/bin", "${HOME}/.pixi/bin", "~/.pixi/bin", "$PIXI_HOME/bin", "${PIXI_HOME}/bin", "$PIXI_BIN_DIR", "${PIXI_BIN_DIR}"]);
  for (const directory of binDirectories) {
    tokens.add(directory);
    if (directory.startsWith(`${home}/`)) {
      tokens.add(directory.replace(home, "$HOME"));
      tokens.add(directory.replace(home, "${HOME}"));
    }
  }
  return content.split("\n").filter(line => {
    return !/^\s*(?:export\s+)?PIXI_(?:HOME|BIN_DIR|CACHE_DIR)=.*$/.test(line)
      && !/^\s*eval\s+["']?\$\(pixi completion --shell (?:bash|zsh)\)["']?\s*$/.test(line)
      && !/^\s*pixi completion --shell fish\s*\|\s*source\s*$/.test(line);
  }).map(line => {
    if (/^\s*(?:export\s+)?PATH=/.test(line)) {
      for (const token of tokens) {
        line = line.split(`${token}:`).join("").split(`:${token}`).join("");
      }
      if (/^\s*(?:export\s+)?PATH=["']?\$(?:PATH|\{PATH\})["']?\s*$/.test(line)) { return ""; }
    }
    if (/^\s*(?:fish_add_path|set -gx PATH) /.test(line)) {
      for (const token of tokens) {
        line = line.split(`"${token}"`).join("").split(`'${token}'`).join("").split(token).join("");
      }
      if (/^\s*(?:fish_add_path|set -gx PATH\s+\$PATH)\s*$/.test(line)) { return ""; }
    }
    return line;
  }).join("\n");
}

async function validateRemoval(target, home, cwd) {
  if (!path.isAbsolute(target)) { throw new Error(`Reset requires an absolute path: ${target}`); }
  const resolved = path.resolve(target);
  const protectedPaths = ["Library", "Library/Caches", "Library/Application Support", "Documents", "Desktop", "Downloads", ".config", ".cache", ".local", ".local/bin", ".cargo", ".cargo/bin"];
  if (!resolved.startsWith(`${path.resolve(home)}${path.sep}`)
      || resolved === cwd || cwd.startsWith(`${resolved}${path.sep}`)
      || protectedPaths.some(directory => resolved === path.join(home, directory))) {
    throw new Error(`Refusing unsafe deletion: ${resolved}. Reset only removes paths below your home, never the current project or its parents.`);
  }
  let ancestor = resolved;
  while (ancestor !== home) {
    try {
      const info = await fs.lstat(ancestor);
      if (info.isSymbolicLink()) { throw new Error(`Refusing deletion through symlink: ${ancestor}`); }
    } catch (error) {
      if (error.code !== "ENOENT") { throw error; }
    }
    ancestor = path.dirname(ancestor);
  }
  return resolved;
}

async function createPlan({ home = os.homedir(), cwd = process.cwd(), env = process.env, roots = [], settings = [], brewPrefixes = ["/opt/homebrew", "/usr/local"] } = {}) {
  home = path.resolve(home);
  cwd = path.resolve(cwd);
  const settingsFiles = new Set([path.join(cwd, ".vscode/settings.json"), ...settings.map(filename => path.resolve(filename))]);
  for (const product of ["Code", "Code - Insiders", "Cursor"]) {
    const user = path.join(home, "Library/Application Support", product, "User");
    settingsFiles.add(path.join(user, "settings.json"));
    try {
      for (const profile of await fs.readdir(path.join(user, "profiles"))) {
        settingsFiles.add(path.join(user, "profiles", profile, "settings.json"));
      }
    } catch (error) {
      if (error.code !== "ENOENT") { throw error; }
    }
  }
  const edits = [];
  const removals = new Set([path.join(home, "pixi_ws"), path.join(home, ".pixi"), ...roots]);
  for (const filename of settingsFiles) {
    const content = await readOptional(filename);
    if (content === undefined) { continue; }
    const workspaceFile = filename.endsWith(".code-workspace");
    const updated = cleanSettings(content, workspaceFile);
    const document = parse(content);
    const config = workspaceFile ? document.settings || {} : document;
    if (config["ROS2.pixiRoot"]) { removals.add(config["ROS2.pixiRoot"]); }
    const machineLocations = config["ROS2.pixiInstallLocationsByMachine"];
    if (machineLocations && typeof machineLocations === "object" && !Array.isArray(machineLocations)) {
      for (const root of Object.values(machineLocations)) {
        if (typeof root === "string" && root) { removals.add(root); }
      }
    }
    if (updated !== content) { edits.push({ filename, content: updated }); }
  }
  const pixiHome = env.PIXI_HOME || path.join(home, ".pixi");
  removals.add(pixiHome);
  for (const directory of [
    path.join(home, "Library/Caches/pixi"), path.join(home, "Library/Caches/rattler"),
    path.join(home, "Library/Application Support/pixi"),
    path.join(home, ".cache/pixi"), path.join(home, ".cache/rattler"), path.join(home, ".config/pixi"),
    env.PIXI_CACHE_DIR,
    env.XDG_CACHE_HOME && path.join(env.XDG_CACHE_HOME, "pixi"),
    env.XDG_CACHE_HOME && path.join(env.XDG_CACHE_HOME, "rattler"),
    env.XDG_CONFIG_HOME && path.join(env.XDG_CONFIG_HOME, "pixi"),
  ].filter(Boolean)) { removals.add(directory); }
  const binDirectories = new Set([path.join(pixiHome, "bin"), path.join(home, ".pixi/bin")]);
  if (env.PIXI_BIN_DIR) {
    binDirectories.add(env.PIXI_BIN_DIR);
    removals.add(path.join(env.PIXI_BIN_DIR, "pixi"));
  }
  for (const directory of [".cargo/bin", ".local/bin"]) {
    const binary = path.join(home, directory, "pixi");
    if (await fs.lstat(binary).catch(() => undefined)) { removals.add(binary); }
  }
  const brewCommands = [];
  for (const prefix of brewPrefixes) {
    const binary = path.join(prefix, "bin/pixi");
    const link = await fs.readlink(binary).catch(() => "");
    if (link.includes("/Cellar/pixi/") || link.includes("../Cellar/pixi/")) {
      brewCommands.push([path.join(prefix, "bin/brew"), ["uninstall", "--force", "pixi"]]);
    } else if (await fs.lstat(binary).catch(() => undefined)) {
      throw new Error(`Unmanaged Pixi executable at ${binary}; uninstall it using its original installer first.`);
    }
  }
  for (const filename of new Set([
    ...[".zshrc", ".zprofile", ".zshenv", ".bashrc", ".bash_profile", ".profile", ".config/fish/config.fish"].map(name => path.join(home, name)),
    ...[".zshrc", ".zprofile", ".zshenv"].map(name => path.join(env.ZDOTDIR || home, name)),
    path.join(env.XDG_CONFIG_HOME || path.join(home, ".config"), "fish/config.fish"),
  ])) {
    const content = await readOptional(filename);
    if (content === undefined) { continue; }
    const updated = cleanProfile(content, home, binDirectories);
    if (updated !== content) { edits.push({ filename, content: updated }); }
  }
  const targets = [];
  for (const target of removals) { targets.push(await validateRemoval(target, home, cwd)); }
  return { targets, edits, brewCommands };
}

async function executePlan(plan, answer) {
  if (answer !== "RESET") { return false; }
  for (const [command, args] of plan.brewCommands) {
    execFileSync(command, args, { stdio: "inherit", env: { ...process.env, HOMEBREW_NO_AUTO_UPDATE: "1" } });
  }
  for (const edit of plan.edits) {
    await fs.copyFile(edit.filename, `${edit.filename}.before-ros-reset-${Date.now()}`);
    await fs.writeFile(edit.filename, edit.content);
  }
  for (const target of plan.targets) { await fs.rm(target, { recursive: true, force: true }); }
  return true;
}

async function main() {
  if (process.platform !== "darwin") { throw new Error("This reset task is macOS-only."); }
  const roots = [];
  const settings = [];
  let dryRun = false;
  const args = process.argv.slice(2);
  while (args.length) {
    const option = args.shift();
    if (option === "--dry-run") { dryRun = true; }
    else if ((option === "--pixi-root" || option === "--settings") && args.length) {
      (option === "--pixi-root" ? roots : settings).push(args.shift());
    } else { throw new Error(`Unknown or incomplete option: ${option}`); }
  }
  const plan = await createPlan({ roots, settings });
  console.log("Deletes ALL contents of the following paths, including ROS workspaces, Pixi global tools, configuration, credentials and shared caches:");
  plan.targets.forEach(target => console.log(`  ${target}`));
  plan.brewCommands.forEach(([command, args]) => console.log(`Run: ${command} ${args.join(" ")}`));
  plan.edits.forEach(edit => console.log(`Edit (with backup): ${edit.filename}`));
  console.log("Stop ROS processes, close other editor windows, and disable Settings Sync before continuing. Unrelated settings and shell entries are preserved.");
  if (dryRun) { return; }
  if (!process.stdin.isTTY) { throw new Error("Interactive confirmation is required. Run this command in a terminal."); }
  const prompt = readline.createInterface({ input: process.stdin, output: process.stdout });
  let answer;
  try { answer = await prompt.question("Type RESET to permanently remove these installations: "); }
  finally { prompt.close(); }
  const reset = await executePlan(plan, answer);
  console.log(reset ? "Reset complete. Fully quit and reopen VS Code/Cursor and your terminals to discard inherited ROS/Pixi environment variables." : "Cancelled. Nothing changed.");
}

module.exports = { cleanSettings, cleanProfile, validateRemoval, createPlan, executePlan };
if (require.main === module) {
  main().catch(error => { console.error(error.message); process.exitCode = 1; });
}