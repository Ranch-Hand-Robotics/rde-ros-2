import * as assert from "assert";
import * as fs from "fs";
import * as path from "path";
import * as yaml from "js-yaml";

describe("Bundled rosdep skill", () => {
  const root = path.resolve(__dirname, "../../..");
  const manifest = JSON.parse(fs.readFileSync(path.join(root, "package.json"), "utf8"));
  const skillPath = "./assets/skills/rosdep/SKILL.md";
  const filename = path.join(root, skillPath);
  const skill = fs.readFileSync(filename, "utf8").replace(/\r\n/g, "\n");

  it("contributes a discoverable skill with valid matching frontmatter", () => {
    assert.strictEqual(manifest.contributes.chatSkills.filter(entry => entry.path === skillPath).length, 1);
    const frontmatter = /^---\r?\n([\s\S]*?)\r?\n---\r?\n/.exec(skill);
    assert.ok(frontmatter, "YAML frontmatter is required");
    const metadata = yaml.load(frontmatter[1]) as Record<string, unknown>;
    assert.strictEqual(metadata.name, path.basename(path.dirname(filename)));
    assert.match(String(metadata.name), /^[a-z0-9]+(?:-[a-z0-9]+)*$/);
    assert.ok(typeof metadata.description === "string" && metadata.description.length <= 1024);
    for (const trigger of ["rosdep", "Pixi", "RoboStack", "Jetson", "Windows", "macOS", "repositories"]) {
      assert.ok(String(metadata.description).includes(trigger), `Missing trigger: ${trigger}`);
    }
    assert.notStrictEqual(metadata["user-invocable"], false);
    assert.notStrictEqual(metadata["disable-model-invocation"], true);
    assert.ok(skill.split("\n").length < 120, "Keep the default workflow short");
  });

  it("resolves all bundled references within the skill directory", () => {
    const directory = path.dirname(filename);
    const links = [...skill.matchAll(/\]\((\.\/[^)]+)\)/g)];
    assert.ok(links.length > 0, "Platform guidance must be linked for on-demand loading");
    for (const [, link] of links) {
      const reference = path.resolve(directory, link);
      assert.ok(reference.startsWith(directory + path.sep));
      assert.ok(fs.statSync(reference).isFile(), `Missing resource: ${link}`);
    }
    const platforms = fs.readFileSync(path.join(directory, "references/platforms.md"), "utf8");
    for (const target of ["JetPack", "linux-aarch64", "Windows", "macOS", "osx-arm64"]) {
      assert.ok(platforms.includes(target), `Missing platform guidance: ${target}`);
    }
  });

  it("retains source provenance, approval, and verification guardrails", () => {
    // Content contracts, not a claim that an LLM will always follow the instructions.
    for (const requirement of ["source.url", "source.version", "release.packages", "commit SHA",
      "Get approval before cloning", "overwrite, reset", "--skip-keys", "untrusted input",
      "availability is unknown", "confirm\nHEAD and the expected package manifest"]) {
      assert.ok(skill.includes(requirement), `Missing guardrail: ${requirement}`);
    }
    assert.ok(manifest.contributes.commands.some(entry => entry.command === "ROS2.rosdep"),
      "Keep the existing rosdep command available");
  });

  it("documents invocation and keeps the public guide in navigation", () => {
    const guide = fs.readFileSync(path.join(root, "docs/dependency-recovery.md"), "utf8");
    const walkthrough = fs.readFileSync(path.join(root, "media/walkthroughs/ai-features.md"), "utf8");
    assert.ok(guide.includes("/rosdep") && walkthrough.includes("/rosdep"));
    assert.ok(guide.includes("peer directory of the requesting package"));
    assert.ok(walkthrough.includes("peer-directory destination"));
    assert.ok(guide.includes("Older VS Code versions"));
    assert.ok(guide.includes("not a deterministic dependency resolver"));
    const config = yaml.load(fs.readFileSync(path.join(root, "mkdocs.yml"), "utf8")) as {
      nav: Array<Record<string, string | string[]>>;
    };
    const usage = config.nav.find(section => section.Usage)?.Usage;
    assert.ok(Array.isArray(usage));
    assert.ok(usage.includes("pixi.md"));
    assert.ok(usage.includes("dependency-recovery.md"), "Guides must be separate navigation entries");
    for (const section of config.nav) {
      for (const target of Object.values(section).flat()) {
        assert.ok(fs.statSync(path.join(root, "docs", target)).isFile(), `Missing navigation target: ${target}`);
      }
    }
  });

  it("seeds diagnostics mappings without treating the catalog as an install list", () => {
    assert.ok(skill.includes("./references/common-packages.md"));
    const catalog = fs.readFileSync(path.join(path.dirname(filename), "references/common-packages.md"), "utf8");
    assert.ok(catalog.includes("https://github.com/ros/diagnostics.git"));
    for (const name of ["diagnostic_updater", "diagnostic_aggregator", "self_test"]) {
      assert.ok(catalog.includes(`\`${name}/package.xml\``), `Missing source mapping for ${name}`);
      assert.ok(skill.includes(name), `Missing catalog discovery hint for ${name}`);
    }
    for (const guardrail of ["check", "approval", "availability unknown", "commit SHA",
      "No fixed", "at most once", "--packages-up-to diagnostic_updater",
      "colcon test-result --verbose", "no Windows, macOS, or Jetson source build"]) {
      assert.ok(catalog.includes(guardrail), `Missing recipe guardrail: ${guardrail}`);
    }
  });

  it("records distro branch candidates and the Lyrical metadata conflict", () => {
    const catalog = fs.readFileSync(path.join(path.dirname(filename), "references/common-packages.md"), "utf8");
    for (const [distro, branch] of [["Humble", "ros2-humble"], ["Jazzy", "ros2-jazzy"],
      ["Kilted", "ros2-kilted"], ["Lyrical", "ros2-lyrical"], ["Rolling", "ros2"]]) {
      assert.ok(catalog.includes(`| ${distro} | \`${branch}\` |`));
    }
    assert.ok(catalog.includes("`ros2` — conflict; investigate before selecting"));
    assert.ok(catalog.includes("Source metadata reviewed:"));
    assert.ok(catalog.includes("diagnostic_msgs` is a separate dependency"));
    assert.ok(catalog.includes("ros2-distro-mutex"));
  });

  it("prioritizes direct platform lookup then a concrete source-clone offer", () => {
    const binary = skill.indexOf("## 2. Look for a binary");
    const source = skill.indexOf("## 3. If no binary");
    const offer = skill.indexOf("## 4. Offer a peer-directory clone");
    const approved = skill.indexOf("## 5. After the user accepts");
    assert.ok(binary > 0 && source > binary && offer > source && approved > offer);
    for (const instruction of ["pixi search <candidate>", "--platform <target> --json --limit -1",
      "Do not also research source repositories", "Do not recursively resolve transitive dependencies",
      "Clone into `<absolute peer directory>`", "no dependency table by default",
      "Do not build or install anything by default"]) {
      assert.ok(skill.includes(instruction), `Missing fast-path instruction: ${instruction}`);
    }
    assert.ok(skill.includes("Do not require a `.repos` manifest"));
  });

  it("places every documented clone beside the requesting package, not inside it", () => {
    assert.ok(skill.includes("destination = join(dirname(failingPackageDir), repoBasename)"));
    const examples = [...skill.matchAll(/^\| `([^`]+\/package\.xml)` \| `([^`]+)` \|$/gm)];
    assert.strictEqual(examples.length, 3, "Cover src layout, standalone package, and Windows paths");
    for (const [, requestingManifest, destination] of examples) {
      const paths = /^[A-Z]:\//i.test(requestingManifest) ? path.win32 : path.posix;
      const packageDirectory = paths.dirname(requestingManifest);
      const expected = paths.join(paths.dirname(packageDirectory), "diagnostics").replace(/\\/g, "/");
      assert.strictEqual(destination, expected);
    }
    assert.ok(skill.includes("outside the opened workspace or inside another Git checkout"));
    assert.ok(skill.includes("get confirmation of that exact path"));
    const catalog = fs.readFileSync(path.join(path.dirname(filename), "references/common-packages.md"), "utf8");
    assert.ok(catalog.includes("peer-directory rule"));
    assert.ok(!catalog.includes("typically into `src/diagnostics`"));
  });
});