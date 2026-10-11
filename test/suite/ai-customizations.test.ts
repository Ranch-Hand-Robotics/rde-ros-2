import * as assert from 'assert';
import * as fs from 'fs';
import * as path from 'path';
import { load } from 'js-yaml';
import { listFiles, PackageManager } from '@vscode/vsce';

interface Contribution {
  path: string;
}

interface Customization {
  file: string;
  metadata: Record<string, unknown>;
  body: string;
}

const root = path.resolve(__dirname, '../../..');
const manifest = JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8'));
const expectedAgents = [
  'ROS 2 Expert', 'ROS 2 Core', 'ROS 2 Networking', 'ROS 2 Packaging',
  'ROS 2 MoveIt', 'ROS 2 Navigation', 'ROS 2 Hardware', 'ROS 2 Drones', 'ROS 2 OS',
  'ROS 2 Test', 'ROS 2 Simulation',
];
const expectedSkills = [
  'rosdep',
  'ros2-development', 'ros2-install-troubleshooting',
  'ros2-build', 'ros2-packaging', 'ros2-actions-services-lifecycle', 'ros2-networking',
  'ros2-debugging', 'ros2-perception', 'ros2-manipulation', 'ros2-navigation',
  'ros2-test', 'ros2-gazebo', 'ros2-omniverse', 'ros2-mujoco', 'ros2-launch',
  'ros2-performance', 'ros2-pixi',
];
const commonRules = 'assets/skills/ros2-development/SKILL.md';

function readCustomization(contribution: Contribution): Customization {
  assert.strictEqual(typeof contribution.path, 'string');
  assert.ok(contribution.path.startsWith('./assets/'), contribution.path);
  const absolutePath = path.resolve(root, contribution.path);
  const relativePath = path.relative(root, absolutePath);
  assert.ok(!relativePath.startsWith('..') && !path.isAbsolute(relativePath), contribution.path);
  const text = fs.readFileSync(absolutePath, 'utf8');
  const match = /^---\r?\n([\s\S]*?)\r?\n---\r?\n([\s\S]+)$/.exec(text);
  assert.ok(match, `Missing frontmatter/body: ${contribution.path}`);
  const metadata = load(match[1]);
  assert.ok(metadata && typeof metadata === 'object' && !Array.isArray(metadata));
  return { file: relativePath.split(path.sep).join('/'), metadata: metadata as Record<string, unknown>, body: match[2] };
}

function localLinks(customization: Customization): string[] {
  const links: string[] = [];
  const pattern = /\[[^\]]+\]\(([^)]+)\)/g;
  let match: RegExpExecArray | null;
  while ((match = pattern.exec(customization.body)) !== null) {
    const target = match[1].split('#')[0];
    if (!target || /^[a-z][a-z0-9+.-]*:/i.test(target)) {
      continue;
    }
    assert.ok(target.startsWith('.'), `Expected relative asset link: ${target}`);
    const resolved = path.resolve(root, path.dirname(customization.file), target);
    const relativePath = path.relative(root, resolved);
    assert.ok(!relativePath.startsWith('..') && !path.isAbsolute(relativePath), target);
    assert.ok(fs.statSync(resolved).isFile(), `Broken link in ${customization.file}: ${target}`);
    links.push(relativePath.split(path.sep).join('/'));
  }
  return links;
}

describe('Bundled ROS 2 AI customizations', () => {
  let agents: Customization[];
  let skills: Customization[];

  before(() => {
    assert.ok(Array.isArray(manifest.contributes.chatAgents));
    assert.ok(Array.isArray(manifest.contributes.chatSkills));
    agents = manifest.contributes.chatAgents.map(readCustomization);
    skills = manifest.contributes.chatSkills.map(readCustomization);
  });

  it('registers the coordinator, all ten specialists, and all eighteen skills exactly once', () => {
    assert.deepStrictEqual(agents.map(a => a.metadata.name).sort(), [...expectedAgents].sort());
    assert.deepStrictEqual(skills.map(s => s.metadata.name).sort(), [...expectedSkills].sort());
    const paths = [...agents, ...skills].map(c => c.file);
    assert.strictEqual(new Set(paths).size, paths.length);
  });

  it('registers every agent and skill source, with no orphan assets', () => {
    const agentFiles = fs.readdirSync(path.join(root, 'assets/agents'))
      .filter(file => file.endsWith('.agent.md')).map(file => `assets/agents/${file}`);
    const skillFiles = fs.readdirSync(path.join(root, 'assets/skills'), { withFileTypes: true })
      .filter(entry => entry.isDirectory()).map(entry => `assets/skills/${entry.name}/SKILL.md`);
    assert.deepStrictEqual(agents.map(a => a.file).sort(), agentFiles.sort());
    assert.deepStrictEqual(skills.map(s => s.file).sort(), skillFiles.sort());
  });

  it('provides valid discovery metadata and substantive, model-independent workflows', () => {
    for (const customization of [...agents, ...skills]) {
      const { metadata, body, file } = customization;
      assert.strictEqual(typeof metadata.name, 'string', file);
      assert.strictEqual(typeof metadata.description, 'string', file);
      assert.ok((metadata.description as string).length > 20, file);
      assert.ok((metadata.description as string).length <= 1024, file);
      // The standalone upstream rosdep skill uses the default invocability.
      if (metadata.name !== 'rosdep') {
        assert.strictEqual(typeof metadata['user-invocable'], 'boolean', file);
      }
      assert.notStrictEqual(metadata['disable-model-invocation'], true, file);
      assert.ok(!('model' in metadata), `Do not pin a model: ${file}`);
      assert.ok(body.trim().length > 500, `Missing workflow: ${file}`);
      assert.ok(!/\bTODO\b|\bTBD\b/.test(body), `Unfinished bootstrap: ${file}`);
    }
  });

  it('uses valid slash-command skill names matching their parent directories', () => {
    for (const skill of skills) {
      const name = skill.metadata.name as string;
      assert.ok(/^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(name), name);
      assert.ok(name.length <= 64, name);
      assert.strictEqual(path.basename(path.dirname(skill.file)), name);
      assert.strictEqual(path.basename(skill.file), 'SKILL.md');
      assert.strictEqual(skill.metadata['user-invocable'] ?? true, true);
    }
  });

  it('gives the coordinator an executable delegation graph without recursive specialists', () => {
    const coordinator = agents.find(a => a.metadata.name === 'ROS 2 Expert')!;
    assert.ok(Array.isArray(coordinator.metadata.tools));
    assert.ok((coordinator.metadata.tools as string[]).includes('agent'));
    assert.ok(Array.isArray(coordinator.metadata.agents));
    assert.deepStrictEqual([...(coordinator.metadata.agents as string[])].sort(), expectedAgents.slice(1).sort());
    for (const specialist of agents.filter(a => a !== coordinator)) {
      assert.strictEqual(specialist.metadata['user-invocable'], false, specialist.file);
      assert.strictEqual(specialist.metadata['disable-model-invocation'], false);
      assert.deepStrictEqual(specialist.metadata.agents, [], specialist.file);
      assert.ok(Array.isArray(specialist.metadata.tools), specialist.file);
      assert.ok(!(specialist.metadata.tools as string[]).includes('agent'), specialist.file);
    }
    assert.deepStrictEqual(agents.filter(a => a.metadata['user-invocable']).map(a => a.metadata.name).sort(),
      ['ROS 2 Expert']);
  });

  it('links the ROS 2 agent workflows to shared guidance and resolves all relative resources', () => {
    for (const customization of [...agents, ...skills]) {
      const links = localLinks(customization);
      if (customization.file !== commonRules && customization.metadata.name !== 'rosdep') {
        assert.ok(links.includes(commonRules), customization.file);
      }
    }
    const rules = fs.readFileSync(path.join(root, commonRules), 'utf8');
    for (const requirement of ['shell', 'architecture', 'explicit approval', 'simulation', 'hardware', 'exit status']) {
      assert.ok(rules.includes(requirement), `Missing shared requirement: ${requirement}`);
    }
  });

  it('removes legacy development assets and prompt references from all agent and skill bodies', () => {
    assert.ok(!fs.existsSync(path.join(root, 'assets/prompts/ros2-development.md')));
    // main's installer still loads this prompt; retain it independently of the new skills.
    assert.ok(fs.existsSync(path.join(root, 'assets/prompts/ros-install-troubleshooting.md')));
    for (const legacyFile of ['ros2-development.md', 'ros-install-troubleshooting.md']) {
      for (const customization of [...agents, ...skills]) {
        assert.ok(!customization.body.includes(`prompts/${legacyFile}`), `Legacy prompt link in ${customization.file}`);
      }
    }
  });

  it('covers hybrid Windows/WSL USB setup, device validation, and safe cleanup', () => {
    const osAgent = agents.find(a => a.metadata.name === 'ROS 2 OS')!;
    assert.ok((osAgent.metadata.description as string).includes('Windows/WSL 2'));
    for (const requirement of [
      'Windows PowerShell', 'administrator PowerShell', 'inside Ubuntu',
      'winget install usbipd', 'winget install --interactive --exact dorssel.usbipd-win',
      'wsl --list --verbose', 'usbipd list', 'usbipd bind --busid <BUSID>',
      'usbipd attach --wsl --busid <BUSID>', 'lsusb',
      '/dev/input/', '/dev/video', '/dev/ttyUSB', '/dev/ttyACM',
      'udev', 'driver', 'explicit approval', 'unavailable to Windows',
      'usbipd detach --busid <BUSID>', 'usbipd unbind --busid <BUSID>',
    ]) {
      assert.ok(osAgent.body.includes(requirement), `Missing WSL guidance: ${requirement}`);
    }
  });

  it('makes the testing workflow reachable from orchestration, testing, build, and shared guidance', () => {
    const testingSkill = skills.find(s => s.metadata.name === 'ros2-test')!;
    const entryPoints = [
      agents.find(a => a.metadata.name === 'ROS 2 Expert')!,
      agents.find(a => a.metadata.name === 'ROS 2 Test')!,
      skills.find(s => s.metadata.name === 'ros2-build')!,
      skills.find(s => s.metadata.name === 'ros2-development')!,
    ];
    for (const entry of entryPoints) {
      assert.ok(localLinks(entry).includes(testingSkill.file), `Testing workflow unreachable from ${entry.file}`);
    }
  });

  // Validate discoverable local resources, not whether an agent follows its instructions.
  for (const { agentName, skillNames } of [
    {
      agentName: 'ROS 2 Expert',
      skillNames: ['ros2-gazebo', 'ros2-omniverse', 'ros2-mujoco', 'ros2-launch', 'ros2-performance', 'ros2-pixi'],
    },
    {
      agentName: 'ROS 2 OS',
      skillNames: ['ros2-pixi'],
    },
    {
      agentName: 'ROS 2 Packaging',
      skillNames: ['ros2-pixi'],
    },
    {
      agentName: 'ROS 2 Core',
      skillNames: ['ros2-launch', 'ros2-performance'],
    },
    {
      agentName: 'ROS 2 Simulation',
      skillNames: ['ros2-gazebo', 'ros2-omniverse', 'ros2-mujoco', 'ros2-launch', 'ros2-performance', 'ros2-test'],
    },
  ]) {
    it(`makes the required workflows reachable through local links from ${agentName}`, () => {
      const agent = agents.find(a => a.metadata.name === agentName);
      assert.ok(agent, `Missing registered agent: ${agentName}`);
      const links = localLinks(agent);
      for (const skillName of skillNames) {
        const skill = skills.find(s => s.metadata.name === skillName);
        assert.ok(skill, `Missing registered skill: ${skillName}`);
        assert.ok(links.includes(skill.file), `${skillName} workflow unreachable from ${agent.file}`);
      }
    });
  }

  it('makes Pixi setup reachable from build and packaging without replacing dependency recovery', () => {
    const pixi = skills.find(s => s.metadata.name === 'ros2-pixi');
    assert.ok(pixi, 'Missing registered Pixi workflow');
    for (const name of ['ros2-build', 'ros2-packaging']) {
      const entry = skills.find(s => s.metadata.name === name)!;
      assert.ok(localLinks(entry).includes(pixi.file), `Pixi workflow unreachable from ${entry.file}`);
    }
    const recovery = skills.find(s => s.metadata.name === 'rosdep')!;
    assert.ok(localLinks(pixi).includes(recovery.file), 'Keep focused dependency recovery reachable');
  });

  it('bundles version-aware pixi-ros guidance and exposes its discovery triggers', () => {
    const pixi = skills.find(s => s.metadata.name === 'ros2-pixi');
    assert.ok(pixi, 'Missing registered Pixi workflow');
    for (const trigger of ['Pixi', 'RoboStack', 'pixi-ros', 'pixi.toml', 'pixi.lock']) {
      assert.ok((pixi.metadata.description as string).includes(trigger), `Missing trigger: ${trigger}`);
    }
    const reference = 'assets/skills/ros2-pixi/references/pixi-ros.md';
    assert.ok(localLinks(pixi).includes(reference), 'Missing bundled pixi-ros reference');
    const guidance = fs.readFileSync(path.join(root, reference), 'utf8');
    assert.ok(guidance.includes('https://github.com/ruben-arts/pixi-ros'));
    for (const requirement of [
      'pixi-ros init --distro', '--platform', '--channel', 'README_PIXI.md',
      'NOT FOUND', 'install/setup.bash', 'pixi-build-ros',
    ]) {
      assert.ok(guidance.includes(requirement), `Missing pixi-ros guidance: ${requirement}`);
    }
    // These are documentation contracts, not a claim that instructions enforce behavior.
    for (const requirement of ['--locked', '--frozen', '--manifest-path', '--environment', 'ROS2.rosSetupScript']) {
      assert.ok(pixi.body.includes(requirement), `Missing Pixi environment guidance: ${requirement}`);
    }
  });

  it('documents all registered skills and offers Pixi invocation in the guide and walkthrough', () => {
    const guide = fs.readFileSync(path.join(root, 'docs/ai-agents-and-skills.md'), 'utf8');
    const documentedSkills = Array.from(guide.matchAll(/^\| `([a-z0-9-]+)` \|/gm), match => match[1]);
    assert.deepStrictEqual(documentedSkills.sort(), [...expectedSkills].sort());
    for (const file of ['docs/pixi.md', 'media/walkthroughs/ai-features.md']) {
      const text = fs.readFileSync(path.join(root, file), 'utf8');
      assert.ok(text.includes('/ros2-pixi'), `Missing Pixi invocation in ${file}`);
      assert.ok(text.includes('pixi-ros'), `Missing project guidance in ${file}`);
    }
  });

  it('keeps the VS Code engine baseline and lockfile synchronized', () => {
    assert.strictEqual(manifest.engines.vscode, '^1.110.0');
    const lock = JSON.parse(fs.readFileSync(path.join(root, 'package-lock.json'), 'utf8'));
    assert.strictEqual(lock.packages[''].engines.vscode, manifest.engines.vscode);
  });

  it('exposes the usage guide as a separate documentation navigation entry', () => {
    const config = load(fs.readFileSync(path.join(root, 'mkdocs.yml'), 'utf8')) as {
      nav: Array<Record<string, unknown>>;
    };
    const usage = config.nav.find(section => Array.isArray(section.Usage))!.Usage as string[];
    assert.ok(usage.includes('pixi.md'));
    assert.ok(usage.includes('dependency-recovery.md'));
    assert.ok(usage.includes('ai-agents-and-skills.md'));
    assert.ok(fs.existsSync(path.join(root, 'docs/ai-agents-and-skills.md')));
  });

  it('includes all contributed files and linked resources in the VSIX file selection', async function () {
    this.timeout(30000);
    // The extension bundles JS and excludes node_modules. No npm process or network is needed.
    const packagedFiles = new Set((await listFiles({ cwd: root, packageManager: PackageManager.None }))
      .map(file => file.replace(/\\/g, '/')));
    for (const customization of [...agents, ...skills]) {
      for (const file of [customization.file, ...localLinks(customization)]) {
        assert.ok(packagedFiles.has(file), `VSIX excludes required asset: ${file}`);
      }
    }
  });
});