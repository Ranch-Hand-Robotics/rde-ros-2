import * as assert from 'assert';
import * as fs from 'fs';
import * as os from 'os';
import * as path from 'path';
import { loadInstallTroubleshootingSkill } from '../../src/ros/install-troubleshooting-skill';

describe('Installation troubleshooting skill loader', () => {
  let temporaryRoot: string;

  beforeEach(async () => {
    temporaryRoot = await fs.promises.mkdtemp(path.join(os.tmpdir(), 'ros2-skills-'));
  });

  afterEach(async () => {
    await fs.promises.rm(temporaryRoot, { recursive: true, force: true });
  });

  async function writeSkill(name: string, content: string): Promise<void> {
    const directory = path.join(temporaryRoot, 'assets', 'skills', name);
    await fs.promises.mkdir(directory, { recursive: true });
    await fs.promises.writeFile(path.join(directory, 'SKILL.md'), content);
  }

  it('loads both packaged workflows inline without YAML metadata', async () => {
    const root = path.resolve(__dirname, '../../..');
    const guidance = await loadInstallTroubleshootingSkill(root);
    assert.ok(guidance.startsWith('# ROS 2 development ground rules'));
    assert.ok(guidance.includes('# ROS 2 Installation Troubleshooting'));
    assert.ok(guidance.includes('explicit approval'));
    assert.ok(guidance.includes('Jetson'));
    assert.ok(!guidance.includes('user-invocable:'));
    assert.ok(!guidance.includes('description:'));
  });

  for (const newline of ['\n', '\r\n']) {
    it(`supports ${newline === '\n' ? 'LF' : 'CRLF'} skill files and preserves body order`, async () => {
      await writeSkill('ros2-development', ['---', 'name: ros2-development', '---', '# Rules', ''].join(newline));
      await writeSkill('ros2-install-troubleshooting', ['---', 'name: ros2-install-troubleshooting', '---', '# Diagnose', ''].join(newline));
      assert.strictEqual(await loadInstallTroubleshootingSkill(temporaryRoot), '# Rules\n\n---\n\n# Diagnose');
    });
  }

  it('rejects missing skill assets so the caller can report a load failure', async () => {
    await assert.rejects(loadInstallTroubleshootingSkill(temporaryRoot), /ENOENT/);
  });

  for (const content of ['# Missing frontmatter', '---\nname: empty\n---\n   ']) {
    it(`rejects an invalid skill: ${content.startsWith('#') ? 'missing frontmatter' : 'empty body'}`, async () => {
      await writeSkill('ros2-development', '---\nname: ros2-development\n---\n# Rules');
      await writeSkill('ros2-install-troubleshooting', content);
      await assert.rejects(loadInstallTroubleshootingSkill(temporaryRoot), /Missing skill frontmatter or body/);
    });
  }
});