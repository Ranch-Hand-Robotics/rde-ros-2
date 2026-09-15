import * as fs from "fs";
import * as path from "path";

/** Load skill bodies for clipboard-based help without relying on Chat discovery. */
export async function loadInstallTroubleshootingSkill(extensionPath: string): Promise<string> {
  const bodies = await Promise.all([
    "ros2-development",
    "ros2-install-troubleshooting",
  ].map(async (name) => {
    const skillPath = path.join(extensionPath, "assets", "skills", name, "SKILL.md");
    const markdown = await fs.promises.readFile(skillPath, "utf-8");
    const match = /^---\r?\n[\s\S]*?\r?\n---\r?\n([\s\S]+)$/.exec(markdown);
    if (!match || !match[1].trim()) {
      throw new Error(`Missing skill frontmatter or body: ${skillPath}`);
    }
    return match[1].trim();
  }));

  // Include the shared rules inline: relative asset links cannot resolve after pasting.
  return bodies.join("\n\n---\n\n");
}