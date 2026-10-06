import { promises as fs } from "fs";
import * as path from "path";

export interface RosBuildOptions {
  cwd?: string;
  args?: string[];
  onOutput?: (message: string) => void;
}

/** Windows colcon compares prefix strings, not case/trailing-separator aliases. */
function normalized(value: string): string {
  return path.win32.normalize(value.replace(/^"|"$/g, "")).replace(/[\\/]+$/, "").toLowerCase();
}

export function isWorkspaceInstall(value: string, prefixes: string[]): boolean {
  const candidate = normalized(value);
  return prefixes.some(prefix => {
    const root = normalized(prefix);
    return candidate === root || candidate.startsWith(root + "\\");
  });
}

export function buildInstallPrefixes(workspace: string | undefined, options: RosBuildOptions): string[] {
  const cwd = options.cwd ?? workspace ?? process.cwd();
  const args = options.args ?? [];
  let installBase = "install";
  for (let i = 0; i < args.length; i++) {
    if (args[i] === "--install-base" && args[i + 1]) { installBase = args[++i]; }
    else if (args[i].startsWith("--install-base=")) { installBase = args[i].slice("--install-base=".length); }
  }
  return [...new Set([path.resolve(cwd, installBase), ...(workspace ? [path.join(workspace, "install")] : [])])];
}

/** Remove only this build's install paths, retaining external underlays and SDKs.
 * Never start with extension.env: arbitrary hook assignments cannot be undone.
 */
export function cleanBuildEnvironment(env: NodeJS.ProcessEnv, prefixes: string[]): NodeJS.ProcessEnv {
  const result: NodeJS.ProcessEnv = {};
  for (const [key, value] of Object.entries(env)) {
    if (value === undefined) { continue; }
    // Windows environment names are case insensitive. Keep one spelling.
    const existing = Object.keys(result).find(name => name.toLowerCase() === key.toLowerCase());
    if (existing) { delete result[existing]; }
    const entries = value.split(";");
    const kept = entries.filter(entry => !isWorkspaceInstall(entry, prefixes));
    const prefixList = /^(COLCON|AMENT|CMAKE)_PREFIX_PATH$/i.test(key);
    const seen = new Set<string>();
    const unique = prefixList ? kept.filter(entry => {
      const canonical = normalized(entry);
      if (!canonical || seen.has(canonical)) { return false; }
      seen.add(canonical);
      return true;
    }) : kept;
    if (unique.length) { result[key] = unique.join(";"); }
  }
  return result;
}

/** Read colcon's recorded external parents without executing the self overlay.
 * Do not evaluate arbitrary batch syntax or silently drop an unknown chain.
 */
export async function buildParentScripts(prefixes: string[]): Promise<string[]> {
  const parents = new Map<string, string>();
  for (const prefix of prefixes) {
    const filename = path.join(prefix, "setup.bat");
    let text: string;
    try { text = await fs.readFile(filename, "utf8"); }
    catch (error) {
      if ((error as NodeJS.ErrnoException).code === "ENOENT") { continue; }
      throw error;
    }
    if (!text.trim()) { continue; } // Interrupted generation, no recorded parents to replay.
    if (!text.includes("generated from colcon_core/shell/template/prefix_chain.bat.em")) {
      throw new Error(`Cannot safely read external underlays from non-colcon setup: ${filename}. Use a terminal with explicitly sourced external underlays; this custom chain needs manual review.`);
    }
    for (const line of text.split(/\r?\n/)) {
      if (!/^\s*call\s*:_colcon_prefix_chain_bat_call_script\b/i.test(line)) { continue; }
      const match = /^\s*call\s*:_colcon_prefix_chain_bat_call_script\s+"([^"]+)"\s*$/i.exec(line);
      const script = match?.[1];
      if (script && /^%{1,2}~dp0local_setup\.bat$/i.test(script)) { continue; }
      if (!script || /%/.test(script) || !path.win32.isAbsolute(script)
        || path.win32.basename(script).toLowerCase() !== "local_setup.bat") {
        throw new Error(`Cannot resolve a recorded external underlay in ${filename}: ${line.trim()}. Use a terminal with explicitly sourced external underlays rather than dropping this dependency.`);
      }
      if (!isWorkspaceInstall(script, prefixes)) { parents.set(normalized(script), script); }
    }
  }
  return [...parents.values()];
}

export function sameBuildScript(left: string, right: string): boolean {
  return normalized(left) === normalized(right);
}