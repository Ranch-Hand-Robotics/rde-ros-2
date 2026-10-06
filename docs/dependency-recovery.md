# Resolve missing dependencies with AI

The extension bundles a **rosdep** agent skill for cases where rosdep cannot
resolve a dependency or the needed binary is unavailable in Pixi/RoboStack.
It includes guidance for NVIDIA Jetson, Windows, and macOS.

In a VS Code/Copilot version that supports extension-contributed skills, type
`/rosdep` in Chat and paste the dependency error. The agent infers the requesting
package, ROS distro, and target platform from the workspace, asking only for
missing essentials. It can also select the skill for relevant dependency questions.
Discovery does not guarantee automatic invocation; use the slash command when
you specifically want this workflow. No MCP server is required.

Example requests:

> /rosdep This package fails to find a dependency on Windows with Pixi.
> Check available binaries and propose compatible source repositories before
> changing anything.

> /rosdep Help resolve this rosdep key on my Jetson. Inspect the JetPack and
> ROS versions first, and do not change CUDA or system packages.

> /rosdep This dependency is missing from RoboStack on Apple Silicon.
> Find its canonical upstream and a compatible revision, then ask before cloning.

## What the skill does

1. Identifies the missing dependency, requesting package, and target platform.
2. Queries the appropriate package index. If a compatible binary exists, offers
	to install it in the selected environment and stops the source search.
3. If no compatible binary is found, identifies the upstream repository and a
	distro-appropriate revision containing the package.
4. Offers to clone into a **peer directory of the requesting package**.
5. After approval, clones and verifies the checkout, then reports its location.

For example, a failure in `/ws/src/camera/package.xml` produces a proposed
`/ws/src/diagnostics` checkout. A failure in `/ws/camera/package.xml` produces
`/ws/diagnostics`; the skill does not create `src/` inside the camera package.
Existing destinations are never overwritten. Peer locations outside the open
workspace or inside another repository are called out for confirmation.

The default response is the binary result, source/ref if needed, and a short
"Clone into this directory?" question. No environment audit, recursive dependency
survey, `.repos` file, or build is required to reach that offer. Build/install
follow-ups happen only when requested. Failed index queries are reported as
unknown availability, not as proof that binaries are missing.

## Scope and limitations

This is AI guidance, not a deterministic dependency resolver, new package manager,
or guarantee of platform support. It does not change the existing **ROS2: Install
ROS Dependencies for this workspace using rosdep** command, intercept failures,
or automatically clone repositories. Normal agent tool approvals still apply.

## Seeded source recipes

The skill includes a common-package catalog, starting with `diagnostic_updater`,
`diagnostic_aggregator`, and `self_test` from `ros/diagnostics`. It supplies
distro-specific branch candidates, dependency checks, and targeted-build guidance.
It also flags conflicting metadata, such as Lyrical's recorded source ref
pointing at the branch upstream identifies as Rolling.

> /rosdep diagnostic_updater is missing from my Pixi environment. Use the seeded
> diagnostics recipe, check binaries for my distro and platform, and propose a
> pinned source checkout only if needed.

These are fallback recipes, not a fixed list to clone or a claim that RoboStack
never provides those packages. Binary availability and source compatibility are
rechecked before use. The catalog does not bypass approval or guarantee a build;
its initial entries have verified source metadata, not platform build results.

## Other editors

Older VS Code versions and other editors may not expose `chatSkills`. They can
still use the extension's normal ROS features. To use this guidance in another
skills-capable client, copy the complete `assets/skills/rosdep` directory from
the extension into that client's supported skills location (for example,
`.github/skills/rosdep` in a workspace). Review it first and avoid maintaining
duplicate copies under the same skill name.