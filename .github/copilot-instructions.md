# Project Overview
This is the **Robot Developer Extensions for ROS 2** - a Visual Studio Code extension that provides debugging and development support for Robot Operating System 2 (ROS 2). The extension works with Visual Studio Code and Cursor.

## Technology Stack
* **Language**: TypeScript + Python
* **Framework**: VS Code Extension API
* **Build Tool**: Webpack + npm
* **Testing**: Mocha (via VS Code test runner)
* **Target ROS**: ROS 2 Humble or greater (no ROS 1 support)
* **Supported Languages**: rclpy (Python), rclcpp (C++), rclrust (Rust), rcldonet (.NET)

# Contribution Guidelines

## Building and Testing
1. **Install dependencies**: `npm ci` (preferred over `npm install`)
2. **Build the project**: `npm run build` (runs webpack in production mode to create distributable package)
3. **Lint code**: Focus on passing the build; `npm run lint` exists but may need ESLint configuration
4. **Run tests**: Open Debug viewlet (`Ctrl+Shift+D`), select `Tests`, then hit `F5`
5. **Debug extension**: Open Debug viewlet, select `Extension`, then hit `F5`

## Code Quality Requirements
* All new features and bug fixes **must** include test cases in the `test` directory
* Changes must pass linting and build successfully
* Require/import statements must always be at the top of a file, never in the middle
* Follow `.editorconfig` settings: 2 spaces, LF line endings, insert final newline
* Be concise and to the point in code and documentation

## Acceptance Criteria for Changes
* Code must build without errors: `npm run build`
* Existing tests must continue to pass
* New functionality must include tests
* No introduction of security vulnerabilities
* Changes must not break ROS 2 environment detection or debugging capabilities

# Directory Structure
```
├── src/              # TypeScript source code
│   ├── debugger/     # Debugging functionality (attach, launch, process picker)
│   ├── ros/          # ROS-specific implementations (ROS 2 commands, lifecycle nodes)
│   ├── build-tool/   # Build tool integration (colcon)
│   └── test-provider/ # Test discovery and execution
├── test/             # Test suites and fixtures
│   ├── suite/        # Test implementations
│   └── launch/       # ROS launch file test fixtures
├── assets/           # Scripts and resources bundled with extension
├── scripts/          # Build and maintenance scripts
├── samples/          # Sample ROS 2 workspaces for testing
├── dist/             # Compiled output (do not commit)
└── docs/             # Documentation source files
```

# Code Style and Conventions

## TypeScript Style
```typescript
// Good: Use async/await for asynchronous operations
async function fetchRosNodes(): Promise<string[]> {
    const nodes = await ros2.getNodeList();
    return nodes;
}

// Good: Proper error handling
try {
    await debugManager.launchNode(config);
} catch (error) {
    vscode.window.showErrorMessage(`Failed to launch: ${error}`);
    throw error;
}

// Good: Use typed interfaces
interface LaunchConfig {
    target: string;
    arguments: string[];
    env?: Record<string, string>;
}
```

## Python Style
* Python code is used for interfacing directly with ROS 2
* Python code runs in a managed virtual environment in the extension directory
* Follow Python best practices for ROS 2 integration

# Security and Safety Boundaries

## Forbidden Actions
* **Never** commit secrets, API keys, or credentials to the repository
* **Never** remove or disable security-related code without explicit justification
* **Do not** create summary documents or planning files (work in memory)
* **Do not** use vcpkg for dependency management (use Pixi instead)

## Protected Areas
* `.venv/` directories contain Python virtual environments (git-ignored)
* `node_modules/` contains npm dependencies (git-ignored)
* Build artifacts in `dist/` and `out/` directories (git-ignored)
* Do not commit `.vscode/settings.json` or `.vscode/c_cpp_properties.json`

# Special Considerations

## ROS 2 Environment Setup
* When executing ROS commands, source the ROS 2 setup script first:
  * **Linux**: `/opt/ros/kilted/setup.bash` (we test on ROS 2 Kilted)
  * **Windows Pixi**: `c:\pixi_ws\ros2-windows\local_setup.bat` (configurable via `ROS2.pixiRoot` setting)
* The extension automatically detects and configures ROS environments

## ROS 2 Launch File Dumper
* `ros2_launch_dumper.py` is a critical component in `assets/scripts/`
* It analyzes Python-based launch files to enable per-node debugging
* The dumper iterates over launch file objects and outputs structured data parsed by the extension
* **Do not modify** without testing with multiple launch file configurations

## ROS 2 Lifecycle Node Support
The extension provides comprehensive lifecycle node management:
* Set initial state and transition lifecycle nodes between states
* View current state and available transitions
* During debugging, launch files with lifecycle nodes start in "unconfigured" state
* Transitions managed through command palette and ROS 2 Status Webview
* Launch file event emitters are exported and handled after debugger startup

## Legacy Code Warning
* Some ROS 1 code may still exist in the codebase
* **ROS 1 is not supported** - such code can be removed if encountered
* Do not be confused by ROS 1 references; focus on ROS 2 implementation

# Getting Clarification
If a command or requirement is unclear or would result in numerous changes, **ask for clarification before proceeding**.
