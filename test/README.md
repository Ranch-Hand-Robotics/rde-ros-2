# ROS 2 Extension Test Files

This directory contains test files for the ROS 2 VS Code extension.

## Directory Structure

- `test/` - Main test directory
  - `launch/` - Launch file tests for the dumper functionality
  - `test_launch_dumper.py` - Python test script for the launch dumper

## Launch Test Files

The `test/launch/` directory contains various ROS 2 launch files used to test the extension's launch file dumper functionality:

- **`simple_lifecycle_launch.py`** - Basic lifecycle node launch file
- **`test_basic_lifecycle_launch.py`** - Basic lifecycle node test case  
- **`test_events_launch.py`** - Launch file with event handlers and emitters
- **`test_launch.py`** - Standard ROS nodes (talker/listener demo)
- **`test_lifecycle_launch.py`** - Lifecycle node with event transitions
- **`test_lifecycle_simple.py`** - Simple lifecycle node test
- **`test_mixed_nodes.launch.py`** - Mixed regular and lifecycle nodes

## Running Tests

### Image and point-cloud transport

Run the standalone routing, binary framing, rate-control, and watcher regressions:

```bash
npm run test:topics
```

With the ROS Python environment activated, test the native `sensor_msgs` producers:

```bash
python -m unittest discover -s test -p "test_*subscriber.py"
```

The VS Code test suite also covers point-cloud decoding, panel controls, and an
actual binary `postMessage` round trip into a webview.

### Launch dumper

To test the launch dumper with these files:

```bash
# Test with a basic launch file
python3 assets/scripts/ros2_launch_dumper.py test/launch/test_launch.py --output-format json

# Test with a lifecycle node launch file  
python3 assets/scripts/ros2_launch_dumper.py test/launch/simple_lifecycle_launch.py --output-format json

# Test with mixed node types
python3 assets/scripts/ros2_launch_dumper.py test/launch/test_mixed_nodes.launch.py --output-format json
```

## Extension Debugger Startup

On the macOS VS Code build using Node 24.18.1 and js-debug 1.117.0, extension-host
debugging can abort before activation in
`node::inspector::Agent::ToggleNetworkTracking`. Set
`"debug.javascript.enableNetworkView": false` in local workspace settings to
avoid that inspector request. Normal breakpoints and stepping remain enabled.
Do not commit local `.vscode/settings.json`.

The opt-in regression exercises an attached debugger, empty-window ROS activation,
Start Daemon, Show Status, and terminal creation. It requires an installed ROS
environment and retains the debugger trace in the printed temporary directory.
Run it in the isolated test profile, not your normal development window:

```bash
npm run test-compile
RDE_TEST_DEBUG_LAUNCH=1 RDE_TEST_ROS_DAEMON_SETUP="$HOME/pixi_ws/lyrical/setup.bash" \
node -e 'const path = require("path"); require("@vscode/test-electron").runTests({ vscodeExecutablePath: "/Applications/Visual Studio Code.app/Contents/MacOS/Code", extensionDevelopmentPath: process.cwd(), extensionTestsPath: path.resolve("out/test/emptyWindow"), launchArgs: ["--new-window", "--disable-extensions", "--disable-workspace-trust"] }).catch(error => { console.error(error); process.exitCode = 1; });'
```

Set `RDE_TEST_DEBUG_NETWORK_VIEW=1` as well to reproduce the unmitigated startup
path; the affected runtime can abort intermittently.

## Test Coverage

These launch files test:

- **Regular ROS nodes** (ExecuteProcess actions)
- **Lifecycle nodes** (LifecycleNode actions) 
- **Event handlers and emitters** (OnProcessStart, ChangeState)
- **Mixed node types** in a single launch file
- **Cross-platform compatibility** (Windows and Linux paths)
- **JSON and legacy output formats**
