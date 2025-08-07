# Cursor Environment Detection

This extension includes functionality to detect whether it's running in Cursor (Anysphere's editor) or Visual Studio Code.

## Overview

The extension can detect the current editor environment using multiple methods:

1. **Environment Variables**: Checks for Cursor-specific environment variables
2. **Process Detection**: Examines the process name on Linux systems
3. **Extension Dependencies**: Looks for Anysphere-specific extension dependencies
4. **Workspace Settings**: Checks for Cursor-specific configuration settings
5. **Extension Context**: Analyzes the extension context properties
6. **Command Line Arguments**: Searches for Cursor-related command line arguments

## Usage

### Basic Detection

```typescript
import * as vscode_utils from './vscode-utils';

// In your extension activation or anywhere you have access to the context
const isCursor = vscode_utils.isRunningInCursor(context);
if (isCursor) {
    console.log("Running in Cursor");
} else {
    console.log("Running in VS Code");
}
```

### Command Line Interface

You can also check the environment using the command palette:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Type "ROS2: Check Cursor Environment"
3. Select the command to see which editor you're running in

### Output Channel

The extension logs the detected environment to the "ROS 2" output channel during activation. You can view this by:

1. Opening the Output panel (`View > Output`)
2. Selecting "ROS 2" from the dropdown
3. Looking for the environment detection message

## Implementation Details

The detection function uses multiple fallback methods to ensure reliable detection:

### Method 1: Environment Variables
```typescript
if (process.env.CURSOR_EXTENSION_HOST || process.env.ANYSHPERE_EXTENSION_HOST) {
    return true;
}
```

### Method 2: Process Name Detection (Linux)
```typescript
if (process.platform === 'linux') {
    const procPath = `/proc/${process.env.VSCODE_PID}/comm`;
    const processName = fs.readFileSync(procPath, 'utf8').trim();
    if (processName.includes('cursor') || processName.includes('Cursor')) {
        return true;
    }
}
```

### Method 3: Extension Dependencies
```typescript
const packageJson = require('../../package.json');
if (packageJson.extensionDependencies) {
    const hasAnysphereDeps = packageJson.extensionDependencies.some((dep: string) => 
        dep.startsWith('anysphere.')
    );
    if (hasAnysphereDeps) {
        return true;
    }
}
```

### Method 4: Workspace Settings
```typescript
const workspaceConfig = vscode.workspace.getConfiguration();
const cursorSpecificSettings = ['cursor', 'anysphere', 'cursorExtensionHost'];
for (const setting of cursorSpecificSettings) {
    if (workspaceConfig.has(setting)) {
        return true;
    }
}
```

### Method 5: Extension Context
```typescript
if (context.extensionRuntime === 'node' && context.extensionPath) {
    const extensionPath = context.extensionPath.toLowerCase();
    if (extensionPath.includes('cursor') || extensionPath.includes('anysphere')) {
        return true;
    }
}
```

### Method 6: Command Line Arguments
```typescript
if (process.argv.some(arg => arg.toLowerCase().includes('cursor'))) {
    return true;
}
```

## Use Cases

This detection can be useful for:

- **Conditional Logic**: Implementing different behaviors based on the editor
- **Debugging**: Understanding which environment the extension is running in
- **Compatibility**: Ensuring features work correctly in both VS Code and Cursor
- **User Experience**: Providing editor-specific guidance or features

## Example: Editor-Specific Features

```typescript
const isCursor = vscode_utils.isRunningInCursor(context);

if (isCursor) {
    // Cursor-specific features
    outputChannel.appendLine("Using Cursor-specific optimizations");
    // Implement Cursor-specific logic here
} else {
    // VS Code-specific features
    outputChannel.appendLine("Using VS Code-specific optimizations");
    // Implement VS Code-specific logic here
}
```

## Notes

- The detection is designed to be robust and handle cases where some methods may fail
- The function returns `false` by default (assuming VS Code) if no Cursor-specific indicators are found
- All detection methods are wrapped in try-catch blocks to prevent errors from affecting extension functionality
- The detection is performed during extension activation and logged to the output channel

## Troubleshooting

If the detection isn't working as expected:

1. Check the "ROS 2" output channel for detection messages
2. Use the "ROS2: Check Cursor Environment" command to manually test detection
3. Verify that the environment variables or process names are correctly set
4. Ensure the extension context is properly passed to the detection function
