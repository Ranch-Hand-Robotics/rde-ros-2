// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import { promises as fsPromises } from "fs";
import * as vscode from "vscode";
import { parse, ParseError, modify, applyEdits } from "jsonc-parser";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import { rosApi } from "./ros";

// Import common utilities
import { makeWorkspaceRelative } from "@ranchhandrobotics/rde-common";

// Re-export for backwards compatibility
export { makeWorkspaceRelative };

/**
 * Check if a file or directory exists.
 */
async function exists(filePath: string): Promise<boolean> {
    try {
        await fsPromises.access(filePath);
        return true;
    } catch {
        return false;
    }
}

const PYTHON_AUTOCOMPLETE_PATHS = "python.autoComplete.extraPaths";
const PYTHON_ANALYSIS_PATHS = "python.analysis.extraPaths";

/**
 * Creates missing config files and refreshes the managed ROS C++ configuration.
 */
export async function createConfigFiles() {
    const config = vscode.workspace.getConfiguration();

    // Update the Python autocomplete paths if required.
    if (config.get(PYTHON_AUTOCOMPLETE_PATHS, []).length === 0) {
        updatePythonAutoCompletePathInternal();
    }

    // Update the Python analysis paths if required.
    if (config.get(PYTHON_ANALYSIS_PATHS, []).length === 0) {
        updatePythonAnalysisPathInternal();
    }

    await updateCppPropertiesInternal(true);
}

export async function updateCppProperties(context: vscode.ExtensionContext): Promise<void> {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.UpdateCppProperties);

    await updateCppPropertiesInternal();
}

/**
 * Updates the `c_cpp_properties.json` file with ROS include paths.
 */
async function updateCppPropertiesInternal(managedOnly = false): Promise<void> {
    const workspaceRoot = vscode.workspace.rootPath;
    if (!workspaceRoot) {
        return;
    }
    const filename = path.join(workspaceRoot, ".vscode", "c_cpp_properties.json");
    let content = '{\n  "configurations": [],\n  "version": 4\n}\n';
    let existingFile = false;
    try {
        content = await fsPromises.readFile(filename, "utf8");
        existingFile = true;
    } catch (error) {
        if (error.code !== "ENOENT") {
            throw error;
        }
    }
    const errors: ParseError[] = [];
    const properties = parse(content, errors, { allowTrailingComma: true });
    if (errors.length || !properties || !Array.isArray(properties.configurations)) {
        throw new Error(`Cannot update invalid C++ configuration: ${filename}`);
    }
    const configurationIndex = properties.configurations.findIndex(configuration => configuration?.name === "ros2");
    if (managedOnly && existingFile && configurationIndex < 0) {
        return;
    }
    let includes = await rosApi.getIncludeDirs();
    const workspaceIncludes = await rosApi.getWorkspaceIncludeDirs(vscode.workspace.rootPath);
    includes = includes.concat(workspaceIncludes);

    if (process.platform === "linux") {
        includes.push(path.join("/", "usr", "include"));
    }

    // Convert paths to workspace-relative where possible
    includes = includes.map((include: string) => {
        const relativePath = makeWorkspaceRelative(include, workspaceRoot);
        // If the path was converted to relative (i.e., it's within the workspace)
        if (relativePath !== include) {
            // Handle the workspace root itself specially: path.relative() returns ""
            if (relativePath === "") {
                // Use ${workspaceFolder}/** without an extra slash
                return "${workspaceFolder}/**";
            }
            // Normalize to POSIX-style separators for VS Code config paths
            const vscodeRelativePath = relativePath.split(path.sep).join(path.posix.sep);
            // It's a workspace subdirectory path, use ${workspaceFolder} variable with POSIX join
            return path.posix.join("${workspaceFolder}", vscodeRelativePath, "**");
        }
        // Path is outside workspace, keep absolute with **
        return path.join(include, "**");
    });

    // https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/c_cpp_properties.json.md
    const cppProperties: any = {
        configurations: [
            {
                browse: {
                    databaseFilename: "${workspaceFolder}/.vscode/browse.vc.db",
                    limitSymbolsToIncludedHeaders: false,
                },
                includePath: includes,
                name: "ros2",
            },
        ],
        version: 4,
    };

    if (process.platform === "linux") {
        // set the default configurations.
        cppProperties.configurations[0].intelliSenseMode = "gcc-" + process.arch
        cppProperties.configurations[0].compilerPath = "/usr/bin/gcc"
        cppProperties.configurations[0].cStandard = "gnu11"
        cppProperties.configurations[0].cppStandard = getCppStandard()

    }

    // Ensure the ".vscode" directory exists then update the C++ path.
    const dir = path.join(vscode.workspace.rootPath, ".vscode");

    if (!await exists(dir)) {
        await fsPromises.mkdir(dir, { recursive: true });
    }

    const options = { formattingOptions: { insertSpaces: true, tabSize: 2, eol: "\n" } };
    const edits = configurationIndex < 0
        ? modify(content, ["configurations", -1], cppProperties.configurations[0], options)
        : modify(content, ["configurations", configurationIndex, "includePath"], includes, options);
    await fsPromises.writeFile(filename, applyEdits(content, edits), 'utf8');
}

export function updatePythonPath(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.UpdatePythonPath);

    updatePythonAutoCompletePathInternal();
    updatePythonAnalysisPathInternal();
}

/**
 * Updates the python autocomplete path to support ROS.
 */
function updatePythonAutoCompletePathInternal() {
    const pythonPaths = extension.env.PYTHONPATH ? extension.env.PYTHONPATH.split(path.delimiter) : [];
    const workspaceRoot = vscode.workspace.rootPath;
    
    // Convert absolute paths to relative paths where possible
    // Filter before conversion to remove empty strings from PYTHONPATH
    const relativePaths = pythonPaths
        .filter((p: string) => p.trim() !== "")
        .map((p: string) => makeWorkspaceRelative(p, workspaceRoot));
    
    vscode.workspace.getConfiguration().update(PYTHON_AUTOCOMPLETE_PATHS, relativePaths);
}

/**
 * Updates the python analysis path to support ROS.
 */
function updatePythonAnalysisPathInternal() {
    const pythonPaths = extension.env.PYTHONPATH ? extension.env.PYTHONPATH.split(path.delimiter) : [];
    const workspaceRoot = vscode.workspace.rootPath;
    
    // Convert absolute paths to relative paths where possible
    // Filter before conversion to remove empty strings from PYTHONPATH
    const relativePaths = pythonPaths
        .filter((p: string) => p.trim() !== "")
        .map((p: string) => makeWorkspaceRelative(p, workspaceRoot));
    
    vscode.workspace.getConfiguration().update(PYTHON_ANALYSIS_PATHS, relativePaths);
}

function getCppStandard() {
    switch (vscode.workspace.getConfiguration().get("ros.distro"))
    {
        case "kinetic":
        case "lunar":
            return "c++11"
        case "melodic":
        case "noetic":
        case "ardent":
        case "bouncy":
        case "crystal":
        case "dashing":
        case "eloquent":
        case "foxy":
            return "c++14"
        case "galactic":
        case "humble":
        case "iron":
        case "jazzy":
        case "kilted":
        case "rolling":
            return "c++17"
        default:
            return "c++17"
    }
}