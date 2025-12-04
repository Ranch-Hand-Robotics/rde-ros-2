// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import { rosApi } from "./ros";

/**
 * ROS 2 built-in message types with their descriptions
 */
const BUILTIN_TYPES: { [key: string]: string } = {
    "bool": "Boolean value (true/false)",
    "byte": "8-bit unsigned integer (0-255)",
    "char": "8-bit signed integer (-128 to 127)",
    "int8": "8-bit signed integer (-128 to 127)",
    "uint8": "8-bit unsigned integer (0-255)",
    "int16": "16-bit signed integer (-32,768 to 32,767)",
    "uint16": "16-bit unsigned integer (0-65,535)",
    "int32": "32-bit signed integer (-2,147,483,648 to 2,147,483,647)",
    "uint32": "32-bit unsigned integer (0-4,294,967,295)",
    "int64": "64-bit signed integer",
    "uint64": "64-bit unsigned integer",
    "float32": "32-bit floating point number",
    "float64": "64-bit floating point number (double precision)",
    "string": "UTF-8 encoded string",
    "wstring": "Wide string (UTF-16)",
    "time": "ROS time with seconds and nanoseconds",
    "duration": "ROS duration with seconds and nanoseconds"
};

/**
 * Common ROS 2 interface packages and their descriptions
 */
const COMMON_PACKAGES: { [key: string]: string } = {
    "std_msgs": "Standard ROS message types",
    "geometry_msgs": "Geometric primitive messages for representing common geometric shapes",
    "sensor_msgs": "Sensor data messages",
    "nav_msgs": "Navigation messages",
    "builtin_interfaces": "Built-in ROS 2 interface types (Time, Duration)",
    "action_msgs": "Action communication messages",
    "diagnostic_msgs": "Diagnostic messages for system health monitoring",
    "trajectory_msgs": "Trajectory representation messages",
    "visualization_msgs": "Visualization markers and displays"
};

interface MessageField {
    type: string;
    name: string;
    arraySize?: string;
    defaultValue?: string;
    isConstant: boolean;
    comment?: string;
    line: number;
    column: number;
}

interface ParsedMessage {
    fields: MessageField[];
    comments: Map<number, string>;
}

/**
 * Parses a ROS message or service file
 */
function parseMessageFile(document: vscode.TextDocument): ParsedMessage {
    const fields: MessageField[] = [];
    const comments = new Map<number, string>();
    const text = document.getText();
    const lines = text.split('\n');

    for (let i = 0; i < lines.length; i++) {
        const line = lines[i];
        const lineNumber = i;

        // Extract comments
        const commentMatch = line.match(/#\s*(.+)$/);
        if (commentMatch) {
            comments.set(lineNumber, commentMatch[1].trim());
        }

        // Skip empty lines, comment-only lines, and separators
        if (line.trim() === '' || line.trim().startsWith('#') || line.trim().match(/^-{3,}$/)) {
            continue;
        }

        // Match field definitions: type[array] name [= value] [# comment]
        const fieldMatch = line.match(/^\s*(@optional\s+)?([a-zA-Z0-9_/]+)(\[[^\]]*\])?\s+([a-zA-Z0-9_]+)(\s*=\s*(.+?))?(\s*#.*)?$/);
        
        if (fieldMatch) {
            const isOptional = !!fieldMatch[1];
            const typeStr = fieldMatch[2];
            const arraySize = fieldMatch[3]?.replace(/[\[\]]/g, '') || undefined;
            const fieldName = fieldMatch[4];
            const defaultValue = fieldMatch[6]?.trim();
            const isConstant = !!defaultValue && line.includes('=');
            
            // Find the column position of the type
            const column = line.indexOf(typeStr);

            fields.push({
                type: typeStr,
                name: fieldName,
                arraySize,
                defaultValue,
                isConstant,
                line: lineNumber,
                column
            });
        }
    }

    return { fields, comments };
}

/**
 * Checks if a type is a built-in ROS type
 */
function isBuiltinType(type: string): boolean {
    return type in BUILTIN_TYPES;
}

/**
 * Extracts package and message name from a qualified type (e.g., "geometry_msgs/Point")
 */
function parseQualifiedType(type: string): { package?: string; message: string } {
    const parts = type.split('/');
    if (parts.length === 2) {
        return { package: parts[0], message: parts[1] };
    }
    return { message: type };
}

/**
 * Finds message definition files in the workspace
 */
async function findMessageDefinitions(
    workspaceFolders: readonly vscode.WorkspaceFolder[],
    packageName?: string,
    messageName?: string
): Promise<vscode.Uri[]> {
    const results: vscode.Uri[] = [];
    
    for (const folder of workspaceFolders) {
        // Search patterns for message files
        const patterns = [
            new vscode.RelativePattern(folder, '**/*.msg'),
            new vscode.RelativePattern(folder, '**/*.srv'),
            new vscode.RelativePattern(folder, '**/*.action')
        ];

        for (const pattern of patterns) {
            const files = await vscode.workspace.findFiles(pattern, '{**/build/**,**/install/**,**/log/**,**/node_modules/**}', 1000);
            
            for (const file of files) {
                const fileName = path.basename(file.fsPath, path.extname(file.fsPath));
                
                // If we're looking for a specific message, check if it matches
                if (messageName && fileName !== messageName) {
                    continue;
                }

                // If we're looking for a specific package, check if the file is in that package
                if (packageName) {
                    try {
                        const packages = await rosApi.getPackages();
                        if (packages[packageName]) {
                            const pkgPath = await packages[packageName]();
                            // Check if the file is under the package path
                            if (!file.fsPath.startsWith(pkgPath)) {
                                continue;
                            }
                        } else {
                            // Package not found, skip this file
                            continue;
                        }
                    } catch {
                        // If we can't get packages, fall back to simple name matching
                        // This might happen if ROS environment is not fully sourced
                    }
                }

                results.push(file);
            }
        }
    }

    return results;
}

/**
 * Definition provider for ROS message files
 */
export class RosMessageDefinitionProvider implements vscode.DefinitionProvider {
    async provideDefinition(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken
    ): Promise<vscode.Definition | undefined> {
        const wordRange = document.getWordRangeAtPosition(position);
        if (!wordRange) {
            return undefined;
        }

        const line = document.lineAt(position.line).text;
        const parsed = parseMessageFile(document);

        // Find the field at the current position
        const field = parsed.fields.find(f => f.line === position.line);
        if (!field) {
            return undefined;
        }

        // Check if the cursor is on the type part of the field
        const typeStartCol = field.column;
        const typeEndCol = typeStartCol + field.type.length;
        
        if (position.character < typeStartCol || position.character > typeEndCol) {
            return undefined;
        }

        // Don't provide definitions for built-in types
        if (isBuiltinType(field.type)) {
            return undefined;
        }

        // Parse the qualified type
        const { package: pkgName, message: msgName } = parseQualifiedType(field.type);

        // Find the message definition
        const workspaceFolders = vscode.workspace.workspaceFolders;
        if (!workspaceFolders) {
            return undefined;
        }

        const definitions = await findMessageDefinitions(workspaceFolders, pkgName, msgName);
        
        if (definitions.length > 0) {
            // Return the first match (or all matches for "Go to References" style)
            return definitions.map(uri => new vscode.Location(uri, new vscode.Position(0, 0)));
        }

        return undefined;
    }
}

/**
 * Hover provider for ROS message files
 */
export class RosMessageHoverProvider implements vscode.HoverProvider {
    async provideHover(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken
    ): Promise<vscode.Hover | undefined> {
        const wordRange = document.getWordRangeAtPosition(position);
        if (!wordRange) {
            return undefined;
        }

        const word = document.getText(wordRange);
        const line = document.lineAt(position.line).text;
        const parsed = parseMessageFile(document);

        // Check if we're hovering over a type
        const field = parsed.fields.find(f => f.line === position.line);
        if (!field) {
            return undefined;
        }

        // Check if the cursor is on the type part of the field
        const typeStartCol = field.column;
        const typeEndCol = typeStartCol + field.type.length;
        
        if (position.character >= typeStartCol && position.character <= typeEndCol) {
            // Hovering over a type
            if (isBuiltinType(field.type)) {
                // Built-in type
                const description = BUILTIN_TYPES[field.type];
                const markdown = new vscode.MarkdownString();
                markdown.appendCodeblock(field.type, 'rosmsg');
                markdown.appendMarkdown(`\n${description}`);
                
                if (field.arraySize) {
                    markdown.appendMarkdown(`\n\n**Array**: ${field.arraySize ? `Fixed size [${field.arraySize}]` : 'Dynamic size []'}`);
                }
                
                return new vscode.Hover(markdown, wordRange);
            } else {
                // Custom type - show package and message info
                const { package: pkgName, message: msgName } = parseQualifiedType(field.type);
                const markdown = new vscode.MarkdownString();
                
                if (pkgName) {
                    markdown.appendCodeblock(`${pkgName}/${msgName}`, 'rosmsg');
                    
                    // Add package description if it's a known package
                    if (pkgName in COMMON_PACKAGES) {
                        markdown.appendMarkdown(`\n**Package**: ${COMMON_PACKAGES[pkgName]}`);
                    } else {
                        markdown.appendMarkdown(`\n**Package**: ${pkgName}`);
                    }
                    markdown.appendMarkdown(`\n\n**Message Type**: ${msgName}`);
                } else {
                    markdown.appendCodeblock(msgName, 'rosmsg');
                    markdown.appendMarkdown(`\n**Custom Message Type**`);
                    markdown.appendMarkdown(`\n\nPress **F12** to go to definition`);
                }
                
                if (field.arraySize) {
                    markdown.appendMarkdown(`\n\n**Array**: ${field.arraySize ? `Fixed size [${field.arraySize}]` : 'Dynamic size []'}`);
                }
                
                // Try to find and show the definition preview
                const workspaceFolders = vscode.workspace.workspaceFolders;
                if (workspaceFolders) {
                    const definitions = await findMessageDefinitions(workspaceFolders, pkgName, msgName);
                    if (definitions.length > 0) {
                        const defUri = definitions[0];
                        try {
                            const defDoc = await vscode.workspace.openTextDocument(defUri);
                            const preview = defDoc.getText().split('\n').slice(0, 10).join('\n');
                            if (preview) {
                                markdown.appendMarkdown('\n\n**Preview**:');
                                markdown.appendCodeblock(preview.trim(), 'rosmsg');
                            }
                        } catch {
                            // Ignore errors reading the definition
                        }
                    }
                }
                
                return new vscode.Hover(markdown, new vscode.Range(
                    position.line,
                    typeStartCol,
                    position.line,
                    typeEndCol
                ));
            }
        }

        // Check if we're hovering over a field name
        const fieldNameMatch = line.match(/\s+([a-zA-Z0-9_]+)(\s*=|\s*#|\s*$)/);
        if (fieldNameMatch && word === fieldNameMatch[1]) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`${field.type} ${field.name}`, 'rosmsg');
            
            if (field.isConstant && field.defaultValue) {
                markdown.appendMarkdown(`\n**Constant** with value: \`${field.defaultValue}\``);
            } else if (field.defaultValue) {
                markdown.appendMarkdown(`\n**Default value**: \`${field.defaultValue}\``);
            }
            
            // Show comment if available
            const comment = parsed.comments.get(field.line);
            if (comment) {
                markdown.appendMarkdown(`\n\n${comment}`);
            }
            
            return new vscode.Hover(markdown, wordRange);
        }

        return undefined;
    }
}

/**
 * Registers the ROS message language providers
 */
export function registerRosMessageProviders(context: vscode.ExtensionContext): vscode.Disposable[] {
    const selector: vscode.DocumentSelector = { language: 'rosmsg', scheme: 'file' };
    
    const definitionProvider = vscode.languages.registerDefinitionProvider(
        selector,
        new RosMessageDefinitionProvider()
    );
    
    const hoverProvider = vscode.languages.registerHoverProvider(
        selector,
        new RosMessageHoverProvider()
    );
    
    return [definitionProvider, hoverProvider];
}
