// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";
import * as fs from "fs";
import { rosApi } from "./ros";

/**
 * Provides clickable links for include statements in XML launch files
 */
export class LaunchLinkProvider implements vscode.DocumentLinkProvider {
    /**
     * Regular expression to match include file attributes in XML launch files
     * Matches patterns like: <include file="$(find-pkg-share package)/launch/file.launch.xml"/> or
     * <include ns="my_namespace" file="$(find-pkg-share package)/launch/file.launch.xml"/>
     * Handles attributes in any order
     */
    private readonly includeRegex = /<include\b[^>]*\bfile="([^"]+)"[^>]*\/?>/g;

    /**
     * Regular expression to extract package name from find-pkg-share substitution
     * Matches: $(find-pkg-share package_name)
     * Package names can contain letters, numbers, underscores, and hyphens
     * Global flag allows replacing multiple occurrences in the same path
     */
    private readonly findPkgShareRegex = /\$\(find-pkg-share\s+([a-zA-Z0-9_-]+)\s*\)/g;

    async provideDocumentLinks(
        document: vscode.TextDocument,
        token: vscode.CancellationToken
    ): Promise<vscode.DocumentLink[]> {
        const links: vscode.DocumentLink[] = [];
        const text = document.getText();
        
        // Reset regex state
        this.includeRegex.lastIndex = 0;

        let match: RegExpExecArray | null;
        while ((match = this.includeRegex.exec(text)) !== null) {
            // Check for cancellation before processing each match
            if (token.isCancellationRequested) {
                return links;
            }

            const fullMatch = match[0];
            const filePath = match[1];
            const matchIndex = match.index;

            // Find the position of the file path within the match (after file=")
            const fileAttrStart = fullMatch.indexOf('file="') + 'file="'.length;
            const startPos = document.positionAt(matchIndex + fileAttrStart);
            const endPos = document.positionAt(matchIndex + fileAttrStart + filePath.length);
            const range = new vscode.Range(startPos, endPos);

            // Try to resolve the file path
            const resolvedPath = await this.resolveFilePath(filePath, document.uri, token);
            
            if (resolvedPath) {
                const link = new vscode.DocumentLink(range, resolvedPath);
                link.tooltip = `Open ${path.basename(resolvedPath.fsPath)}`;
                links.push(link);
            }
        }

        return links;
    }

    /**
     * Resolves a file path from a launch file include statement
     * Handles $(find-pkg-share package_name) substitutions
     */
    private async resolveFilePath(
        filePath: string,
        documentUri: vscode.Uri,
        token: vscode.CancellationToken
    ): Promise<vscode.Uri | null> {
        // Check if the path contains $(find-pkg-share ...)
        // Reset regex state for global flag
        this.findPkgShareRegex.lastIndex = 0;
        const pkgShareMatch = this.findPkgShareRegex.exec(filePath);
        
        if (pkgShareMatch) {
            // Check for cancellation before expensive ROS API call
            if (token.isCancellationRequested) {
                return null;
            }

            const packageName = pkgShareMatch[1].trim();
            
            try {
                // Get the package path using ROS API
                const packages = await rosApi.getPackages();
                
                // Check for cancellation after async operation
                if (token.isCancellationRequested) {
                    return null;
                }

                if (packages[packageName]) {
                    const pkgPath = await packages[packageName]();
                    
                    // Replace all $(find-pkg-share package_name) occurrences with the actual package path
                    // Reset regex state before replace
                    this.findPkgShareRegex.lastIndex = 0;
                    const relativePath = filePath.replace(this.findPkgShareRegex, pkgPath);
                    
                    // Check if the resolved file exists
                    if (fs.existsSync(relativePath)) {
                        return vscode.Uri.file(relativePath);
                    }
                }
            } catch (error) {
                // Package not found or ROS environment not available
                return null;
            }
        } else {
            // Handle relative paths (relative to the current launch file)
            const documentDir = path.dirname(documentUri.fsPath);
            const absolutePath = path.isAbsolute(filePath)
                ? filePath
                : path.join(documentDir, filePath);
            
            if (fs.existsSync(absolutePath)) {
                return vscode.Uri.file(absolutePath);
            }
        }

        return null;
    }
}

/**
 * Registers the launch file link provider for XML launch files
 */
export function registerLaunchLinkProvider(): vscode.Disposable {
    // Register for both .launch and .launch.xml files
    const selector: vscode.DocumentSelector = [
        { language: 'xml', pattern: '**/*.launch' },
        { language: 'xml', pattern: '**/*.launch.xml' }
    ];
    
    return vscode.languages.registerDocumentLinkProvider(
        selector,
        new LaunchLinkProvider()
    );
}
