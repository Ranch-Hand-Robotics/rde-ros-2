/**
 * Tests for Launch File Link Provider
 */

import * as assert from 'assert';
import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';

describe('Launch Link Provider Test Suite', () => {
    // Use __dirname to construct a more robust path to the test directory
    const workspaceRoot = path.resolve(__dirname, '../../..');
    const testLaunchDir = path.join(workspaceRoot, 'test', 'launch');
    let hasDocLinkCommand = false;

    before(async () => {
        const commands = await vscode.commands.getCommands(true);
        hasDocLinkCommand = commands.includes('vscode.executeDocumentLinkProvider');
    });
    
    it('Should provide links for include statements in XML launch files', async () => {
        if (!hasDocLinkCommand) {
            console.warn('Skipping: vscode.executeDocumentLinkProvider not available in test host');
            return;
        }
        // Create a test launch file with include statements
        const testLaunchPath = path.join(testLaunchDir, 'test_include.launch');
        
        // Create a simple target file that the include will reference
        const targetLaunchPath = path.join(testLaunchDir, 'target.launch.xml');
        const targetContent = '<launch>\n  <arg name="test" default="value"/>\n</launch>';
        fs.writeFileSync(targetLaunchPath, targetContent);
        
        // Create the test launch file with a relative include
        const testContent = `<launch>
  <include file="target.launch.xml"/>
  <arg name="another_arg" default="123"/>
</launch>`;
        
        fs.writeFileSync(testLaunchPath, testContent);
        
        try {
            // Open the test document
            const doc = await vscode.workspace.openTextDocument(testLaunchPath);
            await vscode.window.showTextDocument(doc);
            
            // Execute document link provider
            const links = await vscode.commands.executeCommand<vscode.DocumentLink[]>(
                'vscode.executeDocumentLinkProvider',
                doc.uri
            );
            
            // Verify that links are provided
            assert.ok(links, 'Document link provider should return links');
            assert.ok(links.length > 0, 'Should find at least one include link');
            
            // Check that the link points to the target file
            const link = links[0];
            assert.ok(link.target, 'Link should have a target URI');
            
            const expectedPath = vscode.Uri.file(targetLaunchPath);
            assert.strictEqual(
                link.target.fsPath,
                expectedPath.fsPath,
                'Link should point to the target launch file'
            );
            
        } finally {
            // Clean up test files
            if (fs.existsSync(testLaunchPath)) {
                fs.unlinkSync(testLaunchPath);
            }
            if (fs.existsSync(targetLaunchPath)) {
                fs.unlinkSync(targetLaunchPath);
            }
        }
    });
    
    it('Should handle multiple include statements', async () => {
        if (!hasDocLinkCommand) {
            console.warn('Skipping: vscode.executeDocumentLinkProvider not available in test host');
            return;
        }
        const testLaunchPath = path.join(testLaunchDir, 'test_multiple_includes.launch');
        
        // Create target files
        const target1Path = path.join(testLaunchDir, 'target1.launch');
        const target2Path = path.join(testLaunchDir, 'target2.launch.xml');
        
        fs.writeFileSync(target1Path, '<launch></launch>');
        fs.writeFileSync(target2Path, '<launch></launch>');
        
        const testContent = `<launch>
  <include file="target1.launch"/>
  <include file="target2.launch.xml"/>
</launch>`;
        
        fs.writeFileSync(testLaunchPath, testContent);
        
        try {
            const doc = await vscode.workspace.openTextDocument(testLaunchPath);
            await vscode.window.showTextDocument(doc);
            
            const links = await vscode.commands.executeCommand<vscode.DocumentLink[]>(
                'vscode.executeDocumentLinkProvider',
                doc.uri
            );
            
            assert.ok(links, 'Document link provider should return links');
            assert.strictEqual(links.length, 2, 'Should find two include links');
            
        } finally {
            // Clean up
            [testLaunchPath, target1Path, target2Path].forEach(p => {
                if (fs.existsSync(p)) {
                    fs.unlinkSync(p);
                }
            });
        }
    });
    
    it('Should not create links for non-existent files', async () => {
        if (!hasDocLinkCommand) {
            console.warn('Skipping: vscode.executeDocumentLinkProvider not available in test host');
            return;
        }
        const testLaunchPath = path.join(testLaunchDir, 'test_nonexistent.launch');
        
        const testContent = `<launch>
  <include file="this_file_does_not_exist.launch.xml"/>
</launch>`;
        
        fs.writeFileSync(testLaunchPath, testContent);
        
        try {
            const doc = await vscode.workspace.openTextDocument(testLaunchPath);
            await vscode.window.showTextDocument(doc);
            
            const links = await vscode.commands.executeCommand<vscode.DocumentLink[]>(
                'vscode.executeDocumentLinkProvider',
                doc.uri
            );
            
            // Links array may be empty or contain null/undefined entries
            // Filter out any invalid links
            const validLinks = links ? links.filter(link => link && link.target) : [];
            
            assert.strictEqual(
                validLinks.length,
                0,
                'Should not create links for non-existent files'
            );
            
        } finally {
            if (fs.existsSync(testLaunchPath)) {
                fs.unlinkSync(testLaunchPath);
            }
        }
    });
});
