/**
 * Tests for ROS Message Providers (Hover and Definition)
 */

import * as assert from 'assert';
import * as vscode from 'vscode';
import * as path from 'path';

suite('ROS Message Providers Test Suite', () => {
    const samplesPath = path.join(__dirname, '..', '..', '..', 'samples', 'msg_interfaces', 'msg');
    
    test('Hover over nested message type should show properties', async () => {
        // Open the ComplexMessage.msg file which has nested message types
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with geometry_msgs/Point (line 8)
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('geometry_msgs/Point position')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find geometry_msgs/Point in ComplexMessage.msg');
        
        // Position cursor on the type (geometry_msgs/Point)
        const position = new vscode.Position(targetLine, 5); // Position within the type name
        
        // Execute hover provider
        const hovers = await vscode.commands.executeCommand<vscode.Hover[]>(
            'vscode.executeHoverProvider',
            doc.uri,
            position
        );
        
        assert.ok(hovers && hovers.length > 0, 'No hover information returned');
        
        const hover = hovers[0];
        const contents = hover.contents;
        assert.ok(contents.length > 0, 'Hover contents are empty');
        
        // Check that the hover contains "Properties" section
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        assert.ok(hoverText.includes('Properties'), 'Hover should contain Properties section');
        assert.ok(hoverText.includes('float64'), 'Hover should show float64 properties from Point message');
        
        // The Point message should have x, y, z properties
        assert.ok(hoverText.includes('x') || hoverText.includes('y') || hoverText.includes('z'), 
            'Hover should show x, y, or z properties from Point message');
    });
    
    test('Hover over custom message type should show properties', async () => {
        // Open the ComplexMessage.msg file
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with BasicTypes (custom message)
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('BasicTypes robot_config')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find BasicTypes in ComplexMessage.msg');
        
        // Position cursor on the type
        const position = new vscode.Position(targetLine, 2); // Position within "BasicTypes"
        
        // Execute hover provider
        const hovers = await vscode.commands.executeCommand<vscode.Hover[]>(
            'vscode.executeHoverProvider',
            doc.uri,
            position
        );
        
        assert.ok(hovers && hovers.length > 0, 'No hover information returned');
        
        const hover = hovers[0];
        const contents = hover.contents;
        assert.ok(contents.length > 0, 'Hover contents are empty');
        
        // Check that the hover contains "Properties" section
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        assert.ok(hoverText.includes('Properties'), 'Hover should contain Properties section');
        
        // BasicTypes should have various primitive types
        assert.ok(hoverText.includes('bool') || hoverText.includes('int') || hoverText.includes('float'), 
            'Hover should show properties from BasicTypes message');
    });
    
    test('Hover over built-in type should show description', async () => {
        // Open the BasicTypes.msg file
        const basicTypesPath = path.join(samplesPath, 'BasicTypes.msg');
        const doc = await vscode.workspace.openTextDocument(basicTypesPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with float64 type
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('float64 precise_position')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find float64 in BasicTypes.msg');
        
        // Position cursor on the type
        const position = new vscode.Position(targetLine, 2); // Position within "float64"
        
        // Execute hover provider
        const hovers = await vscode.commands.executeCommand<vscode.Hover[]>(
            'vscode.executeHoverProvider',
            doc.uri,
            position
        );
        
        assert.ok(hovers && hovers.length > 0, 'No hover information returned');
        
        const hover = hovers[0];
        const contents = hover.contents;
        assert.ok(contents.length > 0, 'Hover contents are empty');
        
        // Check that built-in types show description, not properties
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        assert.ok(!hoverText.includes('Properties'), 'Built-in type hover should not contain Properties section');
        assert.ok(hoverText.includes('floating point') || hoverText.includes('double precision'), 
            'Built-in type hover should show description');
    });
    
    test('Hover over field name should show type documentation and properties', async () => {
        // Open the ComplexMessage.msg file
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with geometry_msgs/Point position
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('geometry_msgs/Point position')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find geometry_msgs/Point position in ComplexMessage.msg');
        
        // Position cursor on the field name "position"
        const lineText = doc.lineAt(targetLine).text;
        const fieldNameIndex = lineText.indexOf('position');
        const position = new vscode.Position(targetLine, fieldNameIndex + 2); // Position within "position"
        
        // Execute hover provider
        const hovers = await vscode.commands.executeCommand<vscode.Hover[]>(
            'vscode.executeHoverProvider',
            doc.uri,
            position
        );
        
        assert.ok(hovers && hovers.length > 0, 'No hover information returned for field name');
        
        const hover = hovers[0];
        const contents = hover.contents;
        assert.ok(contents.length > 0, 'Hover contents are empty');
        
        // Check that field name hover includes both field info and type documentation
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        assert.ok(hoverText.includes('geometry_msgs/Point'), 'Field hover should show type name');
        assert.ok(hoverText.includes('Properties'), 'Field hover should contain Properties section from type');
        assert.ok(hoverText.includes('float64'), 'Field hover should show properties from Point message');
    });
    
    test('Hover over system-installed package message type should show properties', async () => {
        // Open the ComplexMessage.msg file which uses builtin_interfaces/Time
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with builtin_interfaces/Time or builtin_interfaces/Duration
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('builtin_interfaces/Duration')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find builtin_interfaces/Duration in ComplexMessage.msg');
        
        // Position cursor on the type
        const position = new vscode.Position(targetLine, 5); // Position within "builtin_interfaces"
        
        // Execute hover provider
        const hovers = await vscode.commands.executeCommand<vscode.Hover[]>(
            'vscode.executeHoverProvider',
            doc.uri,
            position
        );
        
        assert.ok(hovers && hovers.length > 0, 'No hover information returned for system package');
        
        const hover = hovers[0];
        const contents = hover.contents;
        assert.ok(contents.length > 0, 'Hover contents are empty');
        
        // Check that system package message shows properties
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        assert.ok(hoverText.includes('builtin_interfaces/Duration'), 'Hover should show type name');
        assert.ok(hoverText.includes('Properties'), 'Hover should contain Properties section');
        assert.ok(hoverText.includes('int32 sec') || hoverText.includes('uint32 nanosec'), 
            'Hover should show properties from Duration message (sec and nanosec)');
    });
});
