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
    
    test('Hover over fixed-size array should show array size annotation', async () => {
        // Open ComplexMessage.msg which has fixed-size arrays
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with int32[6] joint_efforts
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('int32[6] joint_efforts')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find int32[6] joint_efforts in ComplexMessage.msg');
        
        // Position cursor on the type
        const position = new vscode.Position(targetLine, 2); // Position within "int32"
        
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check for array annotation
        assert.ok(hoverText.includes('Array'), 'Hover should mention Array');
        assert.ok(hoverText.includes('Fixed size') && hoverText.includes('[6]'), 
            'Hover should show fixed array size [6]');
    });
    
    test('Hover over dynamic array should show dynamic array annotation', async () => {
        // Open ComplexMessage.msg which has dynamic arrays
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with float64[] joint_positions
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('float64[] joint_positions')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find float64[] joint_positions in ComplexMessage.msg');
        
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check for array annotation
        assert.ok(hoverText.includes('Array'), 'Hover should mention Array');
        assert.ok(hoverText.includes('Dynamic size') && hoverText.includes('[]'), 
            'Hover should show dynamic array size []');
    });
    
    test('Hover over field with inline comment should display comment', async () => {
        // Open SensorData.msg which has inline comments
        const sensorDataPath = path.join(samplesPath, 'SensorData.msg');
        const doc = await vscode.workspace.openTextDocument(sensorDataPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with temperature field and comment
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('float64 temperature') && lineText.includes('# Celsius')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find temperature field with comment in SensorData.msg');
        
        // Position cursor on the field name
        const lineText = doc.lineAt(targetLine).text;
        const fieldNameIndex = lineText.indexOf('temperature');
        const position = new vscode.Position(targetLine, fieldNameIndex + 2);
        
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check that comment is included
        assert.ok(hoverText.includes('Celsius'), 'Hover should include inline comment about Celsius');
    });
    
    test('Hover over field with default value should display default value', async () => {
        // Open BasicTypes.msg which has fields with default values
        const basicTypesPath = path.join(samplesPath, 'BasicTypes.msg');
        const doc = await vscode.workspace.openTextDocument(basicTypesPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with is_active field with default value
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('bool is_active true')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find is_active field in BasicTypes.msg');
        
        // Position cursor on the field name
        const lineText = doc.lineAt(targetLine).text;
        const fieldNameIndex = lineText.indexOf('is_active');
        const position = new vscode.Position(targetLine, fieldNameIndex + 2);
        
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check that default value is shown
        assert.ok(hoverText.includes('true'), 'Hover should show default value "true"');
        assert.ok(hoverText.includes('Default') || hoverText.includes('='), 
            'Hover should indicate this is a default value');
    });
    
    test('Hover over constant field should display constant value', async () => {
        // Open BasicTypes.msg which has constants
        const basicTypesPath = path.join(samplesPath, 'BasicTypes.msg');
        const doc = await vscode.workspace.openTextDocument(basicTypesPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with MAX_SPEED constant
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('int32 MAX_SPEED=100')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find MAX_SPEED constant in BasicTypes.msg');
        
        // Position cursor on the constant name
        const lineText = doc.lineAt(targetLine).text;
        const constantNameIndex = lineText.indexOf('MAX_SPEED');
        const position = new vscode.Position(targetLine, constantNameIndex + 2);
        
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check that constant value is shown
        assert.ok(hoverText.includes('100'), 'Hover should show constant value "100"');
        assert.ok(hoverText.includes('Constant') || hoverText.includes('='), 
            'Hover should indicate this is a constant');
    });
    
    test('Hover over custom type with package qualifier should show package info', async () => {
        // Open ComplexMessage.msg
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with geometry_msgs/Point
        let targetLine = -1;
        for (let i = 0; i < doc.lineCount; i++) {
            const lineText = doc.lineAt(i).text;
            if (lineText.includes('geometry_msgs/Point position')) {
                targetLine = i;
                break;
            }
        }
        
        assert.notStrictEqual(targetLine, -1, 'Could not find geometry_msgs/Point in ComplexMessage.msg');
        
        // Position cursor on the type
        const position = new vscode.Position(targetLine, 5); // Position within the type
        
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check that package info is shown
        assert.ok(hoverText.includes('Package'), 'Hover should show Package information');
        assert.ok(hoverText.includes('geometry_msgs'), 'Hover should show package name');
    });
    
    test('Hover over custom type without package qualifier should provide go-to-definition hint', async () => {
        // Open ComplexMessage.msg which references BasicTypes without package
        const complexMsgPath = path.join(samplesPath, 'ComplexMessage.msg');
        const doc = await vscode.workspace.openTextDocument(complexMsgPath);
        await vscode.window.showTextDocument(doc);
        
        // Find line with BasicTypes (no package qualifier)
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
        
        const markdown = contents[0] as vscode.MarkdownString;
        const hoverText = markdown.value;
        
        // Check that F12 hint is shown for custom type without package
        assert.ok(hoverText.includes('F12') || hoverText.includes('definition'), 
            'Hover should suggest using F12 to go to definition for unqualified custom types');
    });
});
