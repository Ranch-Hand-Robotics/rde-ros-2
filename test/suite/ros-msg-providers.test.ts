/**
 * Tests for ROS Message Providers (Hover and Definition)
 */

import * as assert from 'assert';
import * as vscode from 'vscode';
import * as path from 'path';
import { parseMessageFile, MessageField, ParsedMessage } from '../../src/ros/ros-msg-providers';

/**
 * Helper function to create a mock TextDocument for testing parseMessageFile
 */
function createMockDocument(content: string): vscode.TextDocument {
    return {
        getText: () => content,
        uri: vscode.Uri.file('/test.msg'),
        fileName: '/test.msg',
        languageId: 'rosmsg',
        version: 1,
        isDirty: false,
        isClosed: false,
        isUntitled: false,
        eol: vscode.EndOfLine.LF,
        lineCount: content.split('\n').length,
        encoding: undefined,
        save: () => Promise.resolve(true),
        lineAt: (lineOrPosition: number | vscode.Position) => {
            const lines = content.split('\n');
            const lineNum = typeof lineOrPosition === 'number' ? lineOrPosition : lineOrPosition.line;
            // Bounds checking
            const validLineNum = Math.max(0, Math.min(lineNum, lines.length - 1));
            const text = lines[validLineNum] || '';
            return {
                lineNumber: validLineNum,
                text,
                range: new vscode.Range(validLineNum, 0, validLineNum, text.length),
                rangeIncludingLineBreak: new vscode.Range(validLineNum, 0, validLineNum + 1, 0),
                firstNonWhitespaceCharacterIndex: text.search(/\S/),
                isEmptyOrWhitespace: text.trim().length === 0
            };
        },
        offsetAt: (position: vscode.Position) => {
            const lines = content.split('\n');
            let offset = 0;
            for (let i = 0; i < position.line && i < lines.length; i++) {
                offset += lines[i].length + 1; // +1 for newline
            }
            return offset + position.character;
        },
        positionAt: (offset: number) => {
            const lines = content.split('\n');
            let currentOffset = 0;
            for (let i = 0; i < lines.length; i++) {
                if (currentOffset + lines[i].length >= offset) {
                    return new vscode.Position(i, offset - currentOffset);
                }
                currentOffset += lines[i].length + 1; // +1 for newline
            }
            return new vscode.Position(lines.length - 1, lines[lines.length - 1].length);
        },
        getWordRangeAtPosition: () => undefined,
        validateRange: (range: vscode.Range) => range,
        validatePosition: (position: vscode.Position) => position
    } as vscode.TextDocument;
}

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

  suite('parseMessageFile Unit Tests', () => {
    test('Should parse basic field definitions with primitive types', () => {
        const content = `int32 counter
float64 temperature
string robot_name
bool is_active`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 4, 'Should parse 4 fields');
        
        // Check int32 field
        const counterField = result.fields[0];
        assert.strictEqual(counterField.type, 'int32');
        assert.strictEqual(counterField.name, 'counter');
        assert.strictEqual(counterField.isConstant, false);
        assert.strictEqual(counterField.arraySize, undefined);
        assert.strictEqual(counterField.line, 0);
        
        // Check float64 field
        const tempField = result.fields[1];
        assert.strictEqual(tempField.type, 'float64');
        assert.strictEqual(tempField.name, 'temperature');
        
        // Check string field
        const nameField = result.fields[2];
        assert.strictEqual(nameField.type, 'string');
        assert.strictEqual(nameField.name, 'robot_name');
        
        // Check bool field
        const activeField = result.fields[3];
        assert.strictEqual(activeField.type, 'bool');
        assert.strictEqual(activeField.name, 'is_active');
    });
    
    test('Should handle unbounded array syntax []', () => {
        const content = `float64[] joint_positions
string[] error_messages
geometry_msgs/Point[] waypoints`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 array fields');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'float64');
        assert.strictEqual(field1.name, 'joint_positions');
        assert.strictEqual(field1.arraySize, '', 'Unbounded array should have empty string as arraySize');
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'string');
        assert.strictEqual(field2.name, 'error_messages');
        assert.strictEqual(field2.arraySize, '');
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'geometry_msgs/Point');
        assert.strictEqual(field3.name, 'waypoints');
        assert.strictEqual(field3.arraySize, '');
    });
    
    test('Should handle fixed-size array syntax [N]', () => {
        const content = `uint8[10] sensor_array
float32[9] rotation_matrix
int32[6] joint_efforts`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 fixed-size array fields');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'uint8');
        assert.strictEqual(field1.name, 'sensor_array');
        assert.strictEqual(field1.arraySize, '10');
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'float32');
        assert.strictEqual(field2.name, 'rotation_matrix');
        assert.strictEqual(field2.arraySize, '9');
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'int32');
        assert.strictEqual(field3.name, 'joint_efforts');
        assert.strictEqual(field3.arraySize, '6');
    });
    
    test('Should handle bounded array syntax [<=N]', () => {
        const content = `uint8[<=10] status_codes
string[<=5] recent_messages
geometry_msgs/Pose[<=100] path`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 bounded array fields');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'uint8');
        assert.strictEqual(field1.name, 'status_codes');
        assert.strictEqual(field1.arraySize, '<=10');
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'string');
        assert.strictEqual(field2.name, 'recent_messages');
        assert.strictEqual(field2.arraySize, '<=5');
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'geometry_msgs/Pose');
        assert.strictEqual(field3.name, 'path');
        assert.strictEqual(field3.arraySize, '<=100');
    });
    
    test('Should extract inline comments', () => {
        const content = `# Header comment
int32 counter  # This is a counter
float64 temperature # Temperature in Celsius
string robot_name   # Name of the robot
# Another standalone comment
bool is_active      # Active status`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 4, 'Should parse 4 fields');
        assert.strictEqual(result.comments.size, 6, 'Should extract 6 comments');
        
        // Check that comments are extracted with correct line numbers
        assert.strictEqual(result.comments.get(0), 'Header comment');
        assert.strictEqual(result.comments.get(1), 'This is a counter');
        assert.strictEqual(result.comments.get(2), 'Temperature in Celsius');
        assert.strictEqual(result.comments.get(3), 'Name of the robot');
        assert.strictEqual(result.comments.get(4), 'Another standalone comment');
        assert.strictEqual(result.comments.get(5), 'Active status');
    });
    
    test('Should handle constants with default values', () => {
        const content = `int32 MAX_SPEED=100
float64 PI=3.141592653589793
string DEFAULT_MODE="autonomous"
bool SAFETY_ENABLED=true
uint8 STATUS_OK=0`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 5, 'Should parse 5 constant fields');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'int32');
        assert.strictEqual(field1.name, 'MAX_SPEED');
        assert.strictEqual(field1.defaultValue, '100');
        assert.strictEqual(field1.isConstant, true);
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'float64');
        assert.strictEqual(field2.name, 'PI');
        assert.strictEqual(field2.defaultValue, '3.141592653589793');
        assert.strictEqual(field2.isConstant, true);
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'string');
        assert.strictEqual(field3.name, 'DEFAULT_MODE');
        assert.strictEqual(field3.defaultValue, '"autonomous"');
        assert.strictEqual(field3.isConstant, true);
        
        const field4 = result.fields[3];
        assert.strictEqual(field4.type, 'bool');
        assert.strictEqual(field4.name, 'SAFETY_ENABLED');
        assert.strictEqual(field4.defaultValue, 'true');
        assert.strictEqual(field4.isConstant, true);
        
        const field5 = result.fields[4];
        assert.strictEqual(field5.type, 'uint8');
        assert.strictEqual(field5.name, 'STATUS_OK');
        assert.strictEqual(field5.defaultValue, '0');
        assert.strictEqual(field5.isConstant, true);
    });
    
    test('Should handle constants with spaces around equals', () => {
        const content = `bool is_active = true
int8 temperature_celsius = -25
uint8 battery_percentage = 85
float32 velocity = 3.14159
string robot_name = "TestBot"`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 5, 'Should parse 5 fields with default values');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'bool');
        assert.strictEqual(field1.name, 'is_active');
        assert.strictEqual(field1.defaultValue, 'true');
        assert.strictEqual(field1.isConstant, true);
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'int8');
        assert.strictEqual(field2.name, 'temperature_celsius');
        assert.strictEqual(field2.defaultValue, '-25');
        assert.strictEqual(field2.isConstant, true);
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'uint8');
        assert.strictEqual(field3.name, 'battery_percentage');
        assert.strictEqual(field3.defaultValue, '85');
        assert.strictEqual(field3.isConstant, true);
    });
    
    test('Should skip empty lines', () => {
        const content = `int32 counter

float64 temperature

string robot_name`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 fields, skipping empty lines');
        assert.strictEqual(result.fields[0].line, 0);
        assert.strictEqual(result.fields[1].line, 2);
        assert.strictEqual(result.fields[2].line, 4);
    });
    
    test('Should skip comment-only lines', () => {
        const content = `# This is a header comment
int32 counter
# Another comment
float64 temperature
# Final comment`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 2, 'Should parse 2 fields, skipping comment lines');
        assert.strictEqual(result.comments.size, 3, 'Should extract 3 standalone comments');
    });
    
    test('Should skip separator lines (---)', () => {
        const content = `int32 counter
---
float64 temperature
------
string robot_name`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 fields, skipping separator lines');
        assert.strictEqual(result.fields[0].line, 0);
        assert.strictEqual(result.fields[1].line, 2);
        assert.strictEqual(result.fields[2].line, 4);
    });
    
    test('Should handle optional fields with @optional modifier', () => {
        const content = `@optional int32 optional_counter
@optional float64 optional_temperature
int32 required_field`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 fields including optional');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'int32');
        assert.strictEqual(field1.name, 'optional_counter');
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'float64');
        assert.strictEqual(field2.name, 'optional_temperature');
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'int32');
        assert.strictEqual(field3.name, 'required_field');
    });
    
    test('Should handle nested message types', () => {
        const content = `std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
builtin_interfaces/Time timestamp`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 4, 'Should parse 4 nested message fields');
        
        const field1 = result.fields[0];
        assert.strictEqual(field1.type, 'std_msgs/Header');
        assert.strictEqual(field1.name, 'header');
        
        const field2 = result.fields[1];
        assert.strictEqual(field2.type, 'geometry_msgs/Point');
        assert.strictEqual(field2.name, 'position');
        
        const field3 = result.fields[2];
        assert.strictEqual(field3.type, 'geometry_msgs/Quaternion');
        assert.strictEqual(field3.name, 'orientation');
        
        const field4 = result.fields[3];
        assert.strictEqual(field4.type, 'builtin_interfaces/Time');
        assert.strictEqual(field4.name, 'timestamp');
    });
    
    test('Should correctly identify column position of type', () => {
        const content = `int32 counter
  float64 temperature
    string robot_name`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 fields');
        
        assert.strictEqual(result.fields[0].column, 0, 'First field type starts at column 0');
        assert.strictEqual(result.fields[1].column, 2, 'Second field type starts at column 2');
        assert.strictEqual(result.fields[2].column, 4, 'Third field type starts at column 4');
    });
    
    test('Should handle complex message with all features', () => {
        const content = `# Complex message demonstrating all features
std_msgs/Header header

# Primitive types
int32 counter
float64 temperature  # Temperature in Celsius

# Arrays
float64[] joint_positions
int32[6] joint_efforts
uint8[<=10] status_codes

# Constants
int32 MAX_SPEED=100
string DEFAULT_MODE="autonomous"

---

# Optional fields
@optional string optional_message

# Nested types
geometry_msgs/Point position
builtin_interfaces/Time timestamp`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 11, 'Should parse all fields correctly');
        assert.ok(result.comments.size > 0, 'Should extract comments');
        
        // Verify a few key fields
        const headerField = result.fields.find(f => f.name === 'header');
        assert.ok(headerField);
        assert.strictEqual(headerField.type, 'std_msgs/Header');
        
        const arrayField = result.fields.find(f => f.name === 'joint_positions');
        assert.ok(arrayField);
        assert.strictEqual(arrayField.arraySize, '');
        
        const constantField = result.fields.find(f => f.name === 'MAX_SPEED');
        assert.ok(constantField);
        assert.strictEqual(constantField.isConstant, true);
        assert.strictEqual(constantField.defaultValue, '100');
    });
    
    test('Should handle constants with inline comments', () => {
        const content = `int32 MAX_SPEED=100  # Maximum speed in m/s
float64 PI=3.14159   # Pi constant
string MODE="auto"   # Default mode`;
        
        const doc = createMockDocument(content);
        const result = parseMessageFile(doc);
        
        assert.strictEqual(result.fields.length, 3, 'Should parse 3 constant fields');
        assert.strictEqual(result.comments.size, 3, 'Should extract 3 inline comments');
        
        assert.strictEqual(result.comments.get(0), 'Maximum speed in m/s');
        assert.strictEqual(result.comments.get(1), 'Pi constant');
        assert.strictEqual(result.comments.get(2), 'Default mode');
    });
});
