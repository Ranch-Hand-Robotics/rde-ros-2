// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as assert from 'assert';
import * as vscode from 'vscode';
import * as vscode_utils from '../../src/vscode-utils';

suite('Cursor Detection Test Suite', () => {
    test('Should detect environment correctly', () => {
        // Mock extension context
        const mockContext = {
            extensionPath: '/path/to/extension',
            extensionRuntime: 'node',
            subscriptions: []
        } as vscode.ExtensionContext;

        // Test the detection function
        const isCursor = vscode_utils.isRunningInCursor(mockContext);
        
        // The function should return a boolean
        assert.strictEqual(typeof isCursor, 'boolean');
        
        // Log the result for debugging
        console.log(`Environment detection result: ${isCursor ? 'Cursor' : 'VS Code'}`);
    });

    test('Should handle missing context gracefully', () => {
        // Test with null context
        const isCursor = vscode_utils.isRunningInCursor(null as any);
        assert.strictEqual(typeof isCursor, 'boolean');
    });

    test('Should handle undefined context gracefully', () => {
        // Test with undefined context
        const isCursor = vscode_utils.isRunningInCursor(undefined as any);
        assert.strictEqual(typeof isCursor, 'boolean');
    });
});
