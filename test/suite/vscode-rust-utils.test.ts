/**
 * VSCode Utils - Rust Extension Detection Test
 * 
 * Test detection of Rust debugger extensions.
 */

import * as assert from 'assert';
import * as vscode from 'vscode';
import { isRustDebuggerExtensionInstalled } from '../../src/vscode-utils';

describe('VSCode Utils - Rust', () => {
    describe('isRustDebuggerExtensionInstalled', () => {
        it('should return boolean value', () => {
            // Call the function
            const result = isRustDebuggerExtensionInstalled();
            
            // Should return a boolean (true or false depending on installed extensions)
            assert.strictEqual(typeof result, 'boolean');
        });

        it('should detect CodeLLDB extension if installed', () => {
            const codeLldbExtension = vscode.extensions.getExtension('vadimcn.vscode-lldb');
            const result = isRustDebuggerExtensionInstalled();
            
            if (codeLldbExtension !== undefined) {
                assert.strictEqual(result, true);
            }
        });

        it('should detect rust-analyzer extension if installed', () => {
            const rustAnalyzerExtension = vscode.extensions.getExtension('rust-lang.rust-analyzer');
            const result = isRustDebuggerExtensionInstalled();
            
            if (rustAnalyzerExtension !== undefined) {
                assert.strictEqual(result, true);
            }
        });
    });
});
