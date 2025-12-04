/**
 * EnvFile Test
 * 
 * Test the envFile parsing and merging functionality.
 */

import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';
import * as os from 'os';
import { parseEnvFile, mergeEnvFile } from '../../src/debugger/utils';

describe('EnvFile Tests', () => {
    let tempDir: string;
    let testEnvFile: string;

    before(() => {
        // Create a temporary directory for test files
        tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'envfile-test-'));
        testEnvFile = path.join(tempDir, '.env');
    });

    after(() => {
        // Clean up temp directory
        if (fs.existsSync(testEnvFile)) {
            fs.unlinkSync(testEnvFile);
        }
        if (fs.existsSync(tempDir)) {
            fs.rmdirSync(tempDir);
        }
    });

    describe('parseEnvFile', () => {
        it('should parse basic KEY=VALUE format', () => {
            const content = `KEY1=value1
KEY2=value2`;
            fs.writeFileSync(testEnvFile, content);

            const result = parseEnvFile(testEnvFile);

            assert.strictEqual(result.KEY1, 'value1');
            assert.strictEqual(result.KEY2, 'value2');
        });

        it('should handle empty lines and comments', () => {
            const content = `# This is a comment
KEY1=value1

# Another comment
KEY2=value2
`;
            fs.writeFileSync(testEnvFile, content);

            const result = parseEnvFile(testEnvFile);

            assert.strictEqual(result.KEY1, 'value1');
            assert.strictEqual(result.KEY2, 'value2');
            assert.strictEqual(Object.keys(result).length, 2);
        });

        it('should remove surrounding quotes', () => {
            const content = `KEY1="quoted value"
KEY2='single quoted'
KEY3=unquoted`;
            fs.writeFileSync(testEnvFile, content);

            const result = parseEnvFile(testEnvFile);

            assert.strictEqual(result.KEY1, 'quoted value');
            assert.strictEqual(result.KEY2, 'single quoted');
            assert.strictEqual(result.KEY3, 'unquoted');
        });

        it('should handle values with = sign', () => {
            const content = `CONNECTION_STRING=key=value;other=data`;
            fs.writeFileSync(testEnvFile, content);

            const result = parseEnvFile(testEnvFile);

            assert.strictEqual(result.CONNECTION_STRING, 'key=value;other=data');
        });

        it('should handle values with spaces', () => {
            const content = `KEY1=value with spaces
KEY2="value with spaces"`;
            fs.writeFileSync(testEnvFile, content);

            const result = parseEnvFile(testEnvFile);

            assert.strictEqual(result.KEY1, 'value with spaces');
            assert.strictEqual(result.KEY2, 'value with spaces');
        });

        it('should return empty object for non-existent file', () => {
            const result = parseEnvFile(path.join(tempDir, 'nonexistent.env'));

            assert.deepStrictEqual(result, {});
        });

        it('should handle empty values', () => {
            const content = `KEY1=
KEY2=value`;
            fs.writeFileSync(testEnvFile, content);

            const result = parseEnvFile(testEnvFile);

            assert.strictEqual(result.KEY1, '');
            assert.strictEqual(result.KEY2, 'value');
        });
    });

    describe('mergeEnvFile', () => {
        it('should merge envFile variables', () => {
            const content = `FILE_VAR1=file_value1
FILE_VAR2=file_value2`;
            fs.writeFileSync(testEnvFile, content);

            const result = mergeEnvFile(undefined, testEnvFile);

            assert.strictEqual(result.FILE_VAR1, 'file_value1');
            assert.strictEqual(result.FILE_VAR2, 'file_value2');
        });

        it('should merge env object over envFile', () => {
            const content = `VAR1=from_file
VAR2=from_file`;
            fs.writeFileSync(testEnvFile, content);

            const env = {
                VAR2: 'from_env',
                VAR3: 'from_env'
            };

            const result = mergeEnvFile(env, testEnvFile);

            assert.strictEqual(result.VAR1, 'from_file');
            assert.strictEqual(result.VAR2, 'from_env'); // env takes precedence
            assert.strictEqual(result.VAR3, 'from_env');
        });

        it('should handle undefined envFile', () => {
            const env = {
                VAR1: 'value1'
            };

            const result = mergeEnvFile(env, undefined);

            assert.strictEqual(result.VAR1, 'value1');
            assert.strictEqual(Object.keys(result).length, 1);
        });

        it('should handle undefined env', () => {
            const content = `VAR1=value1`;
            fs.writeFileSync(testEnvFile, content);

            const result = mergeEnvFile(undefined, testEnvFile);

            assert.strictEqual(result.VAR1, 'value1');
        });

        it('should handle both undefined', () => {
            const result = mergeEnvFile(undefined, undefined);

            assert.deepStrictEqual(result, {});
        });

        it('should return empty object for non-existent envFile', () => {
            const result = mergeEnvFile(undefined, path.join(tempDir, 'nonexistent.env'));

            assert.deepStrictEqual(result, {});
        });
    });
});
