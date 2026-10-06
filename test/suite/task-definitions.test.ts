import * as assert from 'assert';
import * as fs from 'fs';
import * as path from 'path';

describe('Task definition contributions', () => {
    it('should contribute a strict colcon task definition', () => {
        const packageJsonPath = path.resolve(__dirname, '../../../package.json');
        const packageJsonContent = fs.readFileSync(packageJsonPath, 'utf8');
        const packageJson = JSON.parse(packageJsonContent);

        const taskDefinitions = packageJson?.contributes?.taskDefinitions;
        assert.ok(Array.isArray(taskDefinitions), 'Expected contributes.taskDefinitions to be an array');

        const colconDefinition = taskDefinitions.find((definition: any) => definition?.type === 'colcon');
        assert.ok(colconDefinition, 'Expected a task definition for type "colcon"');

        assert.ok(Array.isArray(colconDefinition.required), 'Expected colcon.required to be an array');
        assert.ok(colconDefinition.required.includes('command'), 'Expected colcon.required to include "command"');

        assert.strictEqual(
            colconDefinition?.properties?.command?.type,
            'string',
            'Expected colcon.properties.command.type to be "string"',
        );
    });

    for (const type of ['ROS2', 'colcon']) {
        it(`should preserve deferred execution options for ${type} tasks`, () => {
            const manifestPath = path.resolve(__dirname, '../../../package.json');
            const manifest = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));
            const definition = manifest.contributes.taskDefinitions.find((item: any) => item.type === type);
            const options = definition?.properties?.taskOptions;
            assert.strictEqual(options?.type, 'object', 'CustomExecution needs contributed taskOptions; reserved options are stripped');
            assert.strictEqual(options?.properties?.cwd?.type, 'string');
            assert.strictEqual(options?.properties?.env?.type, 'object');
            assert.deepStrictEqual(options?.properties?.env?.additionalProperties?.type, ['string', 'null']);
            assert.strictEqual(options?.properties?.shell?.type, 'object');
            assert.strictEqual(options?.properties?.shell?.properties?.executable?.type, 'string');
            assert.strictEqual(options?.properties?.shell?.properties?.args?.type, 'array');
        });
    }
});
