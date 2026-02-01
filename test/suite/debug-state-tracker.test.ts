/**
 * Debug State Tracker Test
 * 
 * Test the debug state tracking functionality.
 */

import * as assert from 'assert';
import { DebugStateTracker, DebuggedNode } from '../../src/debugger/debug-state-tracker';

describe('Debug State Tracker Tests', () => {
    let tracker: DebugStateTracker;

    beforeEach(() => {
        tracker = new DebugStateTracker();
    });

    afterEach(() => {
        tracker.clear();
    });

    describe('registerSession', () => {
        it('should register a new debug session', () => {
            const nodes: DebuggedNode[] = [
                {
                    node_name: 'test_node',
                    executable: '/path/to/executable',
                    runtime: 'C++',
                    debug_status: 'running'
                }
            ];

            tracker.registerSession('session-1', 'ros2', 'launch', '/path/to/launch.py', nodes);

            const sessions = tracker.getActiveSessions();
            assert.strictEqual(sessions.length, 1);
            assert.strictEqual(sessions[0].session_id, 'session-1');
            assert.strictEqual(sessions[0].nodes.length, 1);
        });

        it('should track multiple sessions', () => {
            const nodes1: DebuggedNode[] = [
                {
                    node_name: 'node1',
                    executable: '/path/to/node1',
                    runtime: 'Python',
                    debug_status: 'running'
                }
            ];

            const nodes2: DebuggedNode[] = [
                {
                    node_name: 'node2',
                    executable: '/path/to/node2',
                    runtime: 'C++',
                    debug_status: 'paused'
                }
            ];

            tracker.registerSession('session-1', 'ros2', 'launch', '/path/to/launch1.py', nodes1);
            tracker.registerSession('session-2', 'ros2', 'launch', '/path/to/launch2.py', nodes2);

            const sessions = tracker.getActiveSessions();
            assert.strictEqual(sessions.length, 2);
        });
    });

    describe('getNodeDebugInfo', () => {
        it('should return node debug info for registered nodes', () => {
            const nodes: DebuggedNode[] = [
                {
                    node_name: 'test_node',
                    executable: '/path/to/executable',
                    runtime: 'C++',
                    debug_status: 'running'
                }
            ];

            tracker.registerSession('session-1', 'ros2', 'launch', '/path/to/launch.py', nodes);

            const nodeInfo = tracker.getNodeDebugInfo('test_node');
            assert.ok(nodeInfo);
            assert.strictEqual(nodeInfo.node_name, 'test_node');
            assert.strictEqual(nodeInfo.runtime, 'C++');
        });

        it('should return null for non-existent nodes', () => {
            const nodeInfo = tracker.getNodeDebugInfo('non_existent');
            assert.strictEqual(nodeInfo, null);
        });
    });

    describe('unregisterSession', () => {
        it('should remove a debug session', () => {
            const nodes: DebuggedNode[] = [
                {
                    node_name: 'test_node',
                    executable: '/path/to/executable',
                    runtime: 'C++',
                    debug_status: 'running'
                }
            ];

            tracker.registerSession('session-1', 'ros2', 'launch', '/path/to/launch.py', nodes);
            tracker.unregisterSession('session-1');

            const sessions = tracker.getActiveSessions();
            assert.strictEqual(sessions.length, 0);
        });
    });

    describe('clear', () => {
        it('should clear all sessions', () => {
            const nodes: DebuggedNode[] = [
                {
                    node_name: 'test_node',
                    executable: '/path/to/executable',
                    runtime: 'C++',
                    debug_status: 'running'
                }
            ];

            tracker.registerSession('session-1', 'ros2', 'launch', '/path/to/launch1.py', nodes);
            tracker.registerSession('session-2', 'ros2', 'launch', '/path/to/launch2.py', nodes);

            tracker.clear();

            const sessions = tracker.getActiveSessions();
            assert.strictEqual(sessions.length, 0);
        });
    });
});
