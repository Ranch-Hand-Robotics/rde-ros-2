// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../extension";

const promisifiedExec = util.promisify(child_process.exec);

/**
 * Represents a ROS 2 Lifecycle Node state
 */
export interface LifecycleState {
    id: number;
    label: string;
}

/**
 * Represents a ROS 2 Lifecycle Node transition
 */
export interface LifecycleTransition {
    id: number;
    label: string;
    start: number;
    goal: number;
}

/**
 * Represents a ROS 2 Lifecycle Node
 */
export interface LifecycleNode {
    name: string;
    namespace: string;
    currentState: LifecycleState;
    availableTransitions: LifecycleTransition[];
}

/**
 * Standard ROS 2 Lifecycle states
 */
export const LIFECYCLE_STATES = {
    UNCONFIGURED: { id: 1, label: "unconfigured" },
    INACTIVE: { id: 2, label: "inactive" },
    ACTIVE: { id: 3, label: "active" },
    FINALIZED: { id: 4, label: "finalized" }
};

/**
 * Standard ROS 2 Lifecycle transitions
 */
export const LIFECYCLE_TRANSITIONS = {
    CONFIGURE: { id: 1, label: "configure", start: 1, goal: 2 },
    CLEANUP: { id: 2, label: "cleanup", start: 2, goal: 1 },
    ACTIVATE: { id: 3, label: "activate", start: 2, goal: 3 },
    DEACTIVATE: { id: 4, label: "deactivate", start: 3, goal: 2 },
    UNCONFIGURED_SHUTDOWN: { id: 5, label: "unconfigured_shutdown", start: 1, goal: 4 },
    INACTIVE_SHUTDOWN: { id: 6, label: "inactive_shutdown", start: 2, goal: 4 },
    ACTIVE_SHUTDOWN: { id: 7, label: "active_shutdown", start: 3, goal: 4 }
};

/**
 * Gets all available lifecycle nodes in the system
 */
export async function getLifecycleNodes(): Promise<string[]> {
    try {
        const { stdout } = await promisifiedExec("ros2 lifecycle nodes", { env: extension.env });
        const lines = stdout.trim().split(os.EOL);
        return lines
            .filter(line => line.trim().length > 0)
            .filter(line => !line.trim().startsWith("ros2cli"));
    } catch (error) {
        extension.outputChannel.appendLine(`Error getting lifecycle nodes: ${error.message}`);
        return [];
    }
}

/**
 * Gets the current state of a lifecycle node
 */
export async function getNodeState(nodeName: string): Promise<LifecycleState | null> {
    try {
        const { stdout } = await promisifiedExec(`ros2 lifecycle get ${nodeName}`, { env: extension.env });
        const stateLabel = stdout.trim();
        
        // Find the state by label
        for (const state of Object.values(LIFECYCLE_STATES)) {
            // State labels have the form state [index], so just do start with match
            if (stateLabel.startsWith(state.label.toLocaleLowerCase())) {
                return state;
            }
        }
        
        extension.outputChannel.appendLine(`Unknown lifecycle state: ${stateLabel}`);
        return null;
    } catch (error) {
        extension.outputChannel.appendLine(`Error getting node state for ${nodeName}: ${error.message}`);
        return null;
    }
}

/**
 * Gets the available transitions for a lifecycle node based on its current state
 */
export async function getAvailableTransitions(nodeName: string): Promise<LifecycleTransition[]> {
    const currentState = await getNodeState(nodeName);
    if (!currentState) {
        return [];
    }

    const availableTransitions: LifecycleTransition[] = [];
    
    for (const transition of Object.values(LIFECYCLE_TRANSITIONS)) {
        if (transition.start === currentState.id) {
            availableTransitions.push(transition);
        }
    }
    
    return availableTransitions;
}

/**
 * Triggers a transition on a lifecycle node
 */
export async function triggerTransition(nodeName: string, transitionId: number): Promise<boolean> {
    try {
        extension.outputChannel.appendLine(`Triggering transition ${transitionId} on node ${nodeName}`);
        const { stdout } = await promisifiedExec(`ros2 lifecycle set ${nodeName} ${transitionId}`, { env: extension.env });
        extension.outputChannel.appendLine(`Transition result: ${stdout.trim()}`);
        return true;
    } catch (error) {
        extension.outputChannel.appendLine(`Error triggering transition on ${nodeName}: ${error.message}`);
        vscode.window.showErrorMessage(`Failed to trigger transition on ${nodeName}: ${error.message}`);
        return false;
    }
}

/**
 * Triggers a transition on a lifecycle node by transition label
 */
export async function triggerTransitionByLabel(nodeName: string, transitionLabel: string): Promise<boolean> {
    const transition = Object.values(LIFECYCLE_TRANSITIONS).find(t => t.label === transitionLabel);
    if (!transition) {
        vscode.window.showErrorMessage(`Unknown transition: ${transitionLabel}`);
        return false;
    }
    
    return await triggerTransition(nodeName, transition.id);
}

/**
 * Gets detailed information about a lifecycle node
 */
export async function getNodeInfo(nodeName: string): Promise<LifecycleNode | null> {
    try {
        const currentState = await getNodeState(nodeName);
        if (!currentState) {
            return null;
        }
        
        const availableTransitions = await getAvailableTransitions(nodeName);
        
        // Parse namespace from node name
        const parts = nodeName.split('/');
        const namespace = parts.length > 1 ? parts.slice(0, -1).join('/') : '';
        const name = parts[parts.length - 1];
        
        return {
            name: name,
            namespace: namespace,
            currentState: currentState,
            availableTransitions: availableTransitions
        };
    } catch (error) {
        extension.outputChannel.appendLine(`Error getting node info for ${nodeName}: ${error.message}`);
        return null;
    }
}

/**
 * Sets a lifecycle node to a specific state by performing the necessary transitions
 */
export async function setNodeToState(nodeName: string, targetStateLabel: string): Promise<boolean> {
    const targetState = Object.values(LIFECYCLE_STATES).find(s => s.label === targetStateLabel);
    if (!targetState) {
        vscode.window.showErrorMessage(`Unknown target state: ${targetStateLabel}`);
        return false;
    }
    
    const currentState = await getNodeState(nodeName);
    if (!currentState) {
        return false;
    }
    
    if (currentState.id === targetState.id) {
        vscode.window.showInformationMessage(`Node ${nodeName} is already in state ${targetStateLabel}`);
        return true;
    }
    
    // Find the transition path from current to target state
    const transitionPath = findTransitionPath(currentState.id, targetState.id);
    if (transitionPath.length === 0) {
        vscode.window.showErrorMessage(`No transition path found from ${currentState.label} to ${targetStateLabel}`);
        return false;
    }
    
    // Execute transitions in sequence
    for (const transitionId of transitionPath) {
        const success = await triggerTransition(nodeName, transitionId);
        if (!success) {
            return false;
        }
        
        // Wait a bit for the transition to complete
        await new Promise(resolve => setTimeout(resolve, 500));
    }
    
    return true;
}

/**
 * Finds the transition path from source state to target state
 */
function findTransitionPath(sourceStateId: number, targetStateId: number): number[] {
    // Simple state transition logic for ROS 2 lifecycle
    const path: number[] = [];
    
    switch (sourceStateId) {
        case LIFECYCLE_STATES.UNCONFIGURED.id:
            if (targetStateId === LIFECYCLE_STATES.INACTIVE.id) {
                path.push(LIFECYCLE_TRANSITIONS.CONFIGURE.id);
            } else if (targetStateId === LIFECYCLE_STATES.ACTIVE.id) {
                path.push(LIFECYCLE_TRANSITIONS.CONFIGURE.id, LIFECYCLE_TRANSITIONS.ACTIVATE.id);
            } else if (targetStateId === LIFECYCLE_STATES.FINALIZED.id) {
                path.push(LIFECYCLE_TRANSITIONS.UNCONFIGURED_SHUTDOWN.id);
            }
            break;
            
        case LIFECYCLE_STATES.INACTIVE.id:
            if (targetStateId === LIFECYCLE_STATES.UNCONFIGURED.id) {
                path.push(LIFECYCLE_TRANSITIONS.CLEANUP.id);
            } else if (targetStateId === LIFECYCLE_STATES.ACTIVE.id) {
                path.push(LIFECYCLE_TRANSITIONS.ACTIVATE.id);
            } else if (targetStateId === LIFECYCLE_STATES.FINALIZED.id) {
                path.push(LIFECYCLE_TRANSITIONS.INACTIVE_SHUTDOWN.id);
            }
            break;
            
        case LIFECYCLE_STATES.ACTIVE.id:
            if (targetStateId === LIFECYCLE_STATES.INACTIVE.id) {
                path.push(LIFECYCLE_TRANSITIONS.DEACTIVATE.id);
            } else if (targetStateId === LIFECYCLE_STATES.UNCONFIGURED.id) {
                path.push(LIFECYCLE_TRANSITIONS.DEACTIVATE.id, LIFECYCLE_TRANSITIONS.CLEANUP.id);
            } else if (targetStateId === LIFECYCLE_STATES.FINALIZED.id) {
                path.push(LIFECYCLE_TRANSITIONS.ACTIVE_SHUTDOWN.id);
            }
            break;
            
        case LIFECYCLE_STATES.FINALIZED.id:
            // No transitions from finalized state
            break;
    }
    
    return path;
}
