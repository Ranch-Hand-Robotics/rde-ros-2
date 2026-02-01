// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

/**
 * Represents a ROS node being debugged
 */
export interface DebuggedNode {
    node_name: string;
    executable: string;
    source_file?: string;
    process_id?: number;
    runtime: "C++" | "Python" | "Rust" | ".NET" | "Unknown";
    debug_status: "running" | "paused" | "stopped";
}

/**
 * Represents an active debug session
 */
export interface DebugSessionInfo {
    session_id: string;
    type: string;
    request: string;
    target: string;
    nodes: DebuggedNode[];
    status: "active" | "stopped";
    started_at: string;
}

/**
 * Tracks active debug sessions and their associated nodes
 */
export class DebugStateTracker {
    private sessions: Map<string, DebugSessionInfo> = new Map();
    private nodeToSession: Map<string, string> = new Map(); // Maps node executable to session ID
    
    /**
     * Registers a new debug session
     */
    public registerSession(
        sessionId: string,
        type: string,
        request: string,
        target: string,
        nodes: DebuggedNode[]
    ): void {
        const sessionInfo: DebugSessionInfo = {
            session_id: sessionId,
            type,
            request,
            target,
            nodes,
            status: "active",
            started_at: new Date().toISOString()
        };
        
        this.sessions.set(sessionId, sessionInfo);
        
        // Map each node to this session
        for (const node of nodes) {
            this.nodeToSession.set(node.executable, sessionId);
        }
    }
    
    /**
     * Updates a node's status within a session
     */
    public updateNodeStatus(
        sessionId: string,
        nodeName: string,
        status: "running" | "paused" | "stopped"
    ): void {
        const session = this.sessions.get(sessionId);
        if (session) {
            const node = session.nodes.find(n => n.node_name === nodeName);
            if (node) {
                node.debug_status = status;
            }
        }
    }
    
    /**
     * Unregisters a debug session
     */
    public unregisterSession(sessionId: string): void {
        const session = this.sessions.get(sessionId);
        if (session) {
            // Remove node mappings
            for (const node of session.nodes) {
                this.nodeToSession.delete(node.executable);
            }
            
            // Mark as stopped
            session.status = "stopped";
            
            // Remove from active sessions
            this.sessions.delete(sessionId);
        }
    }
    
    /**
     * Gets all active debug sessions
     */
    public getActiveSessions(): DebugSessionInfo[] {
        return Array.from(this.sessions.values()).filter(s => s.status === "active");
    }
    
    /**
     * Gets debug info for a specific node by name
     */
    public getNodeDebugInfo(nodeName: string): DebuggedNode | null {
        for (const session of this.sessions.values()) {
            const node = session.nodes.find(n => n.node_name === nodeName);
            if (node) {
                return node;
            }
        }
        return null;
    }
    
    /**
     * Gets the session associated with a node
     */
    public getSessionForNode(nodeName: string): DebugSessionInfo | null {
        for (const session of this.sessions.values()) {
            const node = session.nodes.find(n => n.node_name === nodeName);
            if (node) {
                return session;
            }
        }
        return null;
    }
    
    /**
     * Clears all tracked sessions
     */
    public clear(): void {
        this.sessions.clear();
        this.nodeToSession.clear();
    }
}

// Global instance
export const debugStateTracker = new DebugStateTracker();
