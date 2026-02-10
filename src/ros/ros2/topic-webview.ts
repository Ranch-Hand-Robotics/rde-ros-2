// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../../extension";
import * as topicMonitor from "./topic-monitor";
import { TopicMessage, TopicMonitorState, isImageType } from "./topic-types";

/**
 * Manages webview panels for topic monitoring
 */
export class TopicWebviewManager {
  private panels = new Map<string, vscode.WebviewPanel>();
  private echoManager: topicMonitor.TopicEchoManager;
  private messageBuffers = new Map<string, TopicMessage[]>();
  private pausedTopics = new Set<string>();
  private maxMessagesPerTopic = 100;

  constructor(private context: vscode.ExtensionContext) {
    this.echoManager = new topicMonitor.TopicEchoManager();
  }

  /**
   * Open or show a webview for a topic
   */
  public async openTopicMonitor(topicName: string, topicType: string): Promise<void> {
    // Check if panel already exists
    let panel = this.panels.get(topicName);
    
    if (panel) {
      // Panel exists, just reveal it
      panel.reveal(vscode.ViewColumn.Beside);
      return;
    }

    // Create new panel
    panel = vscode.window.createWebviewPanel(
      `topicMonitor_${topicName}`,
      `Topic: ${topicName}`,
      vscode.ViewColumn.Beside,
      {
        enableScripts: true,
        retainContextWhenHidden: true
      }
    );

    this.panels.set(topicName, panel);
    this.messageBuffers.set(topicName, []);

    // Set up webview content
    panel.webview.html = this.getWebviewContent(panel.webview, topicName, topicType);

    // Handle messages from webview
    panel.webview.onDidReceiveMessage(
      message => this.handleWebviewMessage(topicName, message),
      undefined,
      this.context.subscriptions
    );

    // Handle panel disposal
    panel.onDidDispose(
      () => {
        this.closeTopicMonitor(topicName);
      },
      undefined,
      this.context.subscriptions
    );

    // Start monitoring the topic
    this.startMonitoring(topicName);
  }

  /**
   * Close a topic monitor webview
   */
  public closeTopicMonitor(topicName: string): void {
    this.echoManager.stopEcho(topicName);
    this.panels.delete(topicName);
    this.messageBuffers.delete(topicName);
    this.pausedTopics.delete(topicName);
  }

  /**
   * Close all topic monitors
   */
  public closeAll(): void {
    for (const [topicName, panel] of this.panels) {
      panel.dispose();
      this.closeTopicMonitor(topicName);
    }
  }

  /**
   * Start monitoring a topic
   */
  private startMonitoring(topicName: string): void {
    this.echoManager.startEcho(topicName, (message: TopicMessage) => {
      // Check if topic is paused
      if (this.pausedTopics.has(topicName)) {
        return;
      }

      // Add to message buffer
      const buffer = this.messageBuffers.get(topicName) || [];
      buffer.push(message);

      // Limit buffer size
      if (buffer.length > this.maxMessagesPerTopic) {
        buffer.shift();
      }

      this.messageBuffers.set(topicName, buffer);

      // Send to webview
      const panel = this.panels.get(topicName);
      if (panel) {
        panel.webview.postMessage({
          command: 'newMessage',
          message: message
        });
      }
    });
  }

  /**
   * Handle messages from webview
   */
  private handleWebviewMessage(topicName: string, message: any): void {
    switch (message.command) {
      case 'pause':
        this.pausedTopics.add(topicName);
        break;
      case 'resume':
        this.pausedTopics.delete(topicName);
        break;
      case 'clear':
        this.messageBuffers.set(topicName, []);
        break;
      case 'getHistory':
        // Send message history to webview
        const panel = this.panels.get(topicName);
        const buffer = this.messageBuffers.get(topicName) || [];
        if (panel) {
          panel.webview.postMessage({
            command: 'history',
            messages: buffer
          });
        }
        break;
    }
  }

  /**
   * Get webview HTML content
   */
  private getWebviewContent(webview: vscode.Webview, topicName: string, topicType: string): string {
    const isImage = isImageType(topicType);
    
    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Topic Monitor: ${topicName}</title>
    <style>
        body {
            font-family: var(--vscode-font-family);
            color: var(--vscode-foreground);
            background-color: var(--vscode-editor-background);
            padding: 20px;
            margin: 0;
        }
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 1px solid var(--vscode-panel-border);
        }
        .topic-info {
            flex: 1;
        }
        .topic-name {
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 5px;
        }
        .topic-type {
            font-size: 14px;
            color: var(--vscode-descriptionForeground);
        }
        .controls {
            display: flex;
            gap: 10px;
        }
        button {
            background-color: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            padding: 6px 12px;
            cursor: pointer;
            border-radius: 2px;
        }
        button:hover {
            background-color: var(--vscode-button-hoverBackground);
        }
        button.paused {
            background-color: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
        }
        .content {
            max-height: calc(100vh - 150px);
            overflow-y: auto;
        }
        .message {
            background-color: var(--vscode-editor-background);
            border: 1px solid var(--vscode-panel-border);
            border-radius: 4px;
            padding: 10px;
            margin-bottom: 10px;
        }
        .message-header {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 12px;
            color: var(--vscode-descriptionForeground);
        }
        .message-content {
            font-family: var(--vscode-editor-font-family);
            font-size: 12px;
            white-space: pre-wrap;
            word-break: break-word;
        }
        .image-container {
            text-align: center;
            margin: 10px 0;
        }
        .image-container img {
            max-width: 100%;
            border: 1px solid var(--vscode-panel-border);
            border-radius: 4px;
        }
        .no-messages {
            text-align: center;
            color: var(--vscode-descriptionForeground);
            padding: 40px;
        }
        .json-key {
            color: var(--vscode-symbolIcon-propertyForeground);
        }
        .json-string {
            color: var(--vscode-symbolIcon-stringForeground);
        }
        .json-number {
            color: var(--vscode-symbolIcon-numberForeground);
        }
        .json-boolean {
            color: var(--vscode-symbolIcon-booleanForeground);
        }
    </style>
</head>
<body>
    <div class="header">
        <div class="topic-info">
            <div class="topic-name">${topicName}</div>
            <div class="topic-type">${topicType}</div>
        </div>
        <div class="controls">
            <button id="pauseBtn" onclick="togglePause()">⏸ Pause</button>
            <button onclick="clearMessages()">🗑 Clear</button>
        </div>
    </div>
    <div class="content" id="messageContainer">
        <div class="no-messages">Waiting for messages...</div>
    </div>

    <script>
        const vscode = acquireVsCodeApi();
        let isPaused = false;
        let messages = [];
        const maxMessages = 100;
        const isImageTopic = ${isImage};

        function togglePause() {
            isPaused = !isPaused;
            const btn = document.getElementById('pauseBtn');
            if (isPaused) {
                btn.textContent = '▶ Resume';
                btn.classList.add('paused');
                vscode.postMessage({ command: 'pause' });
            } else {
                btn.textContent = '⏸ Pause';
                btn.classList.remove('paused');
                vscode.postMessage({ command: 'resume' });
            }
        }

        function clearMessages() {
            messages = [];
            renderMessages();
            vscode.postMessage({ command: 'clear' });
        }

        function formatTimestamp(timestamp) {
            const date = new Date(timestamp);
            return date.toLocaleTimeString() + '.' + date.getMilliseconds().toString().padStart(3, '0');
        }

        function syntaxHighlightJson(json) {
            if (typeof json !== 'string') {
                json = JSON.stringify(json, null, 2);
            }
            json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
            return json.replace(/"([^"]+)":/g, '<span class="json-key">"$1":</span>')
                .replace(/"([^"]+)"/g, '<span class="json-string">"$1"</span>')
                .replace(/\b(true|false)\b/g, '<span class="json-boolean">$1</span>')
                .replace(/\b(\d+\.?\d*)\b/g, '<span class="json-number">$1</span>');
        }

        function renderMessages() {
            const container = document.getElementById('messageContainer');
            
            if (messages.length === 0) {
                container.innerHTML = '<div class="no-messages">No messages yet</div>';
                return;
            }

            container.innerHTML = messages.map((msg, idx) => {
                const timestamp = formatTimestamp(msg.timestamp);
                let content = '';

                if (isImageTopic && msg.data.data) {
                    // Render image (assuming base64 data)
                    content = \`<div class="image-container">
                        <img src="data:image/jpeg;base64,\${msg.data.data}" alt="Image message" />
                    </div>\`;
                } else {
                    // Render JSON
                    content = \`<div class="message-content">\${syntaxHighlightJson(msg.data)}</div>\`;
                }

                return \`<div class="message">
                    <div class="message-header">
                        <span>Message #\${idx + 1}</span>
                        <span>\${timestamp}</span>
                    </div>
                    \${content}
                </div>\`;
            }).reverse().join('');
        }

        // Handle messages from extension
        window.addEventListener('message', event => {
            const message = event.data;
            
            switch (message.command) {
                case 'newMessage':
                    messages.push(message.message);
                    if (messages.length > maxMessages) {
                        messages.shift();
                    }
                    renderMessages();
                    break;
                    
                case 'history':
                    messages = message.messages;
                    renderMessages();
                    break;
            }
        });

        // Request message history on load
        vscode.postMessage({ command: 'getHistory' });
    </script>
</body>
</html>`;
  }

  /**
   * Dispose all resources
   */
  public dispose(): void {
    this.echoManager.dispose();
    this.closeAll();
  }
}
