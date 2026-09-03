// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as topicMonitor from '../ros2/topic-monitor';
import { TopicInfo, TopicQoS } from '../ros2/topic-types';
import { TopicTreeItem, TopicTreeItemType } from './topic-tree-item';

export class TopicTreeDataProvider implements vscode.TreeDataProvider<TopicTreeItem> {
  private _onDidChangeTreeData = new vscode.EventEmitter<TopicTreeItem | undefined | void>();
  readonly onDidChangeTreeData = this._onDidChangeTreeData.event;

  private subscribedTopics = new Set<string>();
  private topicMetricsCache = new Map<string, {
    frequency?: number, 
    publisherCount?: number, 
    subscriberCount?: number,
    qos?: TopicQoS,
    updatedAt: number
  }>();
  private topicMetricsRequests = new Map<string, Promise<void>>();
  private refreshInterval?: NodeJS.Timeout;
  private isWindowFocused = true;
  private readonly metricsCacheDurationMs = 15000;

  constructor(
    context: vscode.ExtensionContext,
    private readonly outputChannel: vscode.OutputChannel,
    private readonly onTopicSubscriptionChanged: (topic: TopicInfo, subscribe: boolean) => void | Promise<void>
  ) {
    // Auto-refresh every 5 seconds when window is focused
    this.refreshInterval = setInterval(() => {
      if (this.isWindowFocused) {
        this.refresh();
      }
    }, 5000);

    // Track window focus to pause refreshing when not active
    context.subscriptions.push(
      vscode.window.onDidChangeWindowState((state) => {
        this.isWindowFocused = state.focused;
      })
    );
  }

  /**
   * Refresh the tree view
   */
  refresh(): void {
    this._onDidChangeTreeData.fire();
  }

  /**
   * Get tree item for display
   */
  getTreeItem(element: TopicTreeItem): vscode.TreeItem {
    // Update metrics tooltip if available
    if (element.type === TopicTreeItemType.Topic && element.topicInfo) {
      const metrics = this.topicMetricsCache.get(element.topicInfo.name);
      if (metrics) {
        element.setMetrics(metrics.frequency, metrics.publisherCount, metrics.subscriberCount, metrics.qos);
      }
    }
    return element;
  }

  /**
   * Get children of a tree item (root level topics)
   */
  async getChildren(element?: TopicTreeItem): Promise<TopicTreeItem[]> {
    if (element) {
      // Topics don't have children
      return [];
    }

    try {
      // Get all topics
      const topics = await topicMonitor.listTopics();

      if (topics.length === 0) {
        return [TopicTreeItem.createEmptyStateItem('No topics found. Make sure ROS 2 daemon is running.')];
      }

      // Create tree items
      const items = topics.map(topic => {
        const isSubscribed = this.subscribedTopics.has(topic.name);
        const item = TopicTreeItem.createTopicItem(topic, isSubscribed);
        
        // Update metrics cache asynchronously (don't block rendering)
        void this.updateTopicMetrics(topic.name).catch(err => {
          const errorMessage = err instanceof Error ? err.message : String(err);
          this.outputChannel.appendLine(`Error updating metrics for ${topic.name}: ${errorMessage}`);
        });
        
        return item;
      });

      return items;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : String(error);
      this.outputChannel.appendLine(`Error getting topics: ${errorMessage}`);
      return [TopicTreeItem.createErrorItem('Failed to retrieve topics')];
    }
  }

  /**
   * Update metrics for a topic
   */
  private async updateTopicMetrics(topicName: string): Promise<void> {
    const cached = this.topicMetricsCache.get(topicName);
    if (cached && Date.now() - cached.updatedAt < this.metricsCacheDurationMs) {
      return;
    }

    const existingRequest = this.topicMetricsRequests.get(topicName);
    if (existingRequest) {
      return existingRequest;
    }

    const request = (async () => {
      const info = await topicMonitor.getTopicInfo(topicName);
      if (info) {
        this.topicMetricsCache.set(topicName, {
          publisherCount: info.publisherCount,
          subscriberCount: info.subscriberCount,
          qos: info.qos,
          updatedAt: Date.now()
        });
      }
    })();

    this.topicMetricsRequests.set(topicName, request);
    try {
      await request;
    } finally {
      this.topicMetricsRequests.delete(topicName);
    }
  }

  /**
   * Handle checkbox state changes
   */
  async handleCheckboxChange(events: readonly vscode.TreeCheckboxChangeEvent<TopicTreeItem>[]): Promise<void> {
    for (const event of events) {
      for (const [treeItem, newState] of event.items) {
        if (treeItem.type !== TopicTreeItemType.Topic || !treeItem.topicInfo) {
          continue;
        }

        const topicName = treeItem.topicInfo.name;
        const isChecked = newState === vscode.TreeItemCheckboxState.Checked;

        if (isChecked) {
          this.subscribedTopics.add(topicName);
          await this.onTopicSubscriptionChanged(treeItem.topicInfo, true);
        } else {
          this.subscribedTopics.delete(topicName);
          await this.onTopicSubscriptionChanged(treeItem.topicInfo, false);
        }

        // Update the tree item
        treeItem.isSubscribed = isChecked;
        treeItem.checkboxState = newState;
        treeItem.contextValue = isChecked ? 'topicSubscribed' : 'topicUnsubscribed';
      }
    }

    this.refresh();
  }

  /**
   * Subscribe to a topic programmatically
   */
  public async subscribe(topic: TopicInfo): Promise<void> {
    const topicName = topic.name;
    if (!this.subscribedTopics.has(topicName)) {
      this.subscribedTopics.add(topicName);
      await this.onTopicSubscriptionChanged(topic, true);
      this.refresh();
    }
  }

  /**
   * Unsubscribe from a topic programmatically
   */
  public async unsubscribe(topic: TopicInfo): Promise<void> {
    const topicName = topic.name;
    if (this.subscribedTopics.has(topicName)) {
      this.subscribedTopics.delete(topicName);
      await this.onTopicSubscriptionChanged(topic, false);
      this.refresh();
    }
  }

  /**
   * Unsubscribe from all topics
   */
  public async unsubscribeAll(): Promise<void> {
    for (const topicName of this.subscribedTopics) {
      await this.onTopicSubscriptionChanged({ name: topicName, type: "" }, false);
    }
    this.subscribedTopics.clear();
    this.refresh();
  }

  /**
   * Get subscribed topics
   */
  public getSubscribedTopics(): string[] {
    return Array.from(this.subscribedTopics);
  }

  /**
   * Dispose resources
   */
  dispose(): void {
    if (this.refreshInterval) {
      clearInterval(this.refreshInterval);
    }
    this._onDidChangeTreeData.dispose();
  }
}
