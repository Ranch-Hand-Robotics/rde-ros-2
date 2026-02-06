// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as topicMonitor from '../ros2/topic-monitor';
import { TopicTreeItem, TopicTreeItemType } from './topic-tree-item';

export class TopicTreeDataProvider implements vscode.TreeDataProvider<TopicTreeItem> {
  private _onDidChangeTreeData = new vscode.EventEmitter<TopicTreeItem | undefined | void>();
  readonly onDidChangeTreeData = this._onDidChangeTreeData.event;

  private subscribedTopics = new Set<string>();
  private topicMetricsCache = new Map<string, { frequency?: number, publisherCount?: number, subscriberCount?: number }>();
  private refreshInterval?: NodeJS.Timeout;

  constructor(
    private context: vscode.ExtensionContext,
    private outputChannel: vscode.OutputChannel,
    private onTopicSubscriptionChanged: (topicName: string, subscribe: boolean) => void
  ) {
    // Auto-refresh every 5 seconds to update metrics
    this.refreshInterval = setInterval(() => {
      this.refresh();
    }, 5000);
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
        element.setMetrics(metrics.frequency, metrics.publisherCount, metrics.subscriberCount);
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
        this.updateTopicMetrics(topic.name).catch(err => {
          this.outputChannel.appendLine(`Error updating metrics for ${topic.name}: ${err.message}`);
        });
        
        return item;
      });

      return items;
    } catch (error) {
      this.outputChannel.appendLine(`Error getting topics: ${error.message}`);
      return [TopicTreeItem.createErrorItem('Failed to retrieve topics')];
    }
  }

  /**
   * Update metrics for a topic
   */
  private async updateTopicMetrics(topicName: string): Promise<void> {
    try {
      const info = await topicMonitor.getTopicInfo(topicName);
      if (info) {
        const cached = this.topicMetricsCache.get(topicName) || {};
        cached.publisherCount = info.publisherCount;
        cached.subscriberCount = info.subscriberCount;
        this.topicMetricsCache.set(topicName, cached);
      }
    } catch (error) {
      // Silently fail for individual topic metrics
    }
  }

  /**
   * Handle checkbox state changes
   */
  async handleCheckboxChange(items: readonly vscode.TreeCheckboxChangeEvent<TopicTreeItem>[]): Promise<void> {
    for (const item of items) {
      if (item.items.length === 0) continue;
      
      const treeItem = item.items[0][0];
      if (treeItem.type !== TopicTreeItemType.Topic || !treeItem.topicInfo) {
        continue;
      }

      const topicName = treeItem.topicInfo.name;
      const newState = item.items[0][1];
      const isChecked = newState === vscode.TreeItemCheckboxState.Checked;

      if (isChecked) {
        this.subscribedTopics.add(topicName);
        this.onTopicSubscriptionChanged(topicName, true);
      } else {
        this.subscribedTopics.delete(topicName);
        this.onTopicSubscriptionChanged(topicName, false);
      }

      // Update the tree item
      treeItem.isSubscribed = isChecked;
      treeItem.checkboxState = newState;
      treeItem.contextValue = isChecked ? 'topicSubscribed' : 'topicUnsubscribed';
    }

    this.refresh();
  }

  /**
   * Subscribe to a topic programmatically
   */
  public subscribe(topicName: string): void {
    if (!this.subscribedTopics.has(topicName)) {
      this.subscribedTopics.add(topicName);
      this.onTopicSubscriptionChanged(topicName, true);
      this.refresh();
    }
  }

  /**
   * Unsubscribe from a topic programmatically
   */
  public unsubscribe(topicName: string): void {
    if (this.subscribedTopics.has(topicName)) {
      this.subscribedTopics.delete(topicName);
      this.onTopicSubscriptionChanged(topicName, false);
      this.refresh();
    }
  }

  /**
   * Unsubscribe from all topics
   */
  public unsubscribeAll(): void {
    for (const topicName of this.subscribedTopics) {
      this.onTopicSubscriptionChanged(topicName, false);
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
