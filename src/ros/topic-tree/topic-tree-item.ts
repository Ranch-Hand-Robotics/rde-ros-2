// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import { TopicInfo, TopicQoS, isImageType } from '../ros2/topic-types';

export enum TopicTreeItemType {
  Topic = 'topic',
  EmptyState = 'emptyState',
  Error = 'error'
}

export class TopicTreeItem extends vscode.TreeItem {
  public type: TopicTreeItemType;
  public topicInfo?: TopicInfo;
  public isSubscribed: boolean = false;

  constructor(
    label: string,
    type: TopicTreeItemType,
    collapsibleState: vscode.TreeItemCollapsibleState = vscode.TreeItemCollapsibleState.None
  ) {
    super(label, collapsibleState);
    this.type = type;
  }

  /**
   * Create a topic item
   */
  public static createTopicItem(topicInfo: TopicInfo, isSubscribed: boolean = false): TopicTreeItem {
    const item = new TopicTreeItem(
      topicInfo.name,
      TopicTreeItemType.Topic,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.topicInfo = topicInfo;
    item.isSubscribed = isSubscribed;
    item.description = topicInfo.type;
    
    // Set icon based on topic type and subscription state
    if (isImageType(topicInfo.type)) {
      item.iconPath = new vscode.ThemeIcon('file-media');
    } else {
      item.iconPath = new vscode.ThemeIcon('symbol-event');
    }
    
    // Set context value for menu actions
    item.contextValue = isSubscribed ? 'topicSubscribed' : 'topicUnsubscribed';
    
    // Set checkbox state
    item.checkboxState = isSubscribed 
      ? vscode.TreeItemCheckboxState.Checked 
      : vscode.TreeItemCheckboxState.Unchecked;
    
    return item;
  }

  /**
   * Create an empty state item
   */
  public static createEmptyStateItem(message: string): TopicTreeItem {
    const item = new TopicTreeItem(
      message,
      TopicTreeItemType.EmptyState,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('info');
    item.contextValue = 'emptyState';
    
    return item;
  }

  /**
   * Create an error item
   */
  public static createErrorItem(message: string): TopicTreeItem {
    const item = new TopicTreeItem(
      message,
      TopicTreeItemType.Error,
      vscode.TreeItemCollapsibleState.None
    );
    
    item.iconPath = new vscode.ThemeIcon('error');
    item.contextValue = 'error';
    
    return item;
  }

  /**
   * Update tooltip with metrics
   */
  public setMetrics(frequency?: number, publisherCount?: number, subscriberCount?: number, qos?: TopicQoS): void {
    const tooltip = new vscode.MarkdownString();
    tooltip.appendMarkdown(`**${this.topicInfo?.name}**\n\n`);
    tooltip.appendMarkdown(`Type: \`${this.topicInfo?.type}\`\n\n`);
    
    if (frequency !== undefined && frequency > 0) {
      tooltip.appendMarkdown(`Frequency: ${frequency.toFixed(2)} Hz\n\n`);
    }
    
    if (publisherCount !== undefined) {
      tooltip.appendMarkdown(`Publishers: ${publisherCount}\n\n`);
    }
    
    if (subscriberCount !== undefined) {
      tooltip.appendMarkdown(`Subscribers: ${subscriberCount}\n\n`);
    }
    
    // Add QoS information if available
    if (qos) {
      tooltip.appendMarkdown(`**QoS Settings:**\n\n`);
      
      if (qos.reliability) {
        tooltip.appendMarkdown(`- Reliability: ${qos.reliability}\n`);
      }
      if (qos.durability) {
        tooltip.appendMarkdown(`- Durability: ${qos.durability}\n`);
      }
      if (qos.deadline) {
        tooltip.appendMarkdown(`- Deadline: ${qos.deadline}\n`);
      }
      if (qos.lifespan) {
        tooltip.appendMarkdown(`- Lifespan: ${qos.lifespan}\n`);
      }
      if (qos.liveliness) {
        tooltip.appendMarkdown(`- Liveliness: ${qos.liveliness}\n`);
      }
      if (qos.livelinessLeaseDuration) {
        tooltip.appendMarkdown(`- Liveliness Lease Duration: ${qos.livelinessLeaseDuration}\n`);
      }
      
      tooltip.appendMarkdown(`\n`);
    }
    
    this.tooltip = tooltip;
  }
}
