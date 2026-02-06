// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as util from "util";

import * as extension from "../../extension";
import {
  TopicInfo,
  TopicQoS,
  TopicMetrics,
  TopicMessage,
  TopicMonitorState,
  TopicMonitorSession
} from "./topic-types";

const promisifiedExec = util.promisify(child_process.exec);

/**
 * Get list of all topics in the ROS 2 system
 */
export async function listTopics(): Promise<TopicInfo[]> {
  try {
    const { stdout } = await promisifiedExec("ros2 topic list -t", { env: extension.env });
    const lines = stdout.trim().split(os.EOL);
    
    const topics: TopicInfo[] = [];
    for (const line of lines) {
      if (line.trim().length === 0) continue;
      
      // Format: topic_name [message_type]
      const match = line.match(/^(.+?)\s+\[(.+?)\]$/);
      if (match) {
        const [, name, type] = match;
        topics.push({
          name: name.trim(),
          type: type.trim()
        });
      }
    }
    
    // Filter out ros2cli internal topics
    return topics.filter(topic => !topic.name.includes('ros2cli'));
  } catch (error) {
    extension.outputChannel.appendLine(`Error listing topics: ${error.message}`);
    return [];
  }
}

/**
 * Get detailed information about a specific topic
 */
export async function getTopicInfo(topicName: string): Promise<TopicInfo | null> {
  try {
    const { stdout } = await promisifiedExec(`ros2 topic info ${topicName} -v`, { env: extension.env });
    
    const info: TopicInfo = {
      name: topicName,
      type: '',
      publisherCount: 0,
      subscriberCount: 0
    };
    
    // Parse the verbose output
    const lines = stdout.split(os.EOL);
    for (const line of lines) {
      if (line.includes('Type:')) {
        info.type = line.split(':')[1].trim();
      } else if (line.includes('Publisher count:')) {
        info.publisherCount = parseInt(line.split(':')[1].trim());
      } else if (line.includes('Subscription count:')) {
        info.subscriberCount = parseInt(line.split(':')[1].trim();
      }
    }
    
    return info;
  } catch (error) {
    extension.outputChannel.appendLine(`Error getting topic info for ${topicName}: ${error.message}`);
    return null;
  }
}

/**
 * Get frequency statistics for a topic
 * Note: This uses ros2 topic hz which blocks, so use with timeout
 */
export async function getTopicFrequency(topicName: string, durationSec: number = 2): Promise<TopicMetrics | null> {
  try {
    // Use timeout to limit the duration of hz command
    const command = `timeout ${durationSec} ros2 topic hz ${topicName} || true`;
    const { stdout } = await promisifiedExec(command, { env: extension.env });
    
    const metrics: TopicMetrics = {
      averageRate: 0
    };
    
    // Parse output: "average rate: X.XXX"
    const lines = stdout.split(os.EOL);
    for (const line of lines) {
      if (line.includes('average rate:')) {
        const match = line.match(/average rate:\s+([\d.]+)/);
        if (match) {
          metrics.averageRate = parseFloat(match[1]);
        }
      } else if (line.includes('min:')) {
        const match = line.match(/min:\s+([\d.]+)/);
        if (match) {
          metrics.minRate = parseFloat(match[1]);
        }
      } else if (line.includes('max:')) {
        const match = line.match(/max:\s+([\d.]+)/);
        if (match) {
          metrics.maxRate = parseFloat(match[1]);
        }
      }
    }
    
    return metrics;
  } catch (error) {
    extension.outputChannel.appendLine(`Error getting topic frequency for ${topicName}: ${error.message}`);
    return null;
  }
}

/**
 * Topic echo process manager
 */
export class TopicEchoManager {
  private activeProcesses = new Map<string, child_process.ChildProcess>();
  private messageHandlers = new Map<string, (message: TopicMessage) => void>();

  /**
   * Start echoing a topic
   */
  public startEcho(topicName: string, onMessage: (message: TopicMessage) => void): void {
    // Stop any existing process for this topic
    this.stopEcho(topicName);

    const command = `ros2 topic echo ${topicName} --no-arr --no-str-len`;
    const process = child_process.spawn(
      process.platform === 'win32' ? 'cmd' : 'sh',
      process.platform === 'win32' ? ['/c', command] : ['-c', command],
      {
        env: extension.env,
        stdio: ['ignore', 'pipe', 'pipe']
      }
    );

    this.messageHandlers.set(topicName, onMessage);
    this.activeProcesses.set(topicName, process);

    let buffer = '';

    process.stdout?.on('data', (data: Buffer) => {
      buffer += data.toString();
      
      // Split by YAML document separator (---)
      const messages = buffer.split('---');
      
      // Keep the last incomplete message in buffer
      buffer = messages.pop() || '';
      
      // Process complete messages
      for (const messageStr of messages) {
        if (messageStr.trim().length === 0) continue;
        
        try {
          // Parse YAML-like output to JSON
          const jsonData = this.parseYamlToJson(messageStr);
          const message: TopicMessage = {
            timestamp: Date.now(),
            data: jsonData,
            rawData: messageStr
          };
          
          onMessage(message);
        } catch (error) {
          extension.outputChannel.appendLine(`Error parsing message from ${topicName}: ${error.message}`);
        }
      }
    });

    process.stderr?.on('data', (data: Buffer) => {
      extension.outputChannel.appendLine(`Error from topic echo ${topicName}: ${data.toString()}`);
    });

    process.on('exit', (code) => {
      extension.outputChannel.appendLine(`Topic echo process for ${topicName} exited with code ${code}`);
      this.activeProcesses.delete(topicName);
      this.messageHandlers.delete(topicName);
    });
  }

  /**
   * Stop echoing a topic
   */
  public stopEcho(topicName: string): void {
    const process = this.activeProcesses.get(topicName);
    if (process) {
      process.kill();
      this.activeProcesses.delete(topicName);
      this.messageHandlers.delete(topicName);
    }
  }

  /**
   * Stop all echo processes
   */
  public stopAll(): void {
    for (const [topicName] of this.activeProcesses) {
      this.stopEcho(topicName);
    }
  }

  /**
   * Check if a topic is being echoed
   */
  public isEchoing(topicName: string): boolean {
    return this.activeProcesses.has(topicName);
  }

  /**
   * Parse YAML-like output from ros2 topic echo to JSON
   * This is a simple parser that handles basic YAML structures
   */
  private parseYamlToJson(yaml: string): any {
    try {
      // Simple YAML to JSON conversion
      // For complex messages, we just return the raw string
      const lines = yaml.trim().split('\n');
      const obj: any = {};
      
      for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.length === 0 || trimmed === '---') continue;
        
        const colonIndex = trimmed.indexOf(':');
        if (colonIndex > 0) {
          const key = trimmed.substring(0, colonIndex).trim();
          let value: any = trimmed.substring(colonIndex + 1).trim();
          
          // Try to parse numbers and booleans
          if (value === 'true') value = true;
          else if (value === 'false') value = false;
          else if (!isNaN(Number(value)) && value !== '') value = Number(value);
          
          obj[key] = value;
        }
      }
      
      return obj;
    } catch (error) {
      // If parsing fails, return the raw string
      return { raw: yaml };
    }
  }

  /**
   * Dispose all resources
   */
  public dispose(): void {
    this.stopAll();
  }
}
