// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as util from "util";
import * as yaml from "js-yaml";

import * as extension from "../../extension";
import {
  TopicInfo,
  TopicQoS,
  TopicMetrics,
  TopicMessage
} from "./topic-types";

const promisifiedExecFile = util.promisify(child_process.execFile);

/**
 * Get list of all topics in the ROS 2 system
 */
export async function listTopics(): Promise<TopicInfo[]> {
  try {
    const { stdout } = await promisifiedExecFile("ros2", ["topic", "list", "-t"], { env: extension.env });
    const lines = stdout.trim().split(/\r?\n/);
    
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
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(`Error listing topics: ${errorMessage}`);
    throw error;
  }
}

/**
 * Get detailed information about a specific topic
 */
export async function getTopicInfo(topicName: string): Promise<TopicInfo | null> {
  try {
    const { stdout } = await promisifiedExecFile("ros2", ["topic", "info", topicName, "-v"], { env: extension.env });
    
    const info: TopicInfo = {
      name: topicName,
      type: '',
      publisherCount: 0,
      subscriberCount: 0
    };
    
    const qos: TopicQoS = {
      reliability: '',
      durability: '',
      deadline: '',
      lifespan: '',
      liveliness: '',
      livelinessLeaseDuration: ''
    };
    
    // Parse the verbose output
    const lines = stdout.split(os.EOL);
    for (const line of lines) {
      const trimmedLine = line.trim();
      
      if (trimmedLine.includes('Type:')) {
        info.type = trimmedLine.split(':')[1].trim();
      } else if (trimmedLine.includes('Publisher count:')) {
        info.publisherCount = parseInt(trimmedLine.split(':')[1].trim());
      } else if (trimmedLine.includes('Subscription count:')) {
        info.subscriberCount = parseInt(trimmedLine.split(':')[1].trim());
      } 
      // Parse QoS settings
      else if (trimmedLine.includes('Reliability:')) {
        qos.reliability = trimmedLine.split(':')[1].trim();
      } else if (trimmedLine.includes('Durability:')) {
        qos.durability = trimmedLine.split(':')[1].trim();
      } else if (trimmedLine.includes('Deadline:')) {
        qos.deadline = trimmedLine.split(':')[1].trim();
      } else if (trimmedLine.includes('Lifespan:')) {
        qos.lifespan = trimmedLine.split(':')[1].trim();
      } else if (trimmedLine.includes('Liveliness:')) {
        qos.liveliness = trimmedLine.split(':')[1].trim();
      } else if (trimmedLine.includes('Liveliness lease duration:')) {
        qos.livelinessLeaseDuration = trimmedLine.split(':')[1].trim();
      }
    }
    
    // Only add QoS if we found at least one QoS setting
    if (qos.reliability || qos.durability || qos.deadline || qos.lifespan || qos.liveliness) {
      info.qos = qos;
    }
    
    return info;
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    extension.outputChannel.appendLine(`Error getting topic info for ${topicName}: ${errorMessage}`);
    return null;
  }
}

/**
 * Get frequency statistics for a topic
 * Note: This uses ros2 topic hz which blocks, so use with timeout
 */
export async function getTopicFrequency(topicName: string, durationSec: number = 2): Promise<TopicMetrics | null> {
  return new Promise(resolve => {
    const process = child_process.spawn("ros2", ["topic", "hz", topicName], {
      env: extension.env,
      stdio: ["ignore", "pipe", "pipe"],
      windowsHide: true
    });
    let output = "";
    let settled = false;

    const finish = (result: TopicMetrics | null): void => {
      if (settled) {
        return;
      }
      settled = true;
      clearTimeout(timeout);
      resolve(result);
    };

    process.stdout?.on("data", (data: Buffer) => {
      output += data.toString();
    });
    process.on("error", error => {
      extension.outputChannel.appendLine(`Error getting topic frequency for ${topicName}: ${error.message}`);
      finish(null);
    });
    process.on("exit", () => finish(parseTopicFrequency(output)));

    const timeout = setTimeout(() => {
      process.kill();
      finish(parseTopicFrequency(output));
    }, durationSec * 1000);
  });
}

export function parseTopicFrequency(output: string): TopicMetrics | null {
  const averageMatch = output.match(/average rate:\s+([\d.]+)/);
  if (!averageMatch) {
    return null;
  }

  const minMatch = output.match(/min:\s+([\d.]+)/);
  const maxMatch = output.match(/max:\s+([\d.]+)/);
  return {
    averageRate: Number(averageMatch[1]),
    minRate: minMatch ? Number(minMatch[1]) : undefined,
    maxRate: maxMatch ? Number(maxMatch[1]) : undefined
  };
}

export function parseTopicEchoMessage(message: string): unknown {
  return yaml.load(message);
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

    const childProcess = child_process.spawn(
      "ros2",
      ["topic", "echo", topicName],
      {
        env: extension.env,
        stdio: ["ignore", "pipe", "pipe"],
        windowsHide: true
      }
    );

    this.messageHandlers.set(topicName, onMessage);
    this.activeProcesses.set(topicName, childProcess);

    let buffer = '';

    childProcess.stdout?.on('data', (data: Buffer) => {
      if (this.activeProcesses.get(topicName) !== childProcess) {
        return;
      }

      buffer += data.toString();
      
      // Split by YAML document separator (---)
      const messages = buffer.split('---');
      
      // Keep the last incomplete message in buffer
      buffer = messages.pop() || '';
      
      // Process complete messages
      for (const messageStr of messages) {
        if (messageStr.trim().length === 0) continue;
        
        try {
          const jsonData = parseTopicEchoMessage(messageStr);
          const message: TopicMessage = {
            timestamp: Date.now(),
            data: jsonData
          };
          
          if (this.activeProcesses.get(topicName) === childProcess) {
            onMessage(message);
          }
        } catch (error) {
          const errorMessage = error instanceof Error ? error.message : String(error);
          extension.outputChannel.appendLine(`Error parsing message from ${topicName}: ${errorMessage}`);
        }
      }
    });

    childProcess.stderr?.on('data', (data: Buffer) => {
      extension.outputChannel.appendLine(`Error from topic echo ${topicName}: ${data.toString()}`);
    });

    childProcess.on("error", error => {
      extension.outputChannel.appendLine(`Unable to start topic echo for ${topicName}: ${error.message}`);
      if (this.activeProcesses.get(topicName) === childProcess) {
        this.activeProcesses.delete(topicName);
        this.messageHandlers.delete(topicName);
      }
    });

    childProcess.on('exit', (code) => {
      extension.outputChannel.appendLine(`Topic echo process for ${topicName} exited with code ${code}`);
      if (this.activeProcesses.get(topicName) === childProcess) {
        this.activeProcesses.delete(topicName);
        this.messageHandlers.delete(topicName);
      }
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
   * Dispose all resources
   */
  public dispose(): void {
    this.stopAll();
  }
}
