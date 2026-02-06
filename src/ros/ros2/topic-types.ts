// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

/**
 * Represents a ROS 2 topic with its metadata
 */
export interface TopicInfo {
  name: string;
  type: string;
  publisherCount?: number;
  subscriberCount?: number;
}

/**
 * QoS (Quality of Service) information for a topic
 */
export interface TopicQoS {
  reliability: string;
  durability: string;
  deadline: string;
  lifespan: string;
  liveliness: string;
  livelinessLeaseDuration: string;
}

/**
 * Topic metrics including frequency and bandwidth
 */
export interface TopicMetrics {
  averageRate: number; // Hz
  minRate?: number;
  maxRate?: number;
  messageCount?: number;
}

/**
 * Represents a message received from a topic
 */
export interface TopicMessage {
  timestamp: number;
  data: any; // Can be object (parsed JSON) or string
  rawData?: string;
}

/**
 * State of topic monitoring
 */
export enum TopicMonitorState {
  Stopped = 'stopped',
  Running = 'running',
  Paused = 'paused',
  Error = 'error'
}

/**
 * Topic monitor session
 */
export interface TopicMonitorSession {
  topicName: string;
  state: TopicMonitorState;
  messages: TopicMessage[];
  maxMessages?: number; // Maximum number of messages to retain
  errorMessage?: string;
}

/**
 * Detected image message types
 */
export const IMAGE_MESSAGE_TYPES = [
  'sensor_msgs/msg/Image',
  'sensor_msgs/msg/CompressedImage'
];

/**
 * Check if a topic type is an image type
 */
export function isImageType(messageType: string): boolean {
  return IMAGE_MESSAGE_TYPES.some(imageType => 
    messageType.includes(imageType)
  );
}
