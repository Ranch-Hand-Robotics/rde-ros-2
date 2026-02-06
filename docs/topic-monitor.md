# ROS 2 Topic Monitor

The ROS 2 Topic Monitor feature provides an RQT-like interface for monitoring topics directly within VS Code, eliminating the need for external terminals or separate RQT windows.

## Features

- **Topic Tree View**: Browse all available ROS 2 topics in your system
- **Live Monitoring**: Subscribe to topics and view messages in real-time
- **Multiple Topics**: Monitor multiple topics simultaneously in separate webview panels
- **Message Display**: 
  - Generic messages shown as formatted JSON
  - Image topics displayed as images (planned)
- **Metrics**: View topic frequency, publisher count, and subscriber count in tooltips
- **Play/Pause Controls**: Pause and resume topic monitoring per topic or all at once

## How to Use

### Accessing the Topic Monitor

1. Open the **Explorer** sidebar in VS Code
2. Find the **ROS 2 Topics** view panel (below ROS 2 Launch Files)
3. The panel will automatically list all available topics from your ROS 2 system

### Subscribing to Topics

To monitor a topic:

1. Find the topic you want to monitor in the ROS 2 Topics tree
2. Click the checkbox next to the topic name
3. A new webview panel will open showing live messages from that topic

### Topic Information

Hover over any topic in the tree to see:
- Topic type (message type)
- Publishing frequency (Hz)
- Number of publishers
- Number of subscribers

### Controlling Topic Monitoring

**Individual Topic Controls** (in webview):
- **Pause/Resume**: Click the pause button to temporarily stop receiving new messages
- **Clear**: Click the clear button to remove all displayed messages from the view

**All Topics Controls** (in tree view toolbar):
- **Refresh**: Click the refresh icon to update the topic list
- **Stop All**: Click the stop icon to unsubscribe from all topics and close all monitoring windows

## Requirements

- ROS 2 Humble or newer installed and sourced
- ROS 2 daemon must be running for topic discovery
- Active ROS 2 nodes publishing to topics you want to monitor

## Topic Types

### Generic Messages

Most ROS 2 message types are displayed as formatted JSON with syntax highlighting:

```json
{
  "header": {
    "stamp": {
      "sec": 1234567890,
      "nanosec": 123456789
    },
    "frame_id": "base_link"
  },
  "data": 42.0
}
```

### Image Messages (Planned)

Image topics (`sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`) will be rendered as actual images in the webview.

## Tips

- **Performance**: Monitoring many high-frequency topics may impact performance. Use pause controls when not actively viewing messages.
- **Message History**: Each topic webview keeps the last 100 messages by default.
- **Auto-refresh**: The topic list automatically refreshes every 5 seconds to show new topics.

## Troubleshooting

### No Topics Shown

If no topics appear in the list:
1. Verify ROS 2 is properly sourced in your environment
2. Check that the ROS 2 daemon is running: `ros2 daemon status`
3. Start the daemon if needed: `ros2 daemon start`
4. Click the refresh button in the topic tree toolbar

### Topics Not Updating

If topic messages aren't updating:
1. Check that the topic is actually publishing: `ros2 topic hz <topic_name>`
2. Verify publishers exist: `ros2 topic info <topic_name>`
3. Try unchecking and rechecking the topic checkbox

### Webview Not Opening

If clicking a topic checkbox doesn't open a webview:
1. Check the Output panel (View → Output) and select "ROS 2" for error messages
2. Try refreshing the topic list
3. Restart VS Code if the issue persists

## Known Limitations

- Image topic rendering is not yet implemented (shows as JSON currently)
- QoS information is not yet displayed in tooltips (planned)
- Cannot adjust message buffer size (fixed at 100 messages)
- Cannot filter or search within messages

## Related Commands

All commands are available via the Command Palette (Ctrl+Shift+P / Cmd+Shift+P):

- `ROS2: Refresh Topic List` - Manually refresh the topic tree
- `ROS2: Stop All Topic Monitors` - Stop monitoring all topics
