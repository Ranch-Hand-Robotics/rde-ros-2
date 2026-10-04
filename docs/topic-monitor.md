# ROS 2 Topic Monitor

The ROS 2 Topic Monitor feature provides an RQT-like interface for monitoring topics directly within VS Code, eliminating the need for external terminals or separate RQT windows.

## Features

- **Topic Tree View**: Browse all available ROS 2 topics in your system
- **Live Monitoring**: Subscribe to topics and view messages in real-time
- **Multiple Topics**: Monitor multiple topics simultaneously in separate webview panels
- **Message Display**: 
  - Generic messages shown as formatted JSON
  - Raw and compressed image topics displayed as images
  - PointCloud2 topics displayed as interactive 3D point clouds using raw WebGPU
- **Metrics**: View publisher count, subscriber count, and QoS details in tooltips
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
- Number of publishers
- Number of subscribers
- **QoS (Quality of Service) settings:**
  - Reliability (RELIABLE, BEST_EFFORT)
  - Durability (VOLATILE, TRANSIENT_LOCAL)
  - Deadline (time constraint)
  - Lifespan (message validity duration)
  - Liveliness (node liveness policy)
  - Liveliness Lease Duration (timeout for liveliness)

### Controlling Topic Monitoring

**Individual Topic Controls** (in webview):
- **Pause/Resume**: Click the pause button to temporarily stop receiving new messages
- **Clear**: Click the clear button to remove all displayed messages from the view
- **Refresh**: Set the image preview rate from 1–30 Hz (default 5 Hz)
- **Buffer**: Adjust retained message history from 1–500 messages

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

### Image Messages

Raw `sensor_msgs/msg/Image` and compressed `sensor_msgs/msg/CompressedImage` topics are rendered in the webview. Raw previews support common RGB/BGR and monochrome encodings, including 16-bit depth images, with row stride and endianness preserved.

Image monitoring uses a direct `rclpy` subscription in the selected ROS/Pixi Python environment, rather than converting every image byte to YAML through `ros2 topic echo`. The subscriber uses best-effort, volatile QoS with a depth-one queue and throttles before base64 encoding. The refresh slider controls this source-side preview rate without restarting the subscription. Pausing or closing the monitor stops its subscriber process.

### PointCloud2 preview

Subscribe to a `sensor_msgs/msg/PointCloud2` topic to open the WebGPU preview:

- **Orbit:** drag with the left mouse button, or focus the canvas and use arrow keys.
- **Zoom:** scroll, or use `+` / `-`. **Fit view** (or `F`) fits the latest cloud.
  Zoom can move inside the cloud for millimeter and submillimeter close-ups around
  its center. Camera clipping follows the zoom distance; only a one-micrometer
  minimum camera-to-center distance remains. Detail is limited by the source
  point spacing and coordinate precision, not the cloud's overall size.
- **Frame rotation:** enter X, Y, and Z angles in degrees, applied in that order.
  These fields also update when dragging or using the arrow keys, so they describe
  the current preview orientation. The initial view uses ROS's +Z-up convention.
  **Reset** restores the initial view and zeroes all three angles; **Fit view** and
  zoom do not change them. These are preview rotations, not ROS TF transforms.
- **Color:** choose **Depth**, **RGB**, or **RGB + depth**. The blend slider mixes
  embedded color with a depth ramp. Without RGB fields, depth is used automatically.
- **Depth:** distance in meters from the message's sensor-frame origin (not the
  orbit camera). Auto depth uses the rendered cloud's range; turn it off and set
  Near/Far to keep the same color scale across messages. Far must exceed Near.
- **Point size:** adjust screen-space point diameter. The depth buffer hides points
  behind closer points. Optional RGB frame axes are drawn at the cloud center as
  an orientation aid, not a TF origin marker.

The preview supports numeric XYZ fields, packed FLOAT32/UINT32 `rgb` or `rgba`,
and separate `r`, `g`, `b` channels (integer 0–255 or floating-point 0–1).
Endianness, field offsets, and organized-cloud row padding are respected.
Invalid XYZ points are skipped. The latest cloud replaces the previous one;
the default refresh is 5 Hz and can be adjusted. Camera orientation is retained
as messages arrive; use **Fit view** when the scene bounds change.

To bound preview memory and GPU work, payloads are limited to 32 MiB and clouds
over 200,000 points are evenly sampled. Counts and sampling are shown below the
canvas. This is a lightweight preview: it does not resolve TF transforms, accumulate
cloud history, or reproduce all RViz display features. Topic transport still uses
`ros2 topic echo`, so reduce the publisher's rate/size for very large clouds.

WebGPU requires a compatible GPU/driver and a recent VS Code with hardware
acceleration. An explicit notice is shown when WebGPU is unavailable or the GPU
device is lost; close and reopen the topic after correcting the issue. There is
no WebGL fallback or external rendering-library dependency.

## Tips

- **Performance**: Monitoring many high-frequency topics may impact performance. Use pause controls when not actively viewing messages.
- **Message History**: Generic topics retain 100 messages by default; image previews retain only the latest frame.
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
4. For images, check the **ROS 2** Output channel for subscriber or Python environment errors. The selected ROS Python environment must include `rclpy` and `sensor_msgs`.

### Webview Not Opening

If clicking a topic checkbox doesn't open a webview:
1. Check the Output panel (View → Output) and select "ROS 2" for error messages
2. Try refreshing the topic list
3. Restart VS Code if the issue persists

## Known Limitations

- Image previews accept payloads up to 32 MiB per frame
- Unsupported raw image encodings display an explanatory notice instead of a preview
- Cannot filter or search within messages

## Related Commands

All commands are available via the Command Palette (Ctrl+Shift+P / Cmd+Shift+P):

- `ROS2: Refresh Topic List` - Manually refresh the topic tree
- `ROS2: Stop All Topic Monitors` - Stop monitoring all topics
