# RDE C++ Publisher

A simple ROS 2 C++ package that publishes string messages on the `/rde` topic.

## Features

- Publishes "hello rde" messages with an incrementing index
- Messages are published every second
- Uses standard ROS 2 C++ APIs (rclcpp)

## Building

From the workspace root:

```bash
# Build the package
colcon build --packages-select rde_cpp

# Source the workspace
source install/setup.bash
```

## Running

### Method 1: Publisher only
```bash
ros2 run rde_cpp rde_publisher
```

### Method 2: Subscriber only
```bash
ros2 run rde_cpp rde_subscriber
```

### Method 3: Using launch files
```bash
# Publisher only
ros2 launch rde_cpp rde_publisher.launch.py

# Full demo (publisher + subscriber)
ros2 launch rde_cpp rde_demo.launch.py
```

## Monitoring

To see the published messages:

```bash
# Listen to the /rde topic
ros2 topic echo /rde

# Check topic info
ros2 topic info /rde

# List all topics
ros2 topic list
```

## Expected Output

The publisher will output messages like:
```
[INFO] [rde_publisher]: Publishing: 'hello rde 0'
[INFO] [rde_publisher]: Publishing: 'hello rde 1'
[INFO] [rde_publisher]: Publishing: 'hello rde 2'
...
```

And the topic will contain string messages with incrementing numbers.

## Package Structure

```
rde_cpp/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── src/
│   ├── rde_publisher.cpp  # Publisher source code
│   └── rde_subscriber.cpp # Subscriber source code
├── launch/
│   ├── rde_publisher.launch.py  # Publisher launch file
│   └── rde_demo.launch.py      # Full demo launch file
└── README.md              # This file
```
