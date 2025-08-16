# RDE Python Package

A simple ROS 2 Python package that demonstrates basic publisher and subscriber functionality.

## Package Structure

```
rde_py/
├── rde_py/                    # Python package directory
│   ├── __init__.py
│   ├── rde_publisher.py      # Publisher node
│   └── rde_subscriber.py     # Subscriber node
├── launch/                    # Launch files
│   ├── rde_publisher.launch.py
│   ├── rde_subscriber.launch.py
│   └── rde_demo.launch.py
├── resource/                  # Package resources
├── package.xml               # Package metadata
├── setup.py                  # Package setup
└── README.md                 # This file
```

## Building the Package

From the workspace root (`/home/polyhobbyist/ws/rde-ros-2`):

```bash
# Build the package
colcon build --packages-select rde_py

# Source the workspace
source install/setup.bash
```

## Running the Package

### Option 1: Publisher Only
```bash
ros2 launch rde_py rde_publisher.launch.py
```

### Option 2: Subscriber Only
```bash
ros2 launch rde_py rde_subscriber.launch.py
```

### Option 3: Full Demo (Publisher + Subscriber)
```bash
ros2 launch rde_py rde_demo.launch.py
```

### Option 4: Run Nodes Individually
```bash
# Terminal 1: Start publisher
ros2 run rde_py rde_publisher

# Terminal 2: Start subscriber
ros2 run rde_py rde_subscriber
```

## What It Does

- **Publisher**: Publishes "Hello from RDE #X" messages on the `/rde` topic every second, where X is an incrementing counter
- **Subscriber**: Listens to the `/rde` topic and prints received messages

## Monitoring

To see the messages being published and received:

```bash
# Monitor the /rde topic
ros2 topic echo /rde

# List active topics
ros2 topic list

# Get topic info
ros2 topic info /rde
```

## Stopping

Press `Ctrl+C` in any terminal running the nodes to stop them.
