# Code Snippets

The ROS 2 extension includes comprehensive code snippets to accelerate development in Python, C++, and launch files.

## How to Use Snippets

1. In your code editor, start typing the snippet prefix
2. Press `Tab` or `Enter` to expand the snippet
3. Use `Tab` to navigate between placeholders
4. Fill in the required values

## Python (rclpy) Snippets

### Node Development

| Prefix | Description |
|--------|-------------|
| `ros2node` | Complete ROS 2 Python node class template with main function |
| `ros2pub` | Create a publisher |
| `ros2pubtimer` | Create a publisher with timer callback |
| `ros2sub` | Create a subscriber with callback method |
| `ros2timer` | Create a timer with callback |

### Services

| Prefix | Description |
|--------|-------------|
| `ros2srv` | Create a service server |
| `ros2client` | Create a service client |
| `ros2call` | Make an async service call |

### Actions

| Prefix | Description |
|--------|-------------|
| `ros2actionserver` | Create an action server |
| `ros2actionclient` | Create an action client |

### Parameters

| Prefix | Description |
|--------|-------------|
| `ros2param` | Declare a parameter |
| `ros2getparam` | Get a parameter value |
| `ros2declgetparam` | Declare and get a parameter |

### Logging

| Prefix | Description |
|--------|-------------|
| `ros2debug` | Log a debug message |
| `ros2info` | Log an info message |
| `ros2warn` | Log a warning message |
| `ros2error` | Log an error message |
| `ros2fatal` | Log a fatal message |

### Quality of Service

| Prefix | Description |
|--------|-------------|
| `ros2qos` | Create a QoS profile |

## C++ (rclcpp) Snippets

### Node Development

| Prefix | Description |
|--------|-------------|
| `ros2node` | Complete ROS 2 C++ node class template with main function |
| `ros2component` | ROS 2 component node template with registration macro |
| `ros2pub` | Create a publisher |
| `ros2pubtimer` | Create a publisher with timer callback |
| `ros2sub` | Create a subscriber with callback method |
| `ros2sublambda` | Create a subscriber with lambda callback |
| `ros2timer` | Create a timer with callback |

### Services

| Prefix | Description |
|--------|-------------|
| `ros2srv` | Create a service server |
| `ros2client` | Create a service client |
| `ros2call` | Make an async service call |

### Actions

| Prefix | Description |
|--------|-------------|
| `ros2actionserver` | Create an action server |
| `ros2actionclient` | Create an action client |

### Parameters

| Prefix | Description |
|--------|-------------|
| `ros2param` | Declare a parameter |
| `ros2getparam` | Get a parameter value |
| `ros2declgetparam` | Declare and get a parameter |

### Logging

| Prefix | Description |
|--------|-------------|
| `ros2debug` | Log a debug message (RCLCPP_DEBUG) |
| `ros2info` | Log an info message (RCLCPP_INFO) |
| `ros2warn` | Log a warning message (RCLCPP_WARN) |
| `ros2error` | Log an error message (RCLCPP_ERROR) |
| `ros2fatal` | Log a fatal message (RCLCPP_FATAL) |

### Component Registration

| Prefix | Description |
|--------|-------------|
| `ros2register` | Register a component node with RCLCPP_COMPONENTS_REGISTER_NODE |

### Quality of Service

| Prefix | Description |
|--------|-------------|
| `ros2qos` | Create a QoS profile |

## Python Launch File Snippets

| Prefix | Description |
|--------|-------------|
| `ros2launch` | Complete launch file template |
| `ros2launchnode` | Launch a node with full options |
| `ros2node` | Simple launch node |
| `ros2launchremap` | Launch node with topic remappings |
| `ros2launchinclude` | Include another launch file |
| `ros2launcharg` | Declare a launch argument |
| `ros2launchgetarg` | Get a launch argument value |
| `ros2launchexec` | Execute a process |
| `ros2composable` | Create a composable node container |
| `ros2launchgroup` | Create a group action with namespace |
| `ros2launchparam` | Set a parameter |
| `ros2lifecycle` | Launch a lifecycle node |

## XML Launch File Snippets

| Prefix | Description |
|--------|-------------|
| `ros2launch` | Complete XML launch file template |
| `ros2node` | Simple launch node |
| `ros2nodens` | Launch node with namespace |
| `ros2nodeblock` | Launch node as block element |
| `ros2arg` | Declare launch argument with default |
| `ros2argnodefault` | Declare launch argument without default |
| `ros2include` | Include another launch file |
| `ros2includeargs` | Include launch file with arguments |
| `ros2param` | Set a parameter |
| `ros2paramfile` | Load parameters from YAML file |
| `ros2remap` | Remap a topic |
| `ros2group` | Create a group block |
| `ros2namespace` | Push ROS namespace |
| `ros2env` | Set environment variable in node |
| `ros2setenv` | Set environment variable globally |
| `ros2let` | Define a variable |
| `ros2var` | Get a variable value |
| `ros2getenv` | Get an environment variable |
| `ros2findpkg` | Find package share directory |
| `ros2exec` | Execute a command |

## Examples

### Python Node Example

Type `ros2node` and press Tab to get:

```python
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ Node Example

Type `ros2node` and press Tab to get:

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        
    }

private:
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Python Launch File Example

Type `ros2launch` and press Tab to get:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
    ])
```

## Tips

- All snippets use tab stops to navigate between placeholder values
- Press `Tab` to move to the next placeholder
- Press `Shift+Tab` to move to the previous placeholder
- Some snippets include choice placeholders - use arrow keys to select options
- Snippets work in both VS Code and VSCodium

## Snippet Design Philosophy

These snippets are designed to:

1. **Accelerate development** - Reduce boilerplate typing
2. **Promote best practices** - Follow ROS 2 conventions
3. **Be discoverable** - Use intuitive prefixes starting with `ros2`
4. **Be comprehensive** - Cover common ROS 2 patterns
5. **Be ergonomic** - Use consistent tab stop ordering and sensible defaults
