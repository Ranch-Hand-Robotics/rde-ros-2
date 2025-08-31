# ROS 2 Message Interfaces - Syntax Highlighting Test Package

This package provides comprehensive examples of ROS 2 message, service, and action definitions for testing and verifying syntax highlighting in development environments.

## Package Contents

### Messages (`msg/`)

1. **BasicTypes.msg** - Demonstrates all primitive ROS 2 data types
   - Boolean, integer, floating-point, string types
   - Arrays (fixed and dynamic)
   - Constants and default values
   - Comments and documentation

2. **ComplexMessage.msg** - Shows complex message structures
   - Imported standard messages (std_msgs, geometry_msgs)
   - Nested custom messages
   - Multi-dimensional arrays
   - Time and duration types
   - Bounded arrays with different syntaxes

3. **SensorData.msg** - Robotics sensor data patterns
   - Sensor message imports (sensor_msgs)
   - Real-world robotics data structures
   - Covariance matrices
   - Environmental sensor data
   - Status enumeration patterns

4. **RobotStatus.msg** - System status reporting
   - Enumeration-like constants
   - Battery and power management
   - Network connectivity information
   - Hardware status arrays
   - Performance metrics
   - Error reporting structures

5. **NavigationGoal.msg** - Advanced navigation message
   - Complex nested structures
   - Path planning parameters
   - Constraint definitions
   - Behavior configuration
   - Polygon area definitions

### Services (`srv/`)

1. **CalculateSum.srv** - Basic service structure
   - Simple request/response pattern
   - Array handling
   - Optional parameters
   - Response metadata

2. **GetRobotInfo.srv** - Complex service example
   - Conditional response fields
   - Nested custom message types
   - Comprehensive robot information
   - Multiple data categories
   - Performance and diagnostic data

### Actions (`action/`)

1. **MoveToGoal.action** - Complete action definition
   - Goal specification with multiple parameters
   - Comprehensive result reporting
   - Detailed periodic feedback
   - State enumeration
   - Performance tracking
   - Error handling patterns

## Syntax Highlighting Features Tested

This package tests syntax highlighting for:

- **Data Types**: All primitive ROS 2 types (bool, int8-64, uint8-64, float32/64, string)
- **Arrays**: Fixed-size, dynamic, bounded, and multi-dimensional arrays
- **Constants**: Different constant declaration patterns with various data types
- **Comments**: Single-line comments with different formatting styles
- **Imports**: Standard ROS message imports (std_msgs, geometry_msgs, sensor_msgs, etc.)
- **Message Separators**: Service request/response separator (`---`)
- **Action Sections**: Goal, Result, and Feedback sections in action files
- **Field Documentation**: Inline comments explaining field purposes
- **Default Values**: Field initialization with default values
- **Nested Messages**: Custom and standard message nesting
- **Enumeration Patterns**: Constant definitions simulating enums

## Building the Package

```bash
# From your ROS 2 workspace root
colcon build --packages-select msg_interfaces

# Source the setup files
source install/setup.bash
```

## Usage in Development

### VS Code
Install the ROS extension pack for optimal syntax highlighting support.

### VIM/Neovim
Use ros.vim or similar ROS syntax highlighting plugins.

### Other Editors
Most editors with ROS support should recognize these file patterns and provide appropriate syntax highlighting.

## File Extensions

- `.msg` - Message definitions
- `.srv` - Service definitions  
- `.action` - Action definitions

## Testing Syntax Highlighting

Open any of the files in this package in your editor to verify:

1. **Keywords** are highlighted (e.g., data types, separators)
2. **Comments** are properly styled
3. **Strings** and **numbers** have distinct coloring
4. **Constants** are visually distinguished
5. **Imports** are recognizable
6. **Field names** and **types** are differentiated

## Contributing

This package is designed to be comprehensive but can be extended with additional patterns as needed for testing syntax highlighting in various ROS 2 development environments.

## License

Apache License 2.0