# ComplexMessage.msg - Testing complex types and imports
# This demonstrates nested messages, arrays, and imported types

# Standard ROS message imports
std_msgs/Header header

# Geometry messages
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Twist velocity

# Arrays of different types
float64[] joint_positions
string[] error_messages
geometry_msgs/Point[] waypoints

# Bounded arrays
int32[6] joint_efforts
uint8[<=10] status_codes

# Nested custom message
BasicTypes robot_config

# Time types
builtin_interfaces/Time last_update
builtin_interfaces/Duration operation_time

# Multi-dimensional arrays (represented as flattened arrays with size comments)
float32[9] rotation_matrix    # 3x3 rotation matrix (row-major order)
uint8[307200] image_data      # 480x640 image data (flattened)

# Constants with different types
uint8 STATUS_OK=0
uint8 STATUS_WARNING=1
uint8 STATUS_ERROR=2
string FRAME_ID="base_link"
float64 TIMEOUT_SECONDS=5.0