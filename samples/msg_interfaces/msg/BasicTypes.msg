# BasicTypes.msg - Testing primitive data types syntax highlighting
# This message demonstrates all basic ROS2 message types

# Boolean type
bool is_active

# Integer types
int8 temperature_celsius
uint8 battery_percentage
int16 servo_position
uint16 sensor_reading
int32 encoder_count
uint32 timestamp_ms
int64 large_counter
uint64 system_uptime

# Floating point types
float32 velocity
float64 precise_position

# String type
string robot_name
string status_message

# Byte array (bounded)
uint8[10] sensor_array
uint8[] dynamic_data

# Constants for syntax highlighting
int32 MAX_SPEED=100
float64 PI=3.141592653589793
string DEFAULT_MODE="autonomous"
bool SAFETY_ENABLED=true