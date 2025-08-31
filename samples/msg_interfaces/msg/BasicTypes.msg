# BasicTypes.msg - Testing primitive data types syntax highlighting
# This message demonstrates all basic ROS2 message types

# Boolean type
bool is_active true

# Integer types
int8 temperature_celsius -25
uint8 battery_percentage 85
int16 servo_position 1500
uint16 sensor_reading 4095
int32 encoder_count -123456
uint32 timestamp_ms 4294967295
int64 large_counter -9223372036854775808
uint64 system_uptime 18446744073709551615

# Floating point types
float32 velocity 3.14159
float64 precise_position 3.141592653589793

# String type
string robot_name "TestBot-2025"
string status_message "All systems operational"

# Byte array (bounded)
uint8[10] sensor_array
uint8[] dynamic_data

# Constants for syntax highlighting
int32 MAX_SPEED=100
float64 PI=3.141592653589793
string DEFAULT_MODE="autonomous"
bool SAFETY_ENABLED=true