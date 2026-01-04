# SensorData.msg - Testing sensor-related message patterns
# Common patterns used in robotics applications

std_msgs/Header header

# Laser scan data simulation
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

# IMU data
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
float64[9] orientation_covariance
float64[9] angular_velocity_covariance  
float64[9] linear_acceleration_covariance

# Camera info
uint32 height
uint32 width
string distortion_model
float64[] d_params
float64[9] k_matrix
float64[9] r_matrix
float64[12] p_matrix
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi

# Environmental sensors
float64 temperature    # Celsius
float64 humidity       # Percentage
float64 pressure       # hPa
uint16 air_quality     # AQI
bool rain_detected

# Status flags
uint8 SENSOR_OK=0
uint8 SENSOR_WARNING=1
uint8 SENSOR_ERROR=2
uint8 SENSOR_DISCONNECTED=3