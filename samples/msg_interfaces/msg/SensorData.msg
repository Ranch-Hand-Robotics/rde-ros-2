# SensorData.msg - Testing sensor-related message patterns
# Common patterns used in robotics applications

std_msgs/Header header

# Laser scan data simulation
float32 angle_min -3.14159
float32 angle_max 3.14159
float32 angle_increment 0.017453292
float32 time_increment 0.0
float32 scan_time 0.1
float32 range_min 0.02
float32 range_max 30.0
float32[] ranges
float32[] intensities

# IMU data
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
float64[9] orientation_covariance
float64[9] angular_velocity_covariance  
float64[9] linear_acceleration_covariance

# Camera info
uint32 height 480
uint32 width 640
string distortion_model "plumb_bob"
float64[] d_params
float64[9] k_matrix
float64[9] r_matrix
float64[12] p_matrix
uint32 binning_x 0
uint32 binning_y 0
sensor_msgs/RegionOfInterest roi

# Environmental sensors
float64 temperature 22.5    # Celsius
float64 humidity 45.2       # Percentage
float64 pressure 1013.25    # hPa
uint16 air_quality 150      # AQI
bool rain_detected false

# Status flags
uint8 SENSOR_OK=0
uint8 SENSOR_WARNING=1
uint8 SENSOR_ERROR=2
uint8 SENSOR_DISCONNECTED=3