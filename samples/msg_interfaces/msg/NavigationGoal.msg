# NavigationGoal.msg - Testing complex navigation message patterns
# Advanced message with nested structures and planning data

std_msgs/Header header

# Goal identification
string goal_id "nav_goal_12345"
uint32 priority 1                    # 1=high, 5=low
builtin_interfaces/Time deadline

# Target pose
geometry_msgs/PoseStamped target_pose
float64 position_tolerance 0.1       # Meters
float64 orientation_tolerance 0.087  # Radians (~5 degrees)

# Path planning constraints
float64 max_velocity 1.0             # m/s
float64 max_acceleration 0.5         # m/s²
float64 max_angular_velocity 1.57    # rad/s
bool use_global_planner true
bool avoid_obstacles true
string planner_id "NavfnROS"

# Waypoint sequence
geometry_msgs/PoseStamped[] waypoints
float64[] waypoint_tolerances
builtin_interfaces/Duration[] waypoint_timeouts

# Behavior parameters
uint8 behavior_type
uint8 BEHAVIOR_DIRECT=0
uint8 BEHAVIOR_EXPLORE=1
uint8 BEHAVIOR_PATROL=2
uint8 BEHAVIOR_FOLLOW=3

# Recovery behaviors
bool enable_recovery true
string[] recovery_behaviors
float64[] recovery_timeouts

# Area constraints (polygon)
geometry_msgs/Point[] allowed_area
geometry_msgs/Point[] forbidden_areas

# Cost parameters
float64 obstacle_cost_scaling 1.0
float64 path_distance_bias 0.6
float64 goal_distance_bias 0.8
float64 occdist_scale 0.01

# Advanced options
bool preempt_current_goal false
bool clear_costmaps false
string map_frame "map"
string base_frame "base_link"
float64 controller_frequency 10.0    # Hz
float64 planner_frequency 1.0        # Hz

# Callback configurations
bool send_feedback true
float64 feedback_frequency 2.0       # Hz
bool publish_path true
bool publish_goal_markers true