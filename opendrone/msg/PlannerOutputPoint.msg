duration time_from_start

uint32 valid_mask
uint32 VALID_POSITION = 1
uint32 VALID_VELOCITY = 2
uint32 VALID_ACCELERATION = 4
uint32 VALID_JERK = 8
uint32 VALID_SNAP = 16
uint32 VALID_YAW = 32
uint32 VALID_YAW_RATE = 64
uint32 VALID_ANGULAR_VELOCITY = 128

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 snap

float64 yaw
float64 yaw_rate

geometry_msgs/Vector3 angular_velocity
