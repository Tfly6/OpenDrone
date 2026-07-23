Header header

string planner_id
string planner_family
string source_topic
int64 trajectory_id

uint32 output_type
uint32 OUTPUT_WAYPOINT = 1
uint32 OUTPUT_SINGLE = 2
uint32 OUTPUT_HORIZON = 4
uint32 OUTPUT_BSPLINE = 8
uint32 OUTPUT_POLYNOMIAL = 16
uint32 OUTPUT_SAMPLED = 32
uint32 OUTPUT_HEARTBEAT = 64

# Every point's absolute desired time is trajectory_start_time + time_from_start.
# This remains true for OUTPUT_HORIZON: successive rolling windows retain the
# source trajectory origin and use monotonically increasing point offsets.
time trajectory_start_time
duration trajectory_duration
bool is_periodic

PlannerOutputPoint[] points

int32 bspline_order
float64[] bspline_knots
geometry_msgs/Point[] bspline_pos_pts
float64[] bspline_yaw_pts
float64 bspline_yaw_dt

uint32 polynomial_type
uint32 polynomial_piece_num_pos
uint32 polynomial_piece_num_yaw
uint32 polynomial_order_pos
uint32 polynomial_order_yaw
time polynomial_start_time_pos
time polynomial_start_time_yaw
float64 polynomial_yaw
float64 polynomial_yaw_rate
float64[] polynomial_coef_yaw
float64[] polynomial_time_yaw
float64[] polynomial_coef_pos_x
float64[] polynomial_coef_pos_y
float64[] polynomial_coef_pos_z
float64[] polynomial_time_pos

string debug_info
