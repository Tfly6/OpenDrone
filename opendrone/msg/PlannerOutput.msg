Header header

# Adapter-lifetime unique trajectory identity. It must change exactly when a
# new native trajectory becomes active, even if the native planner resets or
# reuses its own counter at a waypoint. It remains stable for every rolling
# horizon window belonging to that trajectory.
uint64 trajectory_id
bool is_horizon
# Timing origin for points[].time_from_start; not an identity by itself.
time trajectory_start_time

# Trajectory lifecycle. This describes the local trajectory only and must not
# be interpreted as waypoint or mission completion.
uint8 TRAJECTORY_STATUS_EMPTY     = 0
uint8 TRAJECTORY_STATUS_READY     = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 trajectory_status

PlannerOutputPoint[] points
