mav_control_interface
==================

ROS interface layer between trajectory/odometry topics and a position controller implementation.

Current behavior
------

- No RC teleoperation support.
- Consumes:
  - `command/pose` (`geometry_msgs/PoseStamped`)
  - `command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)
  - `odometry` (`nav_msgs/Odometry`)
- Provides state machine services/events such as `takeoff` and `back_to_position_hold`.

PX4/MAVROS bridge
------

- Node: `px4_attitude_bridge_node`
- Input: `command/roll_pitch_yawrate_thrust` (`mav_msgs/RollPitchYawrateThrust`)
- Output: `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`)
- Odometry input: `/mavros/local_position/odom`

Main parameters:
- `mass` (vehicle mass, kg)
- `hover_thrust` (normalized hover thrust, usually around `0.5`)
- `odom_timeout`, `thrust_min`, `thrust_max`, `yaw_rate_scale`
- `setpoint_rate` (continuous setpoint streaming rate, default `50 Hz`)
- `enable_sim` (if `false`, disables automatic OFFBOARD switch and arming)
- `enable_auto_offboard`, `enable_auto_arm`
- `offboard_warmup_count`, `request_interval`

Migration (RC removal)
------

- Removed classes/files:
  - `RcInterfaceBase`
  - `RcInterfaceAci`
  - `rc_interface.*`, `rc_interface_aci.*`
- Constructor updated:
  - before: `MavControlInterface(nh, private_nh, controller, rc_interface)`
  - now: `MavControlInterface(nh, private_nh, controller)`
- Removed parameter:
  - `use_rc_teleop`
