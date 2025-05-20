#!/bin/bash
rosbag record /mavros/local_position/pose /mavros/local_position/velocity_body /mavros/setpoint_raw/attitude /mavros/local_position/odom /mavros/local_position/accel /mavros/setpoint_position/local /command/trajectory /mavros/setpoint_velocity/cmd_vel_unstamped
