#!/bin/sh

# Lidar scan driver
ros2 launch oradar_lidar ms200_scan.launch.py

# Odom
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
