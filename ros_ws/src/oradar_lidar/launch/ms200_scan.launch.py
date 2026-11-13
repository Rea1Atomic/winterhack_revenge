#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # LiDAR publisher node
  ordlidar_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'lidar_frame'},
        {'scan_topic': '/scan'},
        {'port_name': '/dev/ttyUSB0'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.075},
        {'range_max': 20.0},
        {'interpolation_jump_threshold': 0.30},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )

  # Define LaunchDescription variable
  ord = LaunchDescription()

  ord.add_action(ordlidar_node)

  return ord
