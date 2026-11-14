import os
from os.path import expanduser

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar_launch_path = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )

    rf2o_launch_path = os.path.join(
        get_package_share_directory('rf2o_laser_odometry'),
        'launch',
        'rf2o_laser_odometry.launch.py'
    )

    desc_launch_path = os.path.join(
        get_package_share_directory('landerpi_description'),
        'launch',
        'desc.launch.py'
    )

    cam_launch_path = os.path.join(
        get_package_share_directory('deptrum-ros-driver-aurora930'),
        'launch',
        'aurora930_launch.py'
    )

    driver_node = Node(
            package='hiwonder_driver',
            executable='hiwonder_driver',
    )

    imu_filter_node = Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            parameters=[{
                'use_mag' : False,
                'publish_tf' : False,
                'do_bias_estimation' : True,
                'publish_debug_topics': False,
            }]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rf2o_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(desc_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cam_launch_path)
        ),
        driver_node,
        imu_filter_node
    ])
