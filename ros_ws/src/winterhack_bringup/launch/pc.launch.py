import os
from os.path import expanduser

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('winterhack_bringup')

    navi_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    slam_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    moveit_launch_path = os.path.join(
        get_package_share_directory('moveit_config'),
        'launch',
        'move_group.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(
                    bringup_share, 'slam.yaml'
                )
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navi_launch_path),
            launch_arguments={
                'use_sim_time' : 'false',
                'params_file' : os.path.join(bringup_share, 'nav2_params.yaml')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path),
        )
    ])
