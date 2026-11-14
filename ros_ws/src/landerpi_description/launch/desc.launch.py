from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("landerpi_description"))
    description = (share / "urdf/landerpi_mecanum.urdf").read_text()
    sim_time = LaunchConfiguration("use_sim_time")


    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),


        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": sim_time}]),
    ])
