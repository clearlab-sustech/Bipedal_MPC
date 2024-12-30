import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_name = "p1/config.yaml"
    config_file = os.path.join(get_package_share_path("assets"), config_file_name)
    return LaunchDescription(
        [
            Node(
                package="management",
                executable="management",
                name="p1_management",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"/config_file": config_file},
                ],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="p1_joy_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )