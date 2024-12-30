import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # config_file_name = "aliengo/config.yaml"
    # config_file = os.path.join(get_package_share_path("assets"), config_file_name)
    # # print(config_file)
    # return LaunchDescription(
    #     [
    #         Node(
    #             package="sim",
    #             executable="sim",
    #             name="simulation_mujoco",
    #             output="screen",
    #             emulate_tty=True,
    #             arguments=[config_file, ("__log_level:=debug")],
    #         ),
    #     ]
    # )

    config_file_name = "p1/config.yaml"
    config_file = os.path.join(get_package_share_path("assets"), config_file_name)
    print(config_file)
    return LaunchDescription(
        [
            Node(
                package="sim",
                executable="sim",
                name="simulation_mujoco",
                output="screen",
                emulate_tty=True,
                arguments=[config_file, ("__log_level:=debug")],
            ),
        ]
    )