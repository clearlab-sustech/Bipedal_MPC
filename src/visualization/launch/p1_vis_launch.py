import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    urdf_path = os.path.join(
        get_package_share_directory("assets"),
        "p1/PF_A.urdf",
    )
    print("model file: ", urdf_path)
    urdf = open(urdf_path).read()

    rviz_config = os.path.join(
        get_package_share_directory("visualization"),
        "config/config.rviz",
    )

    config_file_name = "p1/config.yaml"
    config_file = os.path.join(get_package_share_directory("assets"), config_file_name)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # Node(
            #     package="visualization",
            #     executable="visualization",
            #     name="aliengo_visualization",
            #     output="screen",
            #     emulate_tty=True,
            #     arguments=[config_file, ("__log_level:=debug")],
            # ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="aliengo_state_publisher_rivz_vis",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": urdf},
                    {"publish_frequency": 1000.0},
                ],
            ),
            # Node(
            #     package="joint_state_publisher",
            #     executable="joint_state_publisher",
            #     parameters=[
            #         {"robot_description": urdf},
            #     ],
            #     arguments=[urdf_path],
            # ),
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            # ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
