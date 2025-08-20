import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_name = "tp23-layout"
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("amr_charging_manager"), "config.yaml"]
        ),
        description="Path to the configuration file",
    )

    nav_graph_1_file_arg = DeclareLaunchArgument(
        "nav_graph_1_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("amr-rmf"),
                "maps",
                map_name,
                "nav_graphs",
                "0.yaml",
            ]
        ),
        description="Path to the nav_graph_1 file",
    )

    nav_graph_2_file_arg = DeclareLaunchArgument(
        "nav_graph_2_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("amr-rmf"),
                "maps",
                map_name,
                "nav_graphs",
                "1.yaml",
            ]
        ),
        description="Path to the nav_graph_2 file",
    )

    # Sử dụng giá trị từ argument
    config_file = LaunchConfiguration("config_file")
    nav_graph_1_file = LaunchConfiguration("nav_graph_1_file")
    nav_graph_2_file = LaunchConfiguration("nav_graph_2_file")

    return LaunchDescription(
        [
            config_file_arg,
            nav_graph_1_file_arg,
            nav_graph_2_file_arg,
            Node(
                package="amr_charging_manager",
                namespace="",
                executable="charging_manager",
                name="charging_manager",
                output="screen",
                emulate_tty=True,
                respawn=False,
                arguments=[
                    "--config_file",
                    config_file,
                    "--nav_graph_1_file",
                    nav_graph_1_file,
                    "--nav_graph_2_file",
                    nav_graph_2_file,
                ],
                parameters=[
                    {
                        "update_frequency": 0.2,
                        "min_charge_time": 30.0,
                        "mutex_graph": "zone_RF370CB",
                        "debug": True,
                    }
                ],
            ),
        ]
    )
