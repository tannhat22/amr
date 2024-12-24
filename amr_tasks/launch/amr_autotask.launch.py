import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_name = "tp2-tp3-layout"
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([FindPackageShare("amr_tasks"), "config.yaml"]),
        description="Path to the configuration file",
    )
    nav_graph_1_arg = DeclareLaunchArgument(
        "nav_graph_1",
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

    nav_graph_2_arg = DeclareLaunchArgument(
        "nav_graph_2",
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
    nav_graph_1_file = LaunchConfiguration("nav_graph_1")
    nav_graph_2_file = LaunchConfiguration("nav_graph_2")

    return LaunchDescription(
        [
            config_file_arg,
            nav_graph_1_arg,
            nav_graph_2_arg,
            Node(
                package="amr_tasks",
                namespace="",
                executable="autotask_manager",
                name="amr_autotask_manager",
                output="screen",
                emulate_tty=True,
                respawn=False,
                arguments=[
                    "--config_file",
                    config_file,
                    "--nav_graph_1",
                    nav_graph_1_file,
                    "--nav_graph_2",
                    nav_graph_2_file,
                ],
            ),
            # Node(
            #     package="amr_tasks",
            #     namespace="",
            #     executable="dispatch_delivery_task",
            #     name="amr_delivery_requester",
            #     output="screen",
            #     emulate_tty=True,
            #     respawn=False,
            # ),
        ]
    )
