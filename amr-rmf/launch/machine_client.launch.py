import os

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file = PathJoinSubstitution([FindPackageShare("machine_server_ros2"), "config.yaml"])
    return LaunchDescription(
        [
            Node(
                package="machine_server_ros2",
                namespace="",
                executable="machine_server_keyence",
                name="machine_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", config_file],
            ),
            Node(
                package="machine_server_ros2",
                namespace="",
                executable="machine_state_update_keyence",
                name="machine_state_update",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", config_file],
            ),
            Node(
                package="machine_fleet_client_ros2",
                namespace="",
                executable="machine_fleet_client_ros2",
                name="fleet_machine_client_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "machine_name": "nqvlm104",
                        "machine_state_topic": "/nqvlm104_machine_state",
                        "station_request_topic": "/nqvlm104_station_request",
                        "machine_service_name": "/nqvlm104_server",
                        "dds_domain": 52,
                        "dds_state_topic": "machine_state",
                        "dds_machine_request_topic": "machine_request",
                        "dds_station_request_topic": "station_request",
                        "update_frequency": 5.0,
                        "publish_frequency": 1.0,
                    }
                ],
            ),
        ]
    )
