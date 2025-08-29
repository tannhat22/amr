import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="machine_fleet_server_ros2",
                namespace="",
                executable="machine_fleet_server_ros2",
                name="fleet_machine_server_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "fleet_name": "station",
                        "machine_names": ["D241", "D242"],
                        "fleet_state_topic": "fleet_machine_state",
                        "machine_request_topic": "machine_requests",
                        "station_request_topic": "adapter_station_requests",
                        "dds_domain": 52,
                        "dds_machine_state_topic": "machine_state",
                        "dds_machine_request_topic": "machine_request",
                        "dds_station_request_topic": "station_request",
                        "update_state_frequency": 2.0,
                        "publish_state_frequency": 1.0,
                    }
                ],
            ),
        ]
    )
