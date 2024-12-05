import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="charger_fleet_server_ros2",
                namespace="",
                executable="charger_fleet_server_ros2",
                name="fleet_charger_server_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "fleet_name": "amr_tayrua",
                        "fleet_state_topic": "fleet_charger_state",
                        "charger_request_topic": "charger_request",
                        "dds_domain": 53,
                        "dds_charger_state_topic": "charger_state",
                        "dds_charger_request_topic": "charger_request",
                        "update_state_frequency": 2.0,
                        "publish_state_frequency": 1.0,
                    }
                ],
            ),
        ]
    )
