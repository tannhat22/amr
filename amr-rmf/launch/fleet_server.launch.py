import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="free_fleet_server_ros2",
                namespace="",
                executable="free_fleet_server_ros2",
                name="amr_lk1_fleet_server_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "fleet_name": "amr_lk1",
                        "robot_names": ["amr001"],
                        "fleet_state_topic": "fleet_states",
                        "mode_request_topic": "robot_mode_requests",
                        "path_request_topic": "robot_path_requests",
                        "destination_request_topic": "robot_destination_requests",
                        "dock_request_topic": "robot_dock_requests",
                        "cancel_request_topic": "robot_cancel_requests",
                        "localize_request_topic": "robot_localize_requests",
                        "dds_domain": 42,
                        "dds_robot_state_topic": "robot_state",
                        "dds_mode_request_topic": "mode_request",
                        "dds_path_request_topic": "path_request",
                        "dds_destination_request_topic": "destination_request",
                        "dds_dock_request_topic": "dock_request",
                        "dds_cancel_request_topic": "cancel_request",
                        "dds_localize_request_topic": "localize_request",
                        "update_state_frequency": 10.0,
                        "publish_state_frequency": 2.0,
                        "rotation": [0.0, 0.0],
                        "scale": [1.0, 1.0],
                        "translation_x": [-9.52, -9.52],
                        "translation_y": [45.08, 45.08],
                    }
                ],
            ),
        ]
    )
