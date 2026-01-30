import os

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    machine_config = PathJoinSubstitution(
        [FindPackageShare("machine_server_ros2"), "config.yaml"]
    )
    lift_config = PathJoinSubstitution(
        [FindPackageShare("ldm_server_ros2"), "config.yaml"]
    )
    return LaunchDescription(
        [
            # Node for lift client -> LIFT-001
            Node(
                package="ldm_server_ros2",
                namespace="",
                executable="lift_server",
                name="lift_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", lift_config],
            ),
            Node(
                package="ldm_server_ros2",
                namespace="",
                executable="lift_state_update",
                name="lift_state_update",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", lift_config],
            ),
            Node(
                package="ldm_fleet_client_ros2",
                namespace="",
                executable="ldm_fleet_client_ros2",
                name="ldm_fleet_client_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "lift_name": "LIFT-001",
                        "lift_state_topic": "/lift_state",
                        "register_lift_topic": "/register_lift",
                        "lift_trigger_server_name": "/lift_server",
                        "dds_domain": 82,
                        "dds_state_topic": "lift_state",
                        "dds_lift_request_topic": "lift_request",
                        "update_frequency": 5.0,
                        "publish_frequency": 1.0,
                    }
                ],
            ),
            # Node for machine client RF370CB - TP3
            Node(
                package="machine_server_ros2",
                namespace="",
                executable="machine_server_mitsu",
                name="machine_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", machine_config],
            ),
            Node(
                package="machine_server_ros2",
                namespace="",
                executable="machine_state_update_mitsu",
                name="machine_state_update",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", machine_config],
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
                        "machine_name": "station_rf370cb_tp3",
                        "machine_state_topic": "/station_rf370cb_tp3_machine_state",
                        "station_request_topic": "/station_rf370cb_tp3_station_request",
                        "machine_service_name": "/station_rf370cb_tp3_server",
                        "dds_domain": 53,
                        "dds_state_topic": "machine_state",
                        "dds_machine_request_topic": "machine_request",
                        "dds_station_request_topic": "station_request",
                        "update_frequency": 5.0,
                        "publish_frequency": 1.0,
                    }
                ],
            ),
            # Node for features system alarm
            Node(
                package="machine_server_ros2",
                namespace="",
                executable="rmf_alarm",
                name="rmf_alarm",
                output="screen",
                emulate_tty=True,
                respawn=True,
            ),
        ]
    )
