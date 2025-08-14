import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # config_file = os.path.join(get_package_share_directory("amr_fleet_adapter"), "config.yaml")
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("amr_charging_manager"), "config.yaml"]
        ),
        description="Path to the configuration file",
    )

    # Sử dụng giá trị từ argument
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            config_file_arg,
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
                ],
                parameters=[
                    {
                        "update_frequency": 0.2,
                        "min_charge_time": 30.0,
                        "debug": True,
                    }
                ],
            ),
        ]
    )
