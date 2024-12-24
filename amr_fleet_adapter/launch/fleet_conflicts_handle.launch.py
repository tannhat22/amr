import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # config_file = os.path.join(get_package_share_directory("amr_fleet_adapter"), "config.yaml")
    config_file_1_arg = DeclareLaunchArgument(
        "config_file_1",
        default_value=PathJoinSubstitution(
            [FindPackageShare("amr_fleet_adapter"), "tp2_config.yaml"]
        ),
        description="Path to the configuration file",
    )

    config_file_2_arg = DeclareLaunchArgument(
        "config_file_2",
        default_value=PathJoinSubstitution(
            [FindPackageShare("amr_fleet_adapter"), "tp3_config.yaml"]
        ),
        description="Path to the configuration file",
    )

    # Sử dụng giá trị từ argument
    config_file_1 = LaunchConfiguration("config_file_1")
    config_file_2 = LaunchConfiguration("config_file_2")

    return LaunchDescription(
        [
            config_file_1_arg,
            config_file_2_arg,
            Node(
                package="amr_fleet_adapter",
                namespace="",
                executable="fleet_conflicts",
                name="amr_fleet_conflicts_handle",
                output="screen",
                emulate_tty=True,
                respawn=False,
                arguments=["--config_file_1", config_file_1, "--config_file_2", config_file_2],
                parameters=[
                    {
                        "update_frequency": 10.0,
                        "width_conflict": 1.0,
                        "height_conflict": 2.0,
                        "front_extension": 1.0,
                        "debug": True,
                    }
                ],
            ),
        ]
    )
