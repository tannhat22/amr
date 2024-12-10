from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_sim_time = "false"
    map_name = "tp2-tp3-layout"
    server_uri = "http://10.7.11.9:8000/_internal"
    enable_experimental_lift_watchdog = "true"
    nav_graph_file_path = PathJoinSubstitution(
        [
            FindPackageShare("amr-rmf"),
            "maps",
            map_name,
            "nav_graphs",
            "0.yaml",
        ]
    )

    return LaunchDescription(
        [
            # Common launch
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("amr-rmf"), "launch", "common.launch.xml"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "viz_config_file": PathJoinSubstitution(
                        [FindPackageShare("amr-rmf"), "rviz_config", f"{map_name}.rviz"]
                    ),
                    "config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("amr-rmf"),
                            "maps",
                            f"{map_name}.building.yaml",
                        ]
                    ),
                    "config_dynamic_charge_file": PathJoinSubstitution(
                        [FindPackageShare("amr_fleet_adapter"), "charge_schedule.yaml"]
                    ),
                    "server_uri": server_uri,
                }.items(),
            ),
            # Experimental lift watchdog group (enabled)
            GroupAction(
                condition=IfCondition(enable_experimental_lift_watchdog),
                actions=[
                    Node(
                        package="rmf_fleet_adapter",
                        executable="experimental_lift_watchdog",
                        name="experimental_lift_watchdog",
                        output="both",
                    ),
                    SetEnvironmentVariable(
                        name="EXPT_LIFT_WATCHDOG_SRV",
                        value="experimental_lift_watchdog",
                    ),
                ],
                scoped=False,
            ),
            # Experimental lift watchdog group (disabled)
            GroupAction(
                condition=UnlessCondition(enable_experimental_lift_watchdog),
                actions=[SetEnvironmentVariable(name="EXPT_LIFT_WATCHDOG_SRV", value="")],
                scoped=False,
            ),
            # AMR Tayrua fleet adapter
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr_fleet_adapter"),
                            "launch",
                            "fleet_adapter.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "config_file": PathJoinSubstitution(
                        [FindPackageShare("amr_fleet_adapter"), "config.yaml"]
                    ),
                    "nav_graph_file": nav_graph_file_path,
                    "server_uri": server_uri,
                    "experimental_lift_watchdog_service": EnvironmentVariable(
                        "EXPT_LIFT_WATCHDOG_SRV"
                    ),
                }.items(),
            ),
            # Fleet conflicts handle
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr_fleet_adapter"),
                            "launch",
                            "fleet_conflicts_handle.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config_file": PathJoinSubstitution(
                        [FindPackageShare("amr_fleet_adapter"), "config.yaml"]
                    )
                }.items(),
            ),
            # AMR Fleet server
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr-rmf"),
                            "launch",
                            "amrfleet_server.launch.py",
                        ]
                    )
                )
            ),
            # Charger server
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("amr-rmf"),
            #                 "launch",
            #                 "charger_server.launch.py",
            #             ]
            #         )
            #     )
            # ),
            # Machine server
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr-rmf"),
                            "launch",
                            "machine_server.launch.py",
                        ]
                    )
                )
            ),
            # LDM server
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("amr-rmf"), "launch", "ldm_server.launch.py"]
                    )
                )
            ),
            # Workcell adapter
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr_workcell_adapter"),
                            "launch",
                            "workcell_adapter.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "config_file": PathJoinSubstitution(
                        [FindPackageShare("amr_workcell_adapter"), "config.yaml"]
                    ),
                    "nav_graph_file": nav_graph_file_path,
                }.items(),
            ),
            # LDM RMF adapter
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ldm_rmf_adapter"),
                            "launch",
                            "ldm_rmf.launch.py",
                        ]
                    )
                )
            ),
        ]
    )
