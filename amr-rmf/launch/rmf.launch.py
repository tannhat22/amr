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
    server_uri = "http://10.7.11.28:8000/_internal"
    enable_experimental_lift_watchdog = "true"
    nav_graph_tp2_file_path = PathJoinSubstitution(
        [
            FindPackageShare("amr-rmf"),
            "maps",
            map_name,
            "nav_graphs",
            "0.yaml",
        ]
    )
    nav_graph_tp3_file_path = PathJoinSubstitution(
        [
            FindPackageShare("amr-rmf"),
            "maps",
            map_name,
            "nav_graphs",
            "1.yaml",
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
            # # AMR_TP2 fleet adapter
            # IncludeLaunchDescription(
            #     XMLLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("amr_fleet_adapter"),
            #                 "launch",
            #                 "fleet_adapter.launch.xml",
            #             ]
            #         )
            #     ),
            #     launch_arguments={
            #         "use_sim_time": use_sim_time,
            #         "config_file": PathJoinSubstitution(
            #             [FindPackageShare("amr_fleet_adapter"), "tp2_config.yaml"]
            #         ),
            #         "nav_graph_file": nav_graph_tp2_file_path,
            #         "server_uri": server_uri,
            #         "experimental_lift_watchdog_service": EnvironmentVariable(
            #             "EXPT_LIFT_WATCHDOG_SRV"
            #         ),
            #     }.items(),
            # ),
            # # AMR_TP3 fleet adapter
            # IncludeLaunchDescription(
            #     XMLLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("amr_fleet_adapter"),
            #                 "launch",
            #                 "fleet_adapter.launch.xml",
            #             ]
            #         )
            #     ),
            #     launch_arguments={
            #         "use_sim_time": use_sim_time,
            #         "config_file": PathJoinSubstitution(
            #             [FindPackageShare("amr_fleet_adapter"), "tp3_config.yaml"]
            #         ),
            #         "nav_graph_file": nav_graph_tp3_file_path,
            #         "server_uri": server_uri,
            #         "experimental_lift_watchdog_service": EnvironmentVariable(
            #             "EXPT_LIFT_WATCHDOG_SRV"
            #         ),
            #     }.items(),
            # )
            #### AMR_TP2-3 fleet adapter
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
                        [FindPackageShare("amr_fleet_adapter"), "tp23_config.yaml"]
                    ),
                    "nav_graph_file": nav_graph_tp2_file_path,
                    "server_uri": server_uri,
                    "experimental_lift_watchdog_service": EnvironmentVariable(
                        "EXPT_LIFT_WATCHDOG_SRV"
                    ),
                }.items(),
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
                    "nav_graph_1_file": nav_graph_tp2_file_path,
                    "nav_graph_2_file": "",
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
                    "config_file_1": PathJoinSubstitution(
                        [FindPackageShare("amr_fleet_adapter"), "tp23_config.yaml"]
                    ),
                    "config_file_2": "",
                }.items(),
            ),
            # # AMR_TP2 Fleet server
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("amr-rmf"),
            #                 "launch",
            #                 "amr_tp2_fleet_server.launch.py",
            #             ]
            #         )
            #     )
            # ),
            # # AMR_TP3 Fleet server
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("amr-rmf"),
            #                 "launch",
            #                 "amr_tp3_fleet_server.launch.py",
            #             ]
            #         )
            #     )
            # ),
            #### Fleet server
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr-rmf"),
                            "launch",
                            "fleet_server.launch.py",
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
            ## Autotask
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("amr_tasks"),
            #                 "launch",
            #                 "amr_autotask.launch.py",
            #             ]
            #         )
            #     ),
            #     launch_arguments={
            #         "config_file": PathJoinSubstitution(
            #             [FindPackageShare("amr_tasks"), "config.yaml"]
            #         ),
            #         "nav_graph_1_file": nav_graph_tp2_file_path,
            #         "nav_graph_2_file": nav_graph_tp3_file_path,
            #     }.items(),
            # ),
        ]
    )
