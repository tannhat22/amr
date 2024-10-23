import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    rmf_schedule = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("amr-rmf"), "launch"),
                "/amr_rmf_schedule.launch.xml",
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "map_name": "tp2-tp3-layout",
            "server_uri": "http://10.7.11.27:8000/_internal",
        }.items(),
    )
    free_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr-rmf"),
                "launch",
                "amr_ff_server.launch.py",
            )
        )
    )
    charger_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr-rmf"),
                "launch",
                "charger_server.launch.py",
            )
        )
    )
    machine_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr-rmf"),
                "launch",
                "machine_server.launch.py",
            )
        )
    )

    ldm_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr-rmf"), "launch", "ldm_server.launch.py"
            )
        )
    )

    amr_workcells = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("amr_workcells_adapter"), "launch"
                ),
                "/workcells_adapter.launch.xml",
            ]
        )
    )

    ldm_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ldm_rmf_adapter"),
                "launch",
                "ldm_rmf.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            rmf_schedule,
            free_fleet,
            charger_server,
            # machine_server,
            ldm_server,
            amr_workcells,
            ldm_adapter,
        ]
    )
