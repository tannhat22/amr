import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
   rmf_schedule = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('amr-rmf'), 'launch'),
         '/amr_rmf_schedule.launch.xml']),
         launch_arguments={'map_name': 'lk1-layout',
                           'server_uri': 'http://10.7.11.13:8000/_internal'}.items(),
      )
   free_fleet = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('amr-rmf'), 'launch'),
         '/amr_ff_server.launch.xml'])
      )
   charger_server = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('amr-rmf'), 'launch'),
         '/charger_server.launch.xml']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
   machine_server = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('amr-rmf'), 'launch'),
         '/machine_server.launch.xml'])
      )
   amr_workcells = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('amr_workcells_adapter'), 'launch'),
         '/workcells_adapter.launch.xml'])
      )

   return LaunchDescription([
      rmf_schedule,
      free_fleet,
      charger_server,
      machine_server,
      amr_workcells,
   ])