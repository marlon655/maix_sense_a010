import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory('tof_stvl_robot')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    package_share,
                    'launch',
                    'tof_robot_bringup.launch.py',
                )
            ),
            launch_arguments={
                'simulation': 'false',
                'input_topic': '/cloud',
                'target_frame': 'base_footprint',
                'output_frame': 'tof',
                'temporal_required_frames': '1',
                'publish_intermediate_clouds': 'false',
            }.items(),
        ),
    ])
