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
                'simulation': 'true',
                'input_topic': '/cloud',
                'target_frame': 'base_footprint',
                'output_frame': 'tof_lidar_frame',
                'distance_min': '0.0',
                'lateral_min': '-0.28',
                'lateral_max': '0.28',
                'temporal_required_frames': '2',
                'temporal_reference_frame': 'base_footprint',
                'temporal_match_radius': '0.08',
                'publish_intermediate_clouds': 'true',
                'terrain_analysis_enabled': 'true',
                'publish_terrain_debug': 'true',
                'terrain_min_points_per_cell': '1',
                'terrain_seed_x_min': '0.0',
                'terrain_seed_x_max': '1.20',
                'terrain_seed_half_width': '0.55',
                'terrain_seed_height_tolerance': '0.12',
                'terrain_slope_noise_tolerance': '0.03',
            }.items(),
        ),
    ])
