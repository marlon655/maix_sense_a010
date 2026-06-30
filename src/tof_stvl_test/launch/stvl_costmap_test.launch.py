import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_stvl_test')
    params = os.path.join(pkg_share, 'config', 'stvl_costmap_test.yaml')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen',
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='local_costmap',
            name='local_costmap',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_test',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['/local_costmap/local_costmap'],
            }],
        ),
        Node(
            package='tof_stvl_test',
            executable='fake_obstacle_cloud',
            name='fake_obstacle_cloud',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'topic': '/ground_segmentation/obstacle_points',
                'distance_x': 0.50,
                'center_z': 0.35,
            }],
        ),
    ])
