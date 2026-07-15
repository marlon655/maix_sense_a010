import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_stvl_test')
    test_params = os.path.join(
        pkg_share, 'config', 'tof_pointcloud_filters.yaml')
    urdf = os.path.join(pkg_share, 'urdf', 'tof_maix_sense_a010.urdf')
    tof_pkg_share = get_package_share_directory('sipeed_tof_ms_a010')
    tof_params = os.path.join(tof_pkg_share, 'config', 'maixsense_params.yaml')

    with open(urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/tof'),
        DeclareLaunchArgument('tof_x', default_value='0.26'),
        DeclareLaunchArgument('tof_y', default_value='0.0'),
        DeclareLaunchArgument('tof_z', default_value='0.22'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_footprint',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'odom', '--child-frame-id', 'base_footprint',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_footprint_to_tof',
            arguments=[
                '--x', LaunchConfiguration('tof_x'),
                '--y', LaunchConfiguration('tof_y'),
                '--z', LaunchConfiguration('tof_z'),
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'base_footprint', '--child-frame-id', 'tof',
            ],
            output='screen',
        ),
        Node(
            package='sipeed_tof_ms_a010',
            executable='sipeed_tof_node',
            name='sipeed_tof_ms_a010',
            output='screen',
            parameters=[tof_params, {'device': LaunchConfiguration('device')}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='tof_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='tof_stvl_test',
            executable='pointcloud_preprocessor',
            name='tof_pointcloud_preprocessor',
            output='screen',
            parameters=[test_params],
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='local_costmap',
            name='local_costmap',
            output='screen',
            parameters=[test_params],
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
                'bond_timeout': 0.0,
            }],
        ),
    ])
