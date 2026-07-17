import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('tof_stvl_test')
    preprocessor_params = os.path.join(
        package_share, 'config', 'tof_ground_preprocessor.yaml')
    tof_package_share = get_package_share_directory('sipeed_tof_ms_a010')
    tof_params = os.path.join(
        tof_package_share, 'config', 'maixsense_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/tof'),
        DeclareLaunchArgument('tof_x', default_value='0.26'),
        DeclareLaunchArgument('tof_y', default_value='0.0'),
        DeclareLaunchArgument('tof_z', default_value='0.22'),
        DeclareLaunchArgument('tof_roll', default_value='-1.5708'),
        DeclareLaunchArgument('tof_pitch', default_value='0.0'),
        DeclareLaunchArgument('tof_yaw', default_value='-1.5708'),

        # Para o teste no PC, base_footprint e um frame fixo no piso. O frame
        # tof fica na altura da lente e usa a orientacao optica real do A010.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_footprint_to_tof',
            arguments=[
                '--x', LaunchConfiguration('tof_x'),
                '--y', LaunchConfiguration('tof_y'),
                '--z', LaunchConfiguration('tof_z'),
                '--roll', LaunchConfiguration('tof_roll'),
                '--pitch', LaunchConfiguration('tof_pitch'),
                '--yaw', LaunchConfiguration('tof_yaw'),
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'tof',
            ],
            output='screen',
        ),
        Node(
            package='sipeed_tof_ms_a010',
            executable='sipeed_tof_node',
            name='sipeed_tof_ms_a010',
            output='screen',
            parameters=[tof_params, {
                'device': LaunchConfiguration('device'),
            }],
        ),
        Node(
            package='tof_stvl_test',
            executable='tof_cloud_preprocessor',
            name='tof_cloud_preprocessor',
            output='screen',
            parameters=[preprocessor_params],
        ),
    ])
