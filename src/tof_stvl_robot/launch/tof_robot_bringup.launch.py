import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_default = os.path.join(
        get_package_share_directory('sipeed_tof_ms_a010'),
        'config',
        'maixsense_params.yaml',
    )
    preprocessor_default = os.path.join(
        get_package_share_directory('tof_stvl_robot'),
        'config',
        'tof_robot_preprocessor.yaml',
    )

    device = LaunchConfiguration('device')
    driver_params_file = LaunchConfiguration('driver_params_file')
    preprocessor_params_file = LaunchConfiguration('preprocessor_params_file')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/tof',
            description='Serial device used by the MaixSense A010 driver.',
        ),
        DeclareLaunchArgument(
            'driver_params_file',
            default_value=driver_default,
            description='Installed YAML file for the A010 driver.',
        ),
        DeclareLaunchArgument(
            'preprocessor_params_file',
            default_value=preprocessor_default,
            description='Installed YAML file for point cloud preprocessing.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level applied to both production nodes.',
        ),
        LogInfo(msg=['A010 serial device: ', device]),
        LogInfo(msg=['A010 driver parameters: ', driver_params_file]),
        LogInfo(msg=['ToF preprocessor parameters: ', preprocessor_params_file]),
        Node(
            package='sipeed_tof_ms_a010',
            executable='sipeed_tof_node',
            name='sipeed_tof_ms_a010',
            output='screen',
            parameters=[
                driver_params_file,
                {'device': device},
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='tof_stvl_robot',
            executable='pointcloud_preprocessor',
            name='tof_pointcloud_preprocessor',
            output='screen',
            parameters=[preprocessor_params_file],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
