import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    simulation = LaunchConfiguration('simulation')
    input_topic = LaunchConfiguration('input_topic')
    target_frame = LaunchConfiguration('target_frame')
    output_frame = LaunchConfiguration('output_frame')
    temporal_required_frames = LaunchConfiguration('temporal_required_frames')
    publish_intermediate_clouds = LaunchConfiguration(
        'publish_intermediate_clouds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='When true, disables the physical A010 driver and consumes an externally published PointCloud2.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/cloud',
            description='PointCloud2 topic consumed by the point cloud preprocessor.',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_footprint',
            description='Frame used for filtering before publishing the output cloud.',
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='tof',
            description='Frame used by the point cloud preprocessor output cloud.',
        ),
        DeclareLaunchArgument(
            'temporal_required_frames',
            default_value='2',
            description='Number of consecutive frames required by the temporal filter.',
        ),
        DeclareLaunchArgument(
            'publish_intermediate_clouds',
            default_value='false',
            description='Publish /tof_filters/* debug clouds from each filter stage.',
        ),
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
        LogInfo(condition=IfCondition(simulation), msg='ToF mode: simulation'),
        LogInfo(condition=IfCondition(simulation), msg='Physical A010 driver: disabled'),
        LogInfo(condition=IfCondition(simulation), msg=['PointCloud input: ', input_topic]),
        LogInfo(condition=IfCondition(simulation), msg=['PointCloud target frame: ', target_frame]),
        LogInfo(condition=IfCondition(simulation), msg=['PointCloud output frame: ', output_frame]),
        LogInfo(condition=IfCondition(simulation), msg='use_sim_time: true'),
        LogInfo(condition=UnlessCondition(simulation), msg='ToF mode: hardware'),
        LogInfo(condition=UnlessCondition(simulation), msg='Physical A010 driver: enabled'),
        LogInfo(condition=UnlessCondition(simulation), msg=['A010 device: ', device]),
        LogInfo(condition=UnlessCondition(simulation), msg=['PointCloud input: ', input_topic]),
        LogInfo(condition=UnlessCondition(simulation), msg=['PointCloud target frame: ', target_frame]),
        LogInfo(condition=UnlessCondition(simulation), msg=['PointCloud output frame: ', output_frame]),
        LogInfo(condition=UnlessCondition(simulation), msg='use_sim_time: false'),
        LogInfo(condition=UnlessCondition(simulation), msg=['A010 driver parameters: ', driver_params_file]),
        LogInfo(msg=['ToF preprocessor parameters: ', preprocessor_params_file]),
        Node(
            package='sipeed_tof_ms_a010',
            executable='sipeed_tof_node',
            name='sipeed_tof_ms_a010',
            output='screen',
            condition=UnlessCondition(simulation),
            parameters=[
                driver_params_file,
                {
                    'device': device,
                    'use_sim_time': False,
                },
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='tof_stvl_robot',
            executable='pointcloud_preprocessor',
            name='tof_pointcloud_preprocessor',
            output='screen',
            parameters=[
                preprocessor_params_file,
                {
                    'input_topic': input_topic,
                    'target_frame': target_frame,
                    'output_frame': output_frame,
                    'temporal_required_frames': ParameterValue(
                        temporal_required_frames,
                        value_type=int,
                    ),
                    'publish_intermediate_clouds': ParameterValue(
                        publish_intermediate_clouds,
                        value_type=bool,
                    ),
                    'use_sim_time': ParameterValue(
                        simulation,
                        value_type=bool,
                    ),
                },
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
