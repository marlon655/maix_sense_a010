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
    distance_min = LaunchConfiguration('distance_min')
    lateral_min = LaunchConfiguration('lateral_min')
    lateral_max = LaunchConfiguration('lateral_max')
    temporal_required_frames = LaunchConfiguration('temporal_required_frames')
    temporal_reference_frame = LaunchConfiguration('temporal_reference_frame')
    temporal_match_radius = LaunchConfiguration('temporal_match_radius')
    publish_intermediate_clouds = LaunchConfiguration(
        'publish_intermediate_clouds')
    terrain_analysis_enabled = LaunchConfiguration('terrain_analysis_enabled')
    publish_terrain_debug = LaunchConfiguration('publish_terrain_debug')
    terrain_min_points_per_cell = LaunchConfiguration(
        'terrain_min_points_per_cell')
    terrain_min_reliable_points_per_cell = LaunchConfiguration(
        'terrain_min_reliable_points_per_cell')
    terrain_seed_x_min = LaunchConfiguration('terrain_seed_x_min')
    terrain_seed_x_max = LaunchConfiguration('terrain_seed_x_max')
    terrain_seed_half_width = LaunchConfiguration('terrain_seed_half_width')
    terrain_seed_height_tolerance = LaunchConfiguration(
        'terrain_seed_height_tolerance')
    terrain_slope_noise_tolerance = LaunchConfiguration(
        'terrain_slope_noise_tolerance')
    terrain_transition_enabled = LaunchConfiguration(
        'terrain_transition_enabled')
    terrain_transition_max_length = LaunchConfiguration(
        'terrain_transition_max_length')
    terrain_transition_min_forward_cells = LaunchConfiguration(
        'terrain_transition_min_forward_cells')
    terrain_transition_min_lateral_width = LaunchConfiguration(
        'terrain_transition_min_lateral_width')
    terrain_transition_max_plane_residual = LaunchConfiguration(
        'terrain_transition_max_plane_residual')

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
            'distance_min',
            default_value='0.20',
            description='Minimum euclidean distance accepted by the point cloud preprocessor.',
        ),
        DeclareLaunchArgument(
            'lateral_min',
            default_value='-1.10',
            description='Minimum lateral Y accepted by the point cloud preprocessor.',
        ),
        DeclareLaunchArgument(
            'lateral_max',
            default_value='1.10',
            description='Maximum lateral Y accepted by the point cloud preprocessor.',
        ),
        DeclareLaunchArgument(
            'temporal_required_frames',
            default_value='2',
            description='Number of consecutive frames required by the temporal filter.',
        ),
        DeclareLaunchArgument(
            'temporal_reference_frame',
            default_value='odom',
            description='Frame used to compare obstacle persistence between temporal frames.',
        ),
        DeclareLaunchArgument(
            'temporal_match_radius',
            default_value='0.05',
            description='Maximum distance used to match the same obstacle between temporal frames.',
        ),
        DeclareLaunchArgument(
            'publish_intermediate_clouds',
            default_value='false',
            description='Publish /tof_filters/* debug clouds from each filter stage.',
        ),
        DeclareLaunchArgument(
            'terrain_analysis_enabled',
            default_value='false',
            description='Enable terrain/ramp classification before the obstacle height filter.',
        ),
        DeclareLaunchArgument(
            'publish_terrain_debug',
            default_value='false',
            description='Publish /tof/terrain_points and ramp/step debug clouds.',
        ),
        DeclareLaunchArgument(
            'terrain_min_points_per_cell',
            default_value='3',
            description='Minimum points required to build a terrain elevation cell.',
        ),
        DeclareLaunchArgument(
            'terrain_min_reliable_points_per_cell',
            default_value='2',
            description='Minimum points required for a terrain cell to be trusted as seed/support.',
        ),
        DeclareLaunchArgument(
            'terrain_seed_x_min',
            default_value='0.20',
            description='Minimum forward distance used to find initial terrain seed cells.',
        ),
        DeclareLaunchArgument(
            'terrain_seed_x_max',
            default_value='0.45',
            description='Maximum forward distance used to find initial terrain seed cells.',
        ),
        DeclareLaunchArgument(
            'terrain_seed_half_width',
            default_value='0.30',
            description='Half width around the robot centerline used to find terrain seeds.',
        ),
        DeclareLaunchArgument(
            'terrain_seed_height_tolerance',
            default_value='0.05',
            description='Maximum absolute seed height accepted as local terrain.',
        ),
        DeclareLaunchArgument(
            'terrain_slope_noise_tolerance',
            default_value='0.008',
            description='Extra Z tolerance used when comparing neighboring terrain cells.',
        ),
        DeclareLaunchArgument(
            'terrain_transition_enabled',
            default_value='true',
            description='Allow short floor-to-ramp transition recovery when forward terrain support is safe.',
        ),
        DeclareLaunchArgument(
            'terrain_transition_max_length',
            default_value='0.10',
            description='Maximum forward length of a recoverable floor-to-ramp transition.',
        ),
        DeclareLaunchArgument(
            'terrain_transition_min_forward_cells',
            default_value='3',
            description='Minimum reliable cells ahead required to support transition recovery.',
        ),
        DeclareLaunchArgument(
            'terrain_transition_min_lateral_width',
            default_value='0.20',
            description='Minimum lateral support width required to treat a transition as a ramp.',
        ),
        DeclareLaunchArgument(
            'terrain_transition_max_plane_residual',
            default_value='0.025',
            description='Maximum residual accepted when fitting the forward ramp support plane.',
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
                    'distance_min': ParameterValue(
                        distance_min,
                        value_type=float,
                    ),
                    'lateral_min': ParameterValue(
                        lateral_min,
                        value_type=float,
                    ),
                    'lateral_max': ParameterValue(
                        lateral_max,
                        value_type=float,
                    ),
                    'temporal_required_frames': ParameterValue(
                        temporal_required_frames,
                        value_type=int,
                    ),
                    'temporal_reference_frame': temporal_reference_frame,
                    'temporal_match_radius': ParameterValue(
                        temporal_match_radius,
                        value_type=float,
                    ),
                    'publish_intermediate_clouds': ParameterValue(
                        publish_intermediate_clouds,
                        value_type=bool,
                    ),
                    'terrain_analysis_enabled': ParameterValue(
                        terrain_analysis_enabled,
                        value_type=bool,
                    ),
                    'publish_terrain_debug': ParameterValue(
                        publish_terrain_debug,
                        value_type=bool,
                    ),
                    'terrain_min_points_per_cell': ParameterValue(
                        terrain_min_points_per_cell,
                        value_type=int,
                    ),
                    'terrain_min_reliable_points_per_cell': ParameterValue(
                        terrain_min_reliable_points_per_cell,
                        value_type=int,
                    ),
                    'terrain_seed_x_min': ParameterValue(
                        terrain_seed_x_min,
                        value_type=float,
                    ),
                    'terrain_seed_x_max': ParameterValue(
                        terrain_seed_x_max,
                        value_type=float,
                    ),
                    'terrain_seed_half_width': ParameterValue(
                        terrain_seed_half_width,
                        value_type=float,
                    ),
                    'terrain_seed_height_tolerance': ParameterValue(
                        terrain_seed_height_tolerance,
                        value_type=float,
                    ),
                    'terrain_slope_noise_tolerance': ParameterValue(
                        terrain_slope_noise_tolerance,
                        value_type=float,
                    ),
                    'terrain_transition_enabled': ParameterValue(
                        terrain_transition_enabled,
                        value_type=bool,
                    ),
                    'terrain_transition_max_length': ParameterValue(
                        terrain_transition_max_length,
                        value_type=float,
                    ),
                    'terrain_transition_min_forward_cells': ParameterValue(
                        terrain_transition_min_forward_cells,
                        value_type=int,
                    ),
                    'terrain_transition_min_lateral_width': ParameterValue(
                        terrain_transition_min_lateral_width,
                        value_type=float,
                    ),
                    'terrain_transition_max_plane_residual': ParameterValue(
                        terrain_transition_max_plane_residual,
                        value_type=float,
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
