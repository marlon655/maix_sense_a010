import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def _as_launch_bool(value):
    return 'true' if bool(value) else 'false'


def _resolve_package_path(package_share, value):
    if value is None:
        return ''

    value = os.path.expanduser(os.path.expandvars(str(value)))
    if value == '' or os.path.isabs(value):
        return value

    return os.path.join(package_share, value)


def _load_launch_defaults(package_share):
    config_path = os.path.join(package_share, 'config', 'launch_params.yaml')
    defaults = {}

    if os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as config_file:
            defaults = yaml.safe_load(config_file) or {}

    return defaults


def _launch_mode(context, *args, **kwargs):
    nav_hub_share = get_package_share_directory('nav_hub')

    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    route = LaunchConfiguration('route').perform(context)
    speed_filter = LaunchConfiguration('speed_filter').perform(context)

    if mode == 'simulation':
        sim_nav_graph = os.path.join(
            nav_hub_share, 'launch', 'route_graph', 'sim_nav_graph.launch.py')

        return [
            SetParameter(name='use_sim_time', value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_nav_graph),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'route': route,
                    'speed_filter': speed_filter,
                }.items()),
        ]

    if mode == 'hardware':
        robot_description_launch = os.path.join(
            FindPackageShare('robot06_description').find('robot06_description'),
            'launch',
            'display.launch.py')

        lidar_launch = os.path.join(
            FindPackageShare('ldlidar_stl_ros2').find('ldlidar_stl_ros2'),
            'launch',
            'stl27l.launch.py')

        canopen_launch = os.path.join(
            FindPackageShare('canopen_ros').find('canopen_ros'),
            'launch',
            'canopen.launch.py')

        nav_graph_launch = os.path.join(
            nav_hub_share, 'launch', 'route_graph', 'nav_graph.launch.py')

        return [
            SetParameter(name='use_sim_time', value=False),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_description_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canopen_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_graph_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items()),
        ]

    return [
        SetLaunchConfiguration('mode_error', f'Invalid mode: {mode}. Use simulation or hardware.'),
    ]


def generate_launch_description():
    nav_hub_share = get_package_share_directory('nav_hub')
    launch_defaults = _load_launch_defaults(nav_hub_share)

    default_params_file = _resolve_package_path(
        nav_hub_share,
        launch_defaults.get('params_file', os.path.join(nav_hub_share, 'config', 'nav_routegraph.yaml')))

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value=str(launch_defaults.get('mode', 'hardware')),
        description='Operation mode: hardware or simulation')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=_as_launch_bool(launch_defaults.get('use_sim_time', False)),
        description='Use simulation clock')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file')

    declare_route = DeclareLaunchArgument(
        'route',
        default_value=_as_launch_bool(launch_defaults.get('route', True)),
        description='Start nav2_route route_server in simulation mode')

    declare_speed_filter = DeclareLaunchArgument(
        'speed_filter',
        default_value=_as_launch_bool(launch_defaults.get('speed_filter', True)),
        description='Start Nav2 speed filter servers in simulation mode')

    return LaunchDescription([
        declare_mode,
        declare_use_sim_time,
        declare_params_file,
        declare_route,
        declare_speed_filter,
        OpaqueFunction(function=_launch_mode),
    ])
