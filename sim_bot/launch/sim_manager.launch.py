import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _as_launch_bool(value):
    return 'True' if bool(value) else 'False'


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


def _mode_is(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def generate_launch_description():
    package_name = 'sim_bot'
    package_share = get_package_share_directory(package_name)
    launch_defaults = _load_launch_defaults(package_share)

    mode = LaunchConfiguration('mode')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_urdf = LaunchConfiguration('robot_urdf')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    bridge_config = LaunchConfiguration('bridge_config')
    rviz_config = LaunchConfiguration('rviz_config')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    joy = LaunchConfiguration('joy')
    tof_lidar = LaunchConfiguration('tof_lidar')
    camera = LaunchConfiguration('camera')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    nav_params_file = LaunchConfiguration('nav_params_file')
    route = LaunchConfiguration('route')
    speed_filter = LaunchConfiguration('speed_filter')

    world_default = _resolve_package_path(
        package_share,
        launch_defaults.get('world', os.path.join(package_share, 'worlds', 'test.world')))
    robot_urdf_default = _resolve_package_path(
        package_share,
        launch_defaults.get('robot_urdf', os.path.join(package_share, 'description', 'palmares_bot.urdf.xacro')))
    bridge_config_default = _resolve_package_path(
        package_share,
        launch_defaults.get('bridge_config', os.path.join(package_share, 'config', 'gz_bridge.yaml')))
    rviz_config_default = _resolve_package_path(
        package_share,
        launch_defaults.get('rviz_config', os.path.join(package_share, 'rviz', 'bot.rviz')))
    nav_hub_share = get_package_share_directory('nav_hub')
    nav_params_file_default = _resolve_package_path(
        nav_hub_share,
        launch_defaults.get('nav_params_file', os.path.join(nav_hub_share, 'config', 'sim_nav_params.yaml')))

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value=str(launch_defaults.get('mode', 'simulation')),
        description='Operation mode: simulation or hardware')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_default,
        description='Full path to the world model file to load')

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value=str(launch_defaults.get('robot_name', 'palmares_bot')),
        description='Robot name used when spawning in Gazebo')

    declare_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        default_value=robot_urdf_default,
        description='Full path to the robot URDF/Xacro file')

    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value=str(launch_defaults.get('spawn_x', 0.0)),
        description='Initial Gazebo spawn X position')

    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value=str(launch_defaults.get('spawn_y', 0.0)),
        description='Initial Gazebo spawn Y position')

    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value=str(launch_defaults.get('spawn_z', 0.15)),
        description='Initial Gazebo spawn Z position')

    declare_bridge_config = DeclareLaunchArgument(
        'bridge_config',
        default_value=bridge_config_default,
        description='Full path to the ros_gz_bridge YAML config')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_default,
        description='Full path to the RViz config file')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value=_as_launch_bool(launch_defaults.get('headless', False)),
        description='Run Gazebo headless if true')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value=_as_launch_bool(launch_defaults.get('rviz', True)),
        description='Open RViz if true')

    declare_joy = DeclareLaunchArgument(
        'joy',
        default_value=_as_launch_bool(launch_defaults.get('joy', True)),
        description='Enable joystick tele-operation if true')

    declare_tof_lidar = DeclareLaunchArgument(
        'tof_lidar',
        default_value=_as_launch_bool(launch_defaults.get('tof_lidar', True)),
        description='Enable the simulated 3D ToF lidar point cloud sensor')

    declare_camera = DeclareLaunchArgument(
        'camera',
        default_value=_as_launch_bool(launch_defaults.get('camera', True)),
        description='Enable the simulated RGB camera sensor')

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value=_as_launch_bool(launch_defaults.get('slam', True)),
        description='Start slam_toolbox if true')

    declare_nav = DeclareLaunchArgument(
        'nav',
        default_value=_as_launch_bool(launch_defaults.get('nav', True)),
        description='Start Nav2 navigation stack if true')

    declare_nav_params_file = DeclareLaunchArgument(
        'nav_params_file',
        default_value=nav_params_file_default,
        description='Full path to the Nav2 parameters file')

    declare_route = DeclareLaunchArgument(
        'route',
        default_value=_as_launch_bool(launch_defaults.get('route', False)),
        description='Start nav2_route route_server if true')

    declare_speed_filter = DeclareLaunchArgument(
        'speed_filter',
        default_value=_as_launch_bool(launch_defaults.get('speed_filter', False)),
        description='Start Nav2 speed filter servers if true')

    sim_essentials = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'sim_essentials.launch.py')),
        launch_arguments={
            'world': world,
            'robot_name': robot_name,
            'robot_urdf': robot_urdf,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'bridge_config': bridge_config,
            'rviz_config': rviz_config,
            'headless': headless,
            'rviz': rviz,
            'joy': joy,
            'tof_lidar': tof_lidar,
            'camera': camera,
            'use_sim_time': 'true',
        }.items())

    sim_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'sim_navigation.launch.py')),
        launch_arguments={
            'slam': slam,
            'nav': nav,
            'nav_params_file': nav_params_file,
            'route': route,
            'speed_filter': speed_filter,
            'use_sim_time': 'true',
        }.items())

    simulation_stack = GroupAction(
        condition=_mode_is(mode, 'simulation'),
        actions=[
            sim_essentials,
            sim_navigation,
        ])

    hardware_placeholder = GroupAction(
        condition=_mode_is(mode, 'hardware'),
        actions=[
            LogInfo(
                msg=[
                    'mode:=hardware ainda nao foi implementado neste pacote. ',
                    'Use mode:=simulation para o baseline Gazebo atual.',
                ])
        ])

    return LaunchDescription([
        declare_mode,
        declare_world,
        declare_robot_name,
        declare_robot_urdf,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_bridge_config,
        declare_rviz_config,
        declare_headless,
        declare_rviz,
        declare_joy,
        declare_tof_lidar,
        declare_camera,
        declare_slam,
        declare_nav,
        declare_nav_params_file,
        declare_route,
        declare_speed_filter,
        simulation_stack,
        hardware_placeholder,
    ])
