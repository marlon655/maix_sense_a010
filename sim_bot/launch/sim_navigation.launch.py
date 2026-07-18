import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'sim_bot'
    package_share = get_package_share_directory(package_name)
    nav_hub_share = get_package_share_directory('nav_hub')

    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    nav_params_file = LaunchConfiguration('nav_params_file')
    route = LaunchConfiguration('route')
    speed_filter = LaunchConfiguration('speed_filter')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Start slam_toolbox if true')

    declare_nav = DeclareLaunchArgument(
        'nav',
        default_value='True',
        description='Start Nav2 navigation stack if true')

    declare_nav_params_file = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(nav_hub_share, 'config', 'sim_nav_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_route = DeclareLaunchArgument(
        'route',
        default_value='False',
        description='Start nav2_route route_server if true')

    declare_speed_filter = DeclareLaunchArgument(
        'speed_filter',
        default_value='False',
        description='Start Nav2 speed filter servers if true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    slam_node = GroupAction(
        condition=IfCondition(slam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, 'launch', 'slam.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items())
        ])

    nav_node = GroupAction(
        condition=IfCondition(nav),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_hub_share, 'launch', 'essentials.launch.py')),
                launch_arguments={
                    'mode': 'simulation',
                    'use_sim_time': use_sim_time,
                    'params_file': nav_params_file,
                    'route': route,
                    'speed_filter': speed_filter,
                }.items())
        ])

    return LaunchDescription([
        declare_slam,
        declare_nav,
        declare_nav_params_file,
        declare_route,
        declare_speed_filter,
        declare_use_sim_time,
        slam_node,
        nav_node,
    ])
