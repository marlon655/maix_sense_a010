import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_hub_share = get_package_share_directory('nav_hub')

    route_file = LaunchConfiguration('route_file')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    enable_status_topics = LaunchConfiguration('enable_status_topics')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_route_file = DeclareLaunchArgument(
        'route_file',
        default_value=os.path.join(nav_hub_share, 'route', 'route_graph', 'goals_aceleradoras.json'),
        description='JSON de destinos operacionais usado pelo sim_main_route_graph')

    declare_publish_initial_pose = DeclareLaunchArgument(
        'publish_initial_pose',
        default_value='false',
        description='Publica initialpose usando o ponto initial do JSON')

    declare_enable_status_topics = DeclareLaunchArgument(
        'enable_status_topics',
        default_value='true',
        description='Publica has_reached e navegando para simular status operacional')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    sim_main_route_graph = Node(
        package='nav_hub',
        executable='sim_main_route_graph',
        name='sim_main_route_graph',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'route_file': route_file,
            'publish_initial_pose': publish_initial_pose,
            'enable_status_topics': enable_status_topics,
        }])

    return LaunchDescription([
        declare_route_file,
        declare_publish_initial_pose,
        declare_enable_status_topics,
        declare_use_sim_time,
        sim_main_route_graph,
    ])
