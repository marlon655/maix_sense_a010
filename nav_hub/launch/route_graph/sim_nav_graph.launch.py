import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav_hub')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_route = LaunchConfiguration('route')
    use_speed_filter = LaunchConfiguration('speed_filter')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'map_server',
        'speed_filter_mask_server',
        'speed_costmap_filter_info_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'route_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    controller_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('cmd_vel', 'controller_cmd_vel'),
    ]
    smoother_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('cmd_vel', 'controller_cmd_vel'),
        ('cmd_vel_smoothed', 'cmd_vel'),
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'sim_nav_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_route = DeclareLaunchArgument(
        'route',
        default_value='true',
        description='Start nav2_route route_server if true')

    declare_speed_filter = DeclareLaunchArgument(
        'speed_filter',
        default_value='true',
        description='Start Nav2 speed filter servers if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack')

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    localization = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    configured_params,
                    {'use_sim_time': use_sim_time},
                    {'scan_topic': 'scan'},
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        ])

    navigation = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=controller_remappings),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=smoother_remappings),
        ])

    route = GroupAction(
        condition=IfCondition(use_route),
        actions=[
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        ])

    speed_filter = GroupAction(
        condition=IfCondition(use_speed_filter),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='speed_filter_mask_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='speed_costmap_filter_info_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        ])

    lifecycle_manager = GroupAction(
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ]),
        ])

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_route,
        declare_speed_filter,
        declare_autostart,
        declare_log_level,
        localization,
        speed_filter,
        navigation,
        route,
        lifecycle_manager,
    ])
