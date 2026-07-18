import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'sim_bot'
    package_share = get_package_share_directory(package_name)

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
    use_sim_time = LaunchConfiguration('use_sim_time')

    models_path = os.path.join(package_share, 'models')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_share, 'worlds', 'test.world'),
        description='Full path to the world model file to load')

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='palmares_bot',
        description='Robot name used when spawning in Gazebo')

    declare_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        default_value=os.path.join(package_share, 'description', 'palmares_bot.urdf.xacro'),
        description='Full path to the robot URDF/Xacro file')

    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-0.2621192932128906',
        description='Initial Gazebo spawn X position')

    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='12.350018501281738',
        description='Initial Gazebo spawn Y position')

    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.15',
        description='Initial Gazebo spawn Z position')

    declare_bridge_config = DeclareLaunchArgument(
        'bridge_config',
        default_value=os.path.join(package_share, 'config', 'gz_bridge.yaml'),
        description='Full path to the ros_gz_bridge YAML config')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_share, 'rviz', 'bot.rviz'),
        description='Full path to the RViz config file')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo headless if true')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Open RViz if true')

    declare_joy = DeclareLaunchArgument(
        'joy',
        default_value='False',
        description='Enable joystick tele-operation if true')

    declare_tof_lidar = DeclareLaunchArgument(
        'tof_lidar',
        default_value='True',
        description='Enable the simulated 3D ToF lidar point cloud sensor')

    declare_camera = DeclareLaunchArgument(
        'camera',
        default_value='True',
        description='Enable the simulated RGB camera sensor')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    gazebo_models_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path,
        separator=':')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', robot_urdf, ' tof_lidar:=', tof_lidar, ' camera:=', camera]),
                value_type=str),
        }])

    joystick = GroupAction(
        condition=IfCondition(joy),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, 'launch', 'teleop.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items())
        ])

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', world],
            'on_exit_shutdown': 'true',
        }.items())

    gazebo_client = GroupAction(
        condition=IfCondition(PythonExpression(['not ', headless])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': '-g '}.items())
        ])

    spawn_palmares_bot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        output='screen')

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', ['config_file:=', bridge_config]])

    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                remappings=[
                    ('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose'),
                ])
        ])

    return LaunchDescription([
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
        declare_use_sim_time,
        gazebo_models_path,
        rviz2,
        robot_state_publisher,
        joystick,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_palmares_bot,
    ])
