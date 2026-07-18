import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package name
    package_name='sim_bot'

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Path to configuration file 
    default_slam_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'slam_params.yaml')

    # Declare the launch configuration
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params_file,
        description='Full path to the parameters file to use for the slam_toolbox node')

    # Launch the slam node with launch configs
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # Launch!
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_params_file)
    ld.add_action(slam_node)

    return ld
