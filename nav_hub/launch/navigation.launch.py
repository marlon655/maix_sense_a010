from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
import os

def generate_launch_description():
    feedback_pkg_share = FindPackageShare('mobby_feedback').find('mobby_feedback')

    feedback = os.path.join(feedback_pkg_share, 'feedback.launch.py')

    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(feedback)
        ),
        Node (
            package='nav_hub',
            executable='main_route_graph',
            output='screen',
        ),
        Node (
            package='nav_hub',
            executable='botoeira_ponto_ponto_linear',
            output='screen',
        ),
        # Node (
        #     package='sipeed_tof_ms_a010',
        #     executable='sipeed_tof_node',
        #     output='screen'
        # )
        # Node (
        #     package='nav_hub',
        #     executable='stop_obstacle',
        #     output='screen',
        # )
    ])
