from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
import os

def generate_launch_description():
    pkg_share = FindPackageShare('nav_hub').find('nav_hub')
    feedback_pkg_share = FindPackageShare('mobby_feedback').find('mobby_feedback')
    # serial_communication_pkg_share = FindPackageShare('comunication_serial').find('comunication_serial')
    
    essential_linear = os.path.join(pkg_share, 'launch', 'speed_limit+tof/essential.launch.py')
    nav_linear = os.path.join(pkg_share, 'launch', 'speed_limit+tof/nav.launch.py')
    feedback = os.path.join(feedback_pkg_share, 'feedback.launch.py')
    # communication = os.path.join(serial_communication_pkg_share, 'communicator_serial.launch.py')
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(communication)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(feedback)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(essential_linear)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_linear)
        ),
       
    ])
