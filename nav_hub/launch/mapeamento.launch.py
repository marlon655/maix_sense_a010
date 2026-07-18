#!/usr/bin/python3
# -*- coding: utf-8 -*-

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command 
import os

def generate_launch_description():
    
    # carrega URDF
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot06_description'), 'launch', 'display.launch.py'))
    )

    # Launch do Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'stl27l.launch.py'))
    )

    # Launch do CANopen
    canopen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('canopen_ros'), 'launch', 'canopen.launch.py'))
    )

    # Launch do slam toolbox online_async_launch
    slamToolBox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'))
    )

    # Launch do slam toolbox lifelong_launch
    # slamToolBox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('slam_toolbox'), 'launch', 'lifelong_launch.py'))
    # )

    

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        description_launch,
        lidar_launch,
        canopen_launch,
        slamToolBox_launch
    ])
