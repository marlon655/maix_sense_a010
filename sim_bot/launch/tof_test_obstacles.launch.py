import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def spawn_obstacle(package_share, name, sdf_file, x, y, z, yaw=0.0):
    model_path = os.path.join(
        package_share,
        'models',
        'tof_test_obstacles',
        sdf_file,
    )

    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        output='screen',
        arguments=[
            '-file', model_path,
            '-name', name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-Y', str(yaw),
        ],
    )


def generate_launch_description():
    package_share = get_package_share_directory('sim_bot')

    obstacles = [
        {
            'name': 'tof_low_curb_3cm',
            'file': 'low_curb_3cm.sdf',
            'x': 0.80,
            'y': 11.75,
            'z': 0.015,
        },
        {
            'name': 'tof_cube_10cm',
            'file': 'cube_10cm.sdf',
            'x': 1.05,
            'y': 12.10,
            'z': 0.05,
        },
        {
            'name': 'tof_box_30cm',
            'file': 'box_30cm.sdf',
            'x': 1.30,
            'y': 12.45,
            'z': 0.15,
        },
        {
            'name': 'tof_tall_panel',
            'file': 'tall_panel.sdf',
            'x': 1.60,
            'y': 12.85,
            'z': 0.40,
        },
        {
            'name': 'tof_wide_panel',
            'file': 'wide_panel.sdf',
            'x': 1.95,
            'y': 13.25,
            'z': 0.25,
        },
        {
            'name': 'tof_ramp_10deg',
            'file': 'ramp_10deg.sdf',
            'x': 0.95,
            'y': 13.55,
            'z': 0.0,
        },
    ]

    actions = [
        LogInfo(
            msg='Spawning ToF test obstacles: low curb, cube, box, panels and ramp.'
        )
    ]

    for obstacle in obstacles:
        actions.append(
            spawn_obstacle(
                package_share,
                obstacle['name'],
                obstacle['file'],
                obstacle['x'],
                obstacle['y'],
                obstacle['z'],
                obstacle.get('yaw', 0.0),
            )
        )

    return LaunchDescription(actions)
