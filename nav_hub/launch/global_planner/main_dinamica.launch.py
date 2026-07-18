from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_hub',
            executable='main_dinamica',
            name='main_dinamica_node',
            output='screen',
            parameters=[
                # Adicione parâmetros aqui se necessário
            ]
        )
    ])
