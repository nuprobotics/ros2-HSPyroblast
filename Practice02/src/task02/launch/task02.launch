from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task02',
            executable='publisher',
            name='publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                'config/task02.yaml'
            ]
        )
    ])
