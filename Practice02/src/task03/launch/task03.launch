from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task03',
            executable='trigger_service',
            name='trigger_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                'config/task03.yaml'
            ]
        )
    ])
