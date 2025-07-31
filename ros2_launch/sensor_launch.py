
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor',
            executable='sensor_node.py',
            name='sensor_node',
            output='screen'
        )
    ])
