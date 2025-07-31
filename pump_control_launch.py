
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pump_ctrl',
            executable='pump_controller.py',
            name='pump_controller',
            output='screen'
        )
    ])
