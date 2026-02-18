from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speak',
            executable='sender',
            name='senderr',
            output='screen'
        ),
        Node(
            package='speak',
            executable='process', 
            name='process',
            output='screen'
        )
    ])