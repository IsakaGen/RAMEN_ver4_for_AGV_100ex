from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='button',
            executable='button_publisher',
        ),
        Node(
            package='transition',
            executable='transition_tutorial',
        ),
])