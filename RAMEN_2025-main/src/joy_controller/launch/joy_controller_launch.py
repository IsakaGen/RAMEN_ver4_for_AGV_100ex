from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='joy_controller',
            executable='joy_translate_node'
        ),
        Node(
            package='jetson_motor',
            executable='joy_listener',
        )
    ])