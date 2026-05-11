from launch import LaunchDescription
from launch_ros.actions import Node
from status_manager import (NAME)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='status_manager',
            executable='manager',
               
        ),
        Node(
            package='joy',
            executable='joy_node'
        ),
        Node(
            package='joy_controller',
            executable='joy_translate_node',
            
        ),
        #Node(
        #    package='transition',
        #    executable='wall_follower_lifecycle',
        #    
        #),
        Node(
            package='jetson_motor',
            executable='joy_listener',
            
        ),
        Node(
            package='button',
            executable='button_publisher',
            remappings=[('/Button_pushed', f'/{NAME}_Button_pushed')]
        )
    ])