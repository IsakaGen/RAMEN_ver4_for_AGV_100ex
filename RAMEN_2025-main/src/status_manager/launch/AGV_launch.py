from launch import LaunchDescription
from launch_ros.actions import Node
from status_manager import (NAME)

def generate_launch_description():
    return LaunchDescription([
        
        #ステータス管理用ノード
        Node(
            package='status_manager',
            executable='manager',
            namespace=NAME,    
        ),
        
        #自律走行用ノード
        Node(
            package='transition',
            executable='wall_follower_lifecycle',
            namespace=NAME,
        ),
        
        #LiDARノード
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            namespace=NAME,
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }]
        ),
        
        #モーター制御ノード
        Node(
            package='jetson_motor',
            executable='joy_listener',
            namespace=NAME,
        ),
        
        #ボタン入力検知ノード
        Node(
            package='button',
            executable='button_publisher',
            namespace=NAME,
            remappings=[('/Button_pushed', f'/{NAME}_Button_pushed')]
        ),
        
        #目標位置停車ノード
        Node(
            package='move_target',
            executable='target_stop',
            namespace=NAME,
        ),
    ])