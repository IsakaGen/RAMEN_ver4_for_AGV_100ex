from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # xacro 展開.
    urdf_path = os.path.join(
    get_package_share_directory('agv_sim'),
    'urdf',
    'agv.urdf.xacro'
    )
    doc = xacro.process_file(urdf_path, mappings={'use_gazebo': 'false'})
    robot_description = {'robot_description': doc.toxml()}
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Robot state publisher.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                robot_description
            ]
        ),

        # SLLIDAR A2M12 LiDARノード.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
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
        
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',  # これ必須.
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/odom_laser',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'freq': 10.0
            }],
        remappings=[
            ('laser_scan', '/scan')  #相対パス.
        ]
        ),
        Node(package='right_hand_controller', executable='wall_follower2'),
        Node(package='jetson_motor', executable='joy_listener', name='motor_controller_node',output='screen',prefix='python3'),

        Node(
            package='encoder_odometry',
            executable='encoder_odometry_node',
            name='encoder_odometry_node',
            output='screen'
        )


        #Node(package='joy', executable='joy_node'),
        #Node(package='joy_controller', executable='joy_translate_node'),
    ])

