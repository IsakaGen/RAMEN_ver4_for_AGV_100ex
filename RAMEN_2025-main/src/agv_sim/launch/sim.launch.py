#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_agv_sim = get_package_share_directory('agv_sim')

    urdf_file = os.path.join(pkg_agv_sim, 'urdf', 'agv.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])
    world_path = os.path.join(pkg_agv_sim, 'worlds', 'loop_world.world')
    teleop_config = os.path.join(pkg_agv_sim, 'config', 'teleop_config.yaml')  # ← 追加

    return LaunchDescription([
        # Gazebo 起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_path}.items()
        ),

        # robot_state_publisher 起動
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # ロボットスポーン
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'agv',
                '-x', '-3.3',
                '-y', '-0.9',
                '-z', '0.3',
                '-Y', '0.0'
            ],
            output='screen'
        ),

        # ★ジョイコン入力ノード（joy）
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}],
            output='screen'
        ),

        # ★ジョイコン→cmd_vel変換ノード
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[teleop_config],
            output='screen'
        ),

 
      # ★Laser relayノード: /gazebo_ros_ray_sensor/out → /scan
        Node(
            package='topic_tools',
            executable='relay',
            name='laser_relay',
            arguments=['/gazebo_ros_ray_sensor/out', '/scan'],
            output='screen'
        ),      
    ])

