from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the four runtime nodes (no Gazebo / sim‑time support)."""

    return LaunchDescription([
        # ──────────────────────────────────────────────
        # 1) SLLIDAR A2M12 LiDAR driver
        # ──────────────────────────────────────────────
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[
                {
                    "channel_type": "serial",
                    "serial_port": "/dev/ttyUSB0",
                    "serial_baudrate": 256000,
                    "frame_id": "laser",
                    "inverted": False,
                    "angle_compensate": True,
                    "scan_mode": "Sensitivity",
                }
            ],
        ),

        # ──────────────────────────────────────────────
        # 2) RF2O laser odometry (2‑D LiDAR odometry)
        # ──────────────────────────────────────────────
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,   # 常に実時間で動作
                    "odom_topic": "/odom_laser",
                    "publish_tf": True,
                    "base_frame_id": "base_link",
                    "odom_frame_id": "odom",
                    "freq": 10.0,
                }
            ],
            remappings=[
                ("laser_scan", "/scan"),
            ],
        ),

        # ──────────────────────────────────────────────
        # 3) Wall‑following controller
        # ──────────────────────────────────────────────
        Node(
            package="transition",
            executable='transition_tutorial',
            name="wall_follower_lifecycle",
            output="screen",
        ),

        # ──────────────────────────────────────────────
        # 4) Jetson motor interface (joy‑listener)
        # ──────────────────────────────────────────────
        Node(
            package="jetson_motor",
            executable="joy_listener",
            name="motor_controller_node",
            output="screen",
            prefix="python3",
        ),
        Node(
            package='status_manager',
            executable='manager',
               
        ),
    ])
