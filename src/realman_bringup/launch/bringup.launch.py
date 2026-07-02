from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_arm', default_value='true',
                              description='Launch the arm driver node'),
        DeclareLaunchArgument('launch_camera', default_value='true',
                              description='Launch the RealSense camera node'),
        DeclareLaunchArgument('launch_calib', default_value='true',
                              description='Launch the calibration node'),
        DeclareLaunchArgument('arm_ip', default_value='192.168.1.18',
                              description='IP address of the robot arm'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to hand-eye calibration transform file'),

        # ── Arm driver ────────────────────────────────────────────
        Node(
            package='realman_driver',
            executable='arm_node',
            name='arm_node',
            parameters=[{
                'arm_ip': LaunchConfiguration('arm_ip'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
        ),

        # ── RealSense camera ──────────────────────────────────────
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),

        # ── Calibration pipeline ──────────────────────────────────
        Node(
            package='realman_calibration',
            executable='calib_node',
            name='calib_node',
            parameters=[{
                'arm_ip': LaunchConfiguration('arm_ip'),
            }],
            condition=IfCondition(LaunchConfiguration('launch_calib')),
        ),
    ])
