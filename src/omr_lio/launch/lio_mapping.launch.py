"""
ROS2 launch file for omr_lio mapping.

Usage:
  ros2 launch omr_lio lio_mapping.launch.py
  ros2 launch omr_lio lio_mapping.launch.py config_file:=my_config.yaml
  ros2 launch omr_lio lio_mapping.launch.py launch_livox_driver:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='lio.yaml',
        description='Config YAML file in omr_lio/config/'
    )

    launch_livox_driver_arg = DeclareLaunchArgument(
        'launch_livox_driver', default_value='false',
        description='Launch livox_ros_driver2 alongside LIO'
    )

    launch_estopper_arg = DeclareLaunchArgument(
        'launch_estopper', default_value='false',
        description='Launch estopper node'
    )

    # ── LIO mapping node ──
    lio_node = Node(
        package='omr_lio',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('omr_lio'), 'config',
            LaunchConfiguration('config_file')
        ])],
    )

    # ── Livox driver (optional) ──
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_livox_driver')),
        parameters=[PathJoinSubstitution([
            FindPackageShare('omr_lio'), 'config',
            LaunchConfiguration('config_file')
        ])],
    )

    return LaunchDescription([
        config_file_arg,
        launch_livox_driver_arg,
        launch_estopper_arg,
        lio_node,
        livox_node,
    ])
