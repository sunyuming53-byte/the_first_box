#!/usr/bin/env python3
"""
Launch robot_state_publisher + joint_state_publisher_gui + rviz2
to visualize the M65 chassis URDF.

Usage:
  ros2 launch omr_bringup view_m65.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'omr.urdf.xacro']),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('omr_bringup'), 'config', 'view_robot.rviz',
            ])],
        ),
    ])
