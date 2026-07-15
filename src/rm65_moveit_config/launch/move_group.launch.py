import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    package_share = get_package_share_directory(package_name)
    file_path = os.path.join(package_share, relative_path)
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'omr.urdf.xacro']),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                              'config', 'rm65_full.srdf']),
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str),
    }

    kinematics_params = load_yaml('rm65_moveit_config', 'config/kinematics.yaml')
    ompl_params = load_yaml('rm65_moveit_config', 'config/ompl_planning.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('launch_arm', default_value='true'),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),

        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': False}],
            condition=UnlessCondition(LaunchConfiguration('launch_arm')),
        ),

        Node(
            package='moveit_ros_move_group', executable='move_group',
            name='move_group',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'use_sim_time': False, 'publish_robot_description_semantic': True},
                kinematics_params,
                ompl_params,
                {
                    'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                    'controller_names': ['arm_cm'],
                    'arm_cm': {
                        'type': 'FollowJointTrajectory',
                        'action_ns': 'joint_trajectory_controller/follow_joint_trajectory',
                        'default': True,
                        'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                        'action_monitor_rate': 20.0,
                    },
                },
            ],
            output='screen',
        ),

        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
        ),

        Node(
            package='rviz2', executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('rm65_moveit_config'), 'config', 'moveit.rviz',
            ])],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_params,
                ompl_params,
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
