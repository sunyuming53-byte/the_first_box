from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz with MoveIt2 motion planning plugin'),
        DeclareLaunchArgument('launch_arm', default_value='true',
                              description='Launch arm controllers (JSB spawner)'),

        # ── move_group node ─────────────────────────────────
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            parameters=[
                {
                    'robot_description': Command([
                        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                        PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                              'urdf', 'rm65_moveit.urdf.xacro']),
                    ]),
                    'robot_description_semantic': Command([
                        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                        PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                              'config', 'rm65.srdf']),
                    ]),
                    'use_sim_time': False,
                    'publish_robot_description_semantic': True,
                },
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'kinematics.yaml']),
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'ompl_planning.yaml']),
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'controllers.yaml']),
            ],
            output='screen',
        ),

        # ── joint_state_broadcaster spawner ────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
        ),

        # ── RViz with MoveIt2 Motion Planning plugin ───────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('rm65_moveit_config'), 'config', 'moveit.rviz',
            ])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
