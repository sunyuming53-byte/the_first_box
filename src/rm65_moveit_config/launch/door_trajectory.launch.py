from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_door_trajectory', default_value='false',
                              description='Launch door trajectory feed node'),

        # ── Include move_group (no RViz) ──────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rm65_moveit_config'), 'launch', 'move_group.launch.py',
                ]),
            ]),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),

        # ── Door trajectory feed node ─────────────────────
        Node(
            package='omr_controller',
            executable='door_trajectory_node',
            name='door_trajectory_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('rm65_moveit_config'), 'config', 'door_trajectory.yaml',
            ])],
            condition=IfCondition(LaunchConfiguration('launch_door_trajectory')),
            output='screen',
        ),
    ])
