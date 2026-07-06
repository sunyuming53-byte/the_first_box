from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omr_controller',
            executable='task_orchestrator',
            name='task_orchestrator',
            parameters=[PathJoinSubstitution([
                FindPackageShare('omr_controller'),
                'config', 'controller.yaml',
            ])],
            output='screen',
        ),
    ])
