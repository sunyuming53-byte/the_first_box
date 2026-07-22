from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dais_joint_name', default_value='joint_dais'),
        DeclareLaunchArgument('photogate_topic', default_value='/photogate/gate0/blocked'),
        Node(
            package='omr_controller',
            executable='dais_positioning_node',
            name='dais_positioning_node',
            parameters=[{
                'dais_joint_name': LaunchConfiguration('dais_joint_name'),
                'photogate_topic': LaunchConfiguration('photogate_topic'),
            }],
            output='screen',
        ),
    ])
