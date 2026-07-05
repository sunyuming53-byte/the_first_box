from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_arm', default_value='true',
                              description='Launch arm driver (ros2_control + state publisher)'),
        DeclareLaunchArgument('launch_camera', default_value='true',
                              description='Launch the RealSense camera node'),
        DeclareLaunchArgument('launch_calib', default_value='true',
                              description='Launch the calibration node'),
        DeclareLaunchArgument('arm_ip', default_value='192.168.1.18',
                              description='IP address of the robot arm'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to hand-eye calibration transform file'),

        # ── robot_description from xacro ─────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                    PathJoinSubstitution([FindPackageShare('realman_bringup'), 'urdf', 'realman.urdf.xacro']), ' ',
                    'arm_ip:=', LaunchConfiguration('arm_ip'),
                ]),
            }],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
        ),

        # ── Controller manager (ros2_control) ────────────────────
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('realman_bringup'),
                    'config', 'realman_controllers.yaml',
                ]),
                {
                    'robot_description': Command([
                        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                        PathJoinSubstitution([FindPackageShare('realman_bringup'), 'urdf', 'realman.urdf.xacro']), ' ',
                        'arm_ip:=', LaunchConfiguration('arm_ip'),
                    ]),
                },
            ],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
            output='screen',
        ),

        # ── joint_state_broadcaster spawner ──────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
        ),

        # ── joint_trajectory_controller spawner ──────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
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
