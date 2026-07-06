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
        DeclareLaunchArgument('launch_dais', default_value='true',
                              description='Launch dais motor (ros2_control + state publisher)'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyRS485',
                              description='Serial port for dais motor Modbus RTU'),
        DeclareLaunchArgument('baud_rate', default_value='57600',
                              description='Baud rate for dais motor Modbus RTU'),
        DeclareLaunchArgument('slave_id', default_value='1',
                              description='Modbus slave ID for dais motor'),
        DeclareLaunchArgument('gear_ratio', default_value='1000',
                              description='Gear ratio for dais motor'),

        # ── robot_description from xacro ─────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                    PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'realman.urdf.xacro']), ' ',
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
                    FindPackageShare('omr_bringup'),
                    'config', 'realman_controllers.yaml',
                ]),
                {
                    'robot_description': Command([
                        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'realman.urdf.xacro']), ' ',
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

        # ── Dais robot_state_publisher ──────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='dais_robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ',
                    PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'dais.ros2_control.xacro']),
                    ' serial_port:=', LaunchConfiguration('serial_port'),
                    ' baud_rate:=', LaunchConfiguration('baud_rate'),
                    ' slave_id:=', LaunchConfiguration('slave_id'),
                    ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
                ]),
            }],
            condition=IfCondition(LaunchConfiguration('launch_dais')),
        ),

        # ── Dais controller manager (ros2_control) ─────────────
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='dais_controller_manager',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('omr_bringup'),
                    'config', 'dais_controllers.yaml',
                ]),
                {
                    'robot_description': Command([
                        FindExecutable(name='xacro'), ' ',
                        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'dais.ros2_control.xacro']),
                        ' serial_port:=', LaunchConfiguration('serial_port'),
                        ' baud_rate:=', LaunchConfiguration('baud_rate'),
                        ' slave_id:=', LaunchConfiguration('slave_id'),
                        ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
                    ]),
                },
            ],
            condition=IfCondition(LaunchConfiguration('launch_dais')),
            output='screen',
        ),

        # ── Dais joint_state_broadcaster spawner ───────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/dais_controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_dais')),
        ),

        # ── Dais joint_trajectory_controller spawner ───────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/dais_controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_dais')),
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
