from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_arm', default_value='true',
                              description='Launch arm driver (ros2_control + state publisher)'),
        DeclareLaunchArgument('launch_camera', default_value='true',
                              description='Launch the RealSense camera node'),
        DeclareLaunchArgument('launch_calib', default_value='false',
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
        DeclareLaunchArgument('screw_lead', default_value='0.01',
                              description='Screw lead (m/rev) for dais linear rail'),
        DeclareLaunchArgument('launch_m65', default_value='true',
                               description='Launch M65 chassis driver (diff_drive_controller + M65BaseHardware)'),
        DeclareLaunchArgument('m65_serial_port', default_value='/dev/ttyBase',
                               description='Serial port for M65 chassis'),
        DeclareLaunchArgument('m65_baud_rate', default_value='115200',
                               description='Baud rate for M65 chassis'),
        DeclareLaunchArgument('wheel_separation', default_value='0.355',
                              description='Wheel separation for M65 chassis'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0625',
                              description='Wheel radius for M65 chassis'),
        DeclareLaunchArgument('encoder_cpr', default_value='0',
                              description='Encoder CPR for M65 chassis'),
        DeclareLaunchArgument('launch_door_trajectory', default_value='false',
                              description='Launch MoveIt2 door trajectory orchestrator (deprecated, use BT XML)'),
        DeclareLaunchArgument('launch_moveit', default_value='false',
                               description='Launch MoveIt2 move_group as a persistent planning service'),
        DeclareLaunchArgument('launch_m65_lio', default_value='true',
                                description='Launch LIO+Nav2 alongside M65 chassis'),
        DeclareLaunchArgument('launch_foxglove', default_value='true',
                              description='Launch foxglove_bridge for browser visualization'),
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0',
                              description='foxglove_bridge bind address'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='foxglove_bridge websocket port'),
        DeclareLaunchArgument('launch_diagnostics', default_value='true',
                              description='Launch aggregate robot diagnostics node'),
        DeclareLaunchArgument('launch_remote_control', default_value='true',
                              description='Launch remote_control service node for Foxglove teleop'),

        # ── robot_description from xacro ─────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                    PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'arm', 'realman.urdf.xacro']), ' ',
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
                        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'arm', 'realman.urdf.xacro']), ' ',
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
        # Humble JTC declares joints/command_interfaces/state_interfaces
        # as read-only (generate_parameter_library).  YAML auto-load
        # can't populate them; ros2 param load injects them before spawn.
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'load', '/controller_manager',
                         PathJoinSubstitution([
                             FindPackageShare('omr_bringup'),
                             'config', 'jtc_inject.yaml',
                         ])],
                    output='screen',
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_arm')),
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'controller_manager', 'spawner',
                         'joint_trajectory_controller',
                         '-c', '/controller_manager'],
                    output='screen',
                ),
            ],
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
                    PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'dais/dais.ros2_control.xacro']),
                    ' serial_port:=', LaunchConfiguration('serial_port'),
                    ' baud_rate:=', LaunchConfiguration('baud_rate'),
                    ' slave_id:=', LaunchConfiguration('slave_id'),
                    ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
                    ' screw_lead:=', LaunchConfiguration('screw_lead'),
                ]),
            }],
            remappings=[('/robot_description', '/robot_description_dais')],
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
                        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'dais/dais.ros2_control.xacro']),
                        ' serial_port:=', LaunchConfiguration('serial_port'),
                        ' baud_rate:=', LaunchConfiguration('baud_rate'),
                        ' slave_id:=', LaunchConfiguration('slave_id'),
                        ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
                        ' screw_lead:=', LaunchConfiguration('screw_lead'),
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
            arguments=['dais_joint_state_broadcaster', '--controller-manager', '/dais_controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_dais')),
        ),

        # ── Dais joint_trajectory_controller spawner ───────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['dais_joint_trajectory_controller', '--controller-manager', '/dais_controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_dais')),
        ),

        # ── M65 robot_state_publisher ──────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='m65_robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ',
                    PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'm65/m65.ros2_control.xacro']),
                    ' serial_port:=', LaunchConfiguration('m65_serial_port'),
                    ' baud_rate:=', LaunchConfiguration('m65_baud_rate'),
                    ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
                    ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
                    ' encoder_cpr:=', LaunchConfiguration('encoder_cpr'),
                ]),
            }],
            remappings=[('/robot_description', '/robot_description_m65')],
            condition=IfCondition(LaunchConfiguration('launch_m65')),
        ),

        # ── M65 controller manager (ros2_control) ─────────────
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='m65_controller_manager',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('omr_bringup'),
                    'config', 'm65_controllers.yaml',
                ]),
                {
                    'robot_description': Command([
                        FindExecutable(name='xacro'), ' ',
PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'm65/m65.ros2_control.xacro']),
                        ' serial_port:=', LaunchConfiguration('m65_serial_port'),
                        ' baud_rate:=', LaunchConfiguration('m65_baud_rate'),
                        ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
                        ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
                        ' encoder_cpr:=', LaunchConfiguration('encoder_cpr'),
                    ]),
                },
            ],
            condition=IfCondition(LaunchConfiguration('launch_m65')),
            output='screen',
        ),

        # ── M65 joint_state_broadcaster spawner ───────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/m65_controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_m65')),
        ),

        # ── M65 diff_drive_controller spawner ────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/m65_controller_manager'],
            condition=IfCondition(LaunchConfiguration('launch_m65')),
        ),

        # ── LIO + Nav2 (launched with M65) ──────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('omr_lio'), 'launch', 'm65_lio_nav.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('launch_m65_lio')),
        ),

        # ── Task orchestrator (optional BT runtime) ─────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('omr_controller'), 'launch', 'controller.launch.py',
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('launch_door_trajectory')),
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
            package='omr_controller',
            executable='calib_node',
            name='calib_node',
            parameters=[{
                'arm_ip': LaunchConfiguration('arm_ip'),
            }],
            condition=IfCondition(LaunchConfiguration('launch_calib')),
        ),

        # ── MoveIt2 move_group (persistent planning service) ─────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('omr_bringup'), 'launch', 'move_group.launch.py',
                ])
            ]),
            launch_arguments={
                'use_rviz': 'false',
                'standalone': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_moveit')),
        ),

        # ── Aggregate diagnostics for Foxglove /diagnostics panel ───────
        Node(
            package='omr_controller',
            executable='robot_diagnostics',
            name='robot_diagnostics',
            parameters=[{
                'monitor_arm': ParameterValue(LaunchConfiguration('launch_arm'), value_type=bool),
                'monitor_dais': ParameterValue(LaunchConfiguration('launch_dais'), value_type=bool),
                'monitor_m65': ParameterValue(LaunchConfiguration('launch_m65'), value_type=bool),
                'monitor_lio': ParameterValue(
                    LaunchConfiguration('launch_m65_lio'), value_type=bool),
                'monitor_orchestrator': ParameterValue(
                    LaunchConfiguration('launch_door_trajectory'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('launch_diagnostics')),
            output='screen',
        ),

        # ── Remote control service node (Foxglove teleop) ─────────────────
        Node(
            package='omr_controller',
            executable='remote_control',
            name='remote_control',
            condition=IfCondition(LaunchConfiguration('launch_remote_control')),
            output='screen',
        ),

        # ── Foxglove websocket bridge ───────────────────────────────────
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'address': LaunchConfiguration('foxglove_address'),
                'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type=int),
            }],
            condition=IfCondition(LaunchConfiguration('launch_foxglove')),
            output='screen',
        ),
    ])
