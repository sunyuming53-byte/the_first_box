from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    launch_moveit = LaunchConfiguration('launch_moveit')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'sim_combined.urdf.xacro']),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('launch_moveit', default_value='true'),

        # Gazebo Fortress resource path for package:// → model:// resolution
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               PathJoinSubstitution([FindPackageShare('omr_bringup'), '..'])),

        # ── Gazebo Sim (Fortress) ─────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments=[('gz_args', ['-r empty.sdf'])],
        ),

        # ── Clock bridge (Gazebo → ROS2 /clock) ──────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),

        # ── robot_state_publisher ────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str), 'use_sim_time': True}],
        ),

        # ── Spawn robot ──────────────────────────────────
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_omr',
            arguments=['-topic', 'robot_description', '-name', 'omr'],
        ),

        # ── Camera bridge (Fortress → ROS2) ─────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            remappings=[
                ('/camera', '/camera/color/image_raw'),
                ('/camera_info', '/camera/color/camera_info'),
            ],
            output='screen',
        ),

        # ── Controller spawners (delayed 3s for robot to load) ──
        TimerAction(period=3.0, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/dais_controller_manager']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_trajectory_controller', '--controller-manager', '/dais_controller_manager']),
        ]),

        # ── MoveIt2 move_group (optional) ────────────────
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
                    'use_sim_time': True,
                    'publish_robot_description_semantic': True,
                },
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'kinematics.yaml']),
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'ompl_planning.yaml']),
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'controllers.yaml']),
            ],
            condition=IfCondition(launch_moveit),
            output='screen',
        ),
    ])
